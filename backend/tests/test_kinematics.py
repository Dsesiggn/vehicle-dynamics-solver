"""Vehicle rig integration: independent signs, geometry and derivative checks.

These tests use explicit synthetic static alignment, rocker axes and rig
references. They do not infer those values for user designs or constitute
physical validation against vehicle/OptimumKinematics measurements.
"""
from copy import deepcopy
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import unittest

import numpy as np
from fastapi.testclient import TestClient
from pydantic import ValidationError

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vehicle_dynamics.app import create_app
from vehicle_dynamics.kinematics_schemas import KinematicsSweepRequest
from vehicle_dynamics.models.kinematics import prescribed_bump, sweep_kinematics


NODE = shutil.which("node")
ROOT = Path(__file__).resolve().parents[2]


@unittest.skipUnless(NODE, "Node required to exercise canonical Studio design boundary")
class KinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        code = """import {newDesign,axle,generatePoints} from './suspension-studio/src/model.js';
const designs={};for(const topology of ['double-wishbone','macpherson','multi-link']) {
 const d=newDesign();d.axles.front=axle(topology);d.axles.rear=axle(topology,1180,false);
 designs[topology]=d;
}console.log(JSON.stringify(designs));"""
        completed = subprocess.run([NODE, "--input-type=module", "-e", code], cwd=ROOT,
                                   text=True, capture_output=True, check=True)
        cls.sources = json.loads(completed.stdout)
        cls.client = TestClient(create_app())

    def design(self, topology="double-wishbone", *, zero_alignment=True):
        d = deepcopy(self.sources[topology])
        d["kinematics"] = {"contactModel": "rigid-circle", "reference": {"x": -d["wheelbase"]/2, "y": 0},
                           "axles": {name: {"staticCamberDeg": 0 if zero_alignment else -1.3,
                                            "staticToeInDeg": 0 if zero_alignment else .2,
                                            "rockerAxis": [1, 0, 0]} for name in ("front", "rear")}}
        return d

    def request(self, design=None, mode="heave", low=-5, high=5, samples=3):
        return {"design": design or self.design(), "sweep": {"mode": mode, "min": low, "max": high, "samples": samples}}

    def run_sweep(self, design=None, mode="heave", low=-5, high=5, samples=3):
        request = KinematicsSweepRequest.model_validate(self.request(design, mode, low, high, samples)).model_dump()
        return sweep_kinematics(request["design"], request["sweep"])

    def finite_json(self, value):
        json.dumps(value, allow_nan=False)

    def test_explicit_rig_formula_signs_exact_tangent_and_derivative(self):
        reference = {"x": -800, "y": 0}
        for angle in (-8.0, -2.0, 0.0, 2.0, 8.0):
            tangent = math.tan(math.radians(angle))
            scale = math.pi/180 / math.cos(math.radians(angle))**2
            for station, pitch_arm, roll_arm in (([0, -600, -230], -800, -600),
                                                 ([-1600, 590, -230], 800, 590)):
                self.assertEqual(prescribed_bump("heave", angle, station, reference), (angle, 1))
                b, rate = prescribed_bump("pitch", angle, station, reference)
                self.assertAlmostEqual(b, pitch_arm*tangent, places=12)
                self.assertAlmostEqual(rate, pitch_arm*scale, places=12)
                b, rate = prescribed_bump("roll", angle, station, reference)
                self.assertAlmostEqual(b, roll_arm*tangent, places=12)
                self.assertAlmostEqual(rate, roll_arm*scale, places=12)
            self.assertEqual(prescribed_bump("steering", angle, [0, 0, 0], reference), (0, 0))

    def test_all_modes_and_topologies_produce_consistent_finite_channel_sets(self):
        for topology in self.sources:
            for mode, extent in (("heave", 5), ("pitch", .5), ("roll", .5), ("steering", 5)):
                with self.subTest(topology=topology, mode=mode):
                    result = self.run_sweep(self.design(topology), mode, -extent, extent)
                    self.assertEqual(result["model"], "rigid_suspension_v1")
                    self.assertEqual(result["mode"], mode)
                    ids = [channel["id"] for channel in result["channels"]]
                    self.assertEqual(len(ids), len(set(ids)))
                    self.assertEqual(result["input_unit"], "deg" if mode in ("roll", "pitch") else "mm")
                    self.assertEqual([row["input"] for row in result["rows"]], [-extent, 0, extent])
                    for row in result["rows"]:
                        self.assertEqual(set(ids), set(row["values"]))
                        self.assertEqual(set(row["unavailable"]), {key for key, value in row["values"].items() if value is None})
                        self.assertTrue(all(isinstance(reason, str) and reason for reason in row["unavailable"].values()))
                        for axle in ("front", "rear"):
                            self.assertTrue(row["poses"][axle]["ok"])
                            for side in ("left", "right"):
                                for metric in ("camber", "toe", "motionRatio", "compression", "wheelX", "wheelY", "wheelZ", "contactX", "contactY", "contactZ"):
                                    self.assertIsNotNone(row["values"][f"{axle}.{side}.{metric}"])
                        self.assertLess(row["diagnostics"]["maxResidualMm"], 1e-6)
                    self.finite_json(result)

    def test_input_snapshot_preserved_and_result_not_aliased(self):
        d = self.design()
        before = deepcopy(d)
        result = self.run_sweep(d)
        self.assertEqual(d, before)
        result["rows"][1]["poses"]["front"]["left"]["points"]["wheel_center"][0] = 999
        self.assertEqual(d, before)

    def test_rear_station_local_offset_and_front_frame_translation_once(self):
        d = self.design()
        for point in d["axles"]["rear"]["hardpoints"].values():
            point[0] += 50
        d["kinematics"]["reference"]["x"] = -700
        result = self.run_sweep(d, "pitch", -1, 1)
        neutral = result["rows"][1]
        for side in ("left", "right"):
            self.assertAlmostEqual(neutral["values"][f"rear.{side}.wheelX"], -1550, delta=1e-8)
            self.assertAlmostEqual(neutral["poses"]["rear"][side]["points"]["wheel_center"][0], 50, delta=1e-8)
            self.assertAlmostEqual(neutral["values"][f"{side}.wheelbase"], 1550, delta=1e-8)
        for row in result["rows"]:
            angle = math.radians(row["input"])
            for side in ("left", "right"):
                self.assertAlmostEqual(row["values"][f"front.{side}.bump"], -700*math.tan(angle), delta=1e-10)
                self.assertAlmostEqual(row["values"][f"rear.{side}.bump"], 850*math.tan(angle), delta=1e-10)

    def test_roll_reference_is_explicit_and_uses_static_not_migrated_stations(self):
        d = self.design()
        d["kinematics"]["reference"]["y"] = 35
        result = self.run_sweep(d, "roll", -2, 2)
        for row in result["rows"]:
            for axle in ("front", "rear"):
                for side, sign in (("left", -1), ("right", 1)):
                    expected = (sign*d["axles"][axle]["track"]/2-35)*math.tan(math.radians(row["input"]))
                    self.assertAlmostEqual(row["values"][f"{axle}.{side}.bump"], expected, delta=1e-10)
                    self.assertAlmostEqual(row["values"][f"{axle}.{side}.wheelZ"], -230-expected, delta=1e-8)

    def test_heave_symmetry_and_roll_reversal_mirror(self):
        d = self.design(zero_alignment=False)
        heave = self.run_sweep(d)
        for row in heave["rows"]:
            for axle in ("front", "rear"):
                for metric in ("camber", "toe", "compression", "motionRatio", "wheelX", "wheelZ"):
                    self.assertAlmostEqual(row["values"][f"{axle}.left.{metric}"], row["values"][f"{axle}.right.{metric}"], delta=1e-7)
                self.assertAlmostEqual(row["values"][f"{axle}.left.wheelY"], -row["values"][f"{axle}.right.wheelY"], delta=1e-7)
        roll = self.run_sweep(d, "roll", -1, 1)
        negative, positive = roll["rows"][0], roll["rows"][-1]
        for axle in ("front", "rear"):
            for metric in ("camber", "toe", "compression", "motionRatio", "bump"):
                self.assertAlmostEqual(negative["values"][f"{axle}.left.{metric}"], positive["values"][f"{axle}.right.{metric}"], delta=1e-7)

    def test_static_alignment_recovered_and_delta_channels_zero(self):
        d = self.design(zero_alignment=False)
        row = self.run_sweep(d)["rows"][1]
        for axle in ("front", "rear"):
            for side in ("left", "right"):
                entity = f"{axle}.{side}"
                self.assertAlmostEqual(row["values"][entity+".camber"], -1.3, delta=1e-12)
                self.assertAlmostEqual(row["values"][entity+".toe"], .2, delta=1e-12)
                self.assertAlmostEqual(row["values"][entity+".deltaCamber"], 0, delta=1e-12)
                self.assertAlmostEqual(row["values"][entity+".deltaToe"], 0, delta=1e-12)
            self.assertAlmostEqual(row["values"][axle+".totalToe"], .4, delta=1e-12)

    def test_modal_sensitivities_match_halved_central_differences(self):
        d = self.design(zero_alignment=False)
        for mode, center, h in (("heave", 8.0, .02), ("pitch", .6, .002), ("roll", .8, .002), ("steering", 8.0, .002)):
            differences = []
            for delta in (h, h/2):
                rows = self.run_sweep(d, mode, center-delta, center+delta)["rows"]
                low, mid, high = (row["values"] for row in rows)
                maximum = 0
                for axle in ("front", "rear"):
                    for side in ("left", "right"):
                        for metric, derivative in (("camber", "camberModeGain"), ("toe", "toeModeGain"), ("compression", "compressionModeGain")):
                            key = f"{axle}.{side}."
                            fd = (high[key+metric]-low[key+metric])/(2*delta)
                            discrepancy = abs(fd-mid[key+derivative])
                            maximum = max(maximum, discrepancy)
                            self.assertLess(discrepancy, 3e-6, (mode, key, metric, fd, mid[key+derivative]))
                differences.append(maximum)
            self.assertLess(differences[1], differences[0]*.4+5e-8)

    def test_local_motion_ratio_is_dc_db_not_total_compression_quotient(self):
        d = self.design()
        rows = self.run_sweep(d, "heave", 10, 10.02)["rows"]
        for axle in ("front", "rear"):
            for side in ("left", "right"):
                entity = f"{axle}.{side}."
                fd = (rows[-1]["values"][entity+"compression"]-rows[0]["values"][entity+"compression"])/.02
                mr = rows[1]["values"][entity+"motionRatio"]
                self.assertAlmostEqual(mr, fd, delta=1e-7)
                self.assertAlmostEqual(mr, rows[1]["values"][entity+"compressionModeGain"], delta=1e-12)
                quotient = rows[1]["values"][entity+"compression"]/rows[1]["values"][entity+"bump"]
                self.assertGreater(abs(mr-quotient), 1e-4)

    def test_pitch_and_roll_damper_gain_has_mm_per_degree_units_and_chain_rule(self):
        d = self.design()
        for mode in ("pitch", "roll"):
            result = self.run_sweep(d, mode, -.8, .8)
            for channel in result["channels"]:
                if channel["plot"] == "compressionModeGain":
                    self.assertEqual(channel["unit"], "mm/deg")
                elif channel["plot"] == "motionRatio":
                    self.assertEqual(channel["unit"], "mm/mm")
            for row in result["rows"]:
                for axle in ("front", "rear"):
                    for side, sign in (("left", -1), ("right", 1)):
                        station = [0 if axle == "front" else -1600, sign*d["axles"][axle]["track"]/2, -230]
                        _, rate = prescribed_bump(mode, row["input"], station, d["kinematics"]["reference"])
                        prefix = f"{axle}.{side}."
                        self.assertAlmostEqual(row["values"][prefix+"compressionModeGain"],
                                               row["values"][prefix+"motionRatio"]*rate, delta=1e-9)

    def test_static_planar_roll_center_matches_independent_arm_line_construction(self):
        d = self.design()
        for a in d["axles"].values():
            p = a["hardpoints"]
            # Same lower-arm radial line in YZ makes tie length stationary for
            # the pure-X instantaneous wheel rotation; the X lever fixes steer.
            p["tie_rod_inner"] = [90, p["lca_front"][1], p["lca_front"][2]]
            p["tie_rod_outer"] = [90, p["lca_outer"][1], p["lca_outer"][2]]
            a["rackLength"] = -2*p["tie_rod_inner"][1]
        row = self.run_sweep(d)["rows"][1]
        for axle in ("front", "rear"):
            p = d["axles"][axle]["hardpoints"]
            y_l, z_l = p["lca_front"][1:]
            y_lo, z_lo = p["lca_outer"][1:]
            y_u, z_u = p["uca_front"][1:]
            y_uo, z_uo = p["uca_outer"][1:]
            ml, mu = (z_lo-z_l)/(y_lo-y_l), (z_uo-z_u)/(y_uo-y_u)
            bl, bu = z_l-ml*y_l, z_u-mu*y_u
            y_ic = (bu-bl)/(ml-mu)
            z_ic = ml*y_ic+bl
            contact_y = p["wheel_center"][1]
            expected_z = z_ic*(0-contact_y)/(y_ic-contact_y)
            self.assertIsNotNone(row["values"][axle+".rcZ"], row["unavailable"].get(axle+".rcZ"))
            self.assertAlmostEqual(row["values"][axle+".rcZ"], expected_z, delta=1e-6)
            self.assertAlmostEqual(row["values"][axle+".rcY"], 0, delta=1e-6)
            self.assertAlmostEqual(row["values"][axle+".rcX"], 0 if axle == "front" else -1600, delta=1e-6)
            self.assertAlmostEqual(row["values"][axle+".rcHeight"], -expected_z, delta=1e-6)

    def test_missing_alignment_does_not_invent_camber_toe_contacts_or_centers(self):
        d = self.design()
        d["kinematics"]["axles"]["front"]["staticCamberDeg"] = None
        result = self.run_sweep(d)
        for row in result["rows"]:
            self.assertTrue(row["poses"]["front"]["ok"])
            for side in ("left", "right"):
                for metric in ("camber", "toe", "steer", "deltaCamber", "deltaToe", "camberGain", "toeGain", "camberModeGain", "contactX", "contactY", "contactZ", "scrub", "trail"):
                    key = f"front.{side}.{metric}"
                    self.assertIsNone(row["values"][key])
                    self.assertTrue(row["unavailable"][key])
                for metric in ("bump", "wheelZ", "compression", "motionRatio"):
                    self.assertIsNotNone(row["values"][f"front.{side}.{metric}"])
                self.assertIsNotNone(row["values"][f"rear.{side}.camber"])
            self.assertIsNone(row["values"]["front.rcZ"])

    def test_missing_axis_preserves_pose_and_alignment_but_no_fake_actuator(self):
        d = self.design()
        d["kinematics"]["axles"]["front"]["rockerAxis"] = [None, None, None]
        for row in self.run_sweep(d)["rows"]:
            self.assertTrue(row["poses"]["front"]["ok"])
            for side in ("left", "right"):
                for metric in ("compression", "motionRatio", "compressionModeGain", "rockerAngle", "damperLength"):
                    key = f"front.{side}.{metric}"
                    self.assertIsNone(row["values"][key])
                    self.assertIn("axis", row["unavailable"][key])
                self.assertIsNotNone(row["values"][f"front.{side}.camber"])
                self.assertIsNotNone(row["values"][f"rear.{side}.motionRatio"])

    def test_unreachable_corners_leave_gaps_and_do_not_clip_rig_commands(self):
        d = self.design()
        result = self.run_sweep(d, "pitch", -10, 10)
        self.assertEqual(len(result["rows"]), 3)
        for row in (result["rows"][0], result["rows"][-1]):
            for axle in ("front", "rear"):
                self.assertFalse(row["poses"][axle]["ok"])
                for side in ("left", "right"):
                    self.assertFalse(row["poses"][axle][side]["ok"])
                    self.assertIsNone(row["values"][f"{axle}.{side}.camber"])
                    self.assertIn("100", row["unavailable"][f"{axle}.{side}.camber"])
            self.assertIsNone(row["diagnostics"]["maxResidualMm"])
        self.assertTrue(result["rows"][1]["poses"]["front"]["ok"])
        self.finite_json(result)

    def test_multilink_virtual_steering_axis_is_not_fabricated(self):
        for row in self.run_sweep(self.design("multi-link"))["rows"]:
            for axle in ("front", "rear"):
                for side in ("left", "right"):
                    for metric in ("caster", "kpi", "trail", "scrub"):
                        key = f"{axle}.{side}.{metric}"
                        self.assertIsNone(row["values"][key])
                        self.assertIn("virtual", row["unavailable"][key])

    def test_steering_rack_holds_nonsteered_rear_fixed(self):
        result = self.run_sweep(mode="steering", low=-10, high=10)
        for row in result["rows"]:
            for side in ("left", "right"):
                rear = row["poses"]["rear"][side]
                self.assertEqual(rear["rackMm"], 0)
                for metric in ("bump", "compression", "deltaCamber", "deltaToe", "camberModeGain", "toeModeGain", "compressionModeGain"):
                    self.assertAlmostEqual(row["values"][f"rear.{side}.{metric}"], 0, delta=1e-10)
            self.assertEqual(row["poses"]["front"]["left"]["rackMm"], row["input"])

    def test_physical_singular_geometry_is_failure_not_http_input_error(self):
        d = self.design()
        d["axles"]["front"]["hardpoints"]["uca_front"] = d["axles"]["front"]["hardpoints"]["uca_rear"][:]
        payload = self.request(d)
        KinematicsSweepRequest.model_validate(payload)
        response = self.client.post("/api/v1/kinematics/sweep", json=payload)
        self.assertEqual(response.status_code, 200, response.text)
        for row in response.json()["rows"]:
            self.assertFalse(row["poses"]["front"]["ok"])
            self.assertTrue(row["poses"]["rear"]["ok"])
            self.assertIsNone(row["values"]["front.left.camber"])

    def test_api_success_health_openapi_and_current_cache_policy(self):
        response = self.client.post("/api/v1/kinematics/sweep", json=self.request())
        self.assertEqual(response.status_code, 200, response.text)
        self.assertEqual(response.headers.get("cache-control"), "no-store")
        result = response.json()
        self.assertEqual(result["coordinate_system"], "SAE_J670_Z_DOWN")
        self.assertEqual(result["hardpoint_frame"], "AXLE_LOCAL")
        self.assertEqual(result["length_unit"], "mm")
        self.assertEqual(result["angle_unit"], "deg")
        self.assertEqual(len(result["rows"]), 3)
        self.assertIn("rigid_suspension_v1", self.client.get("/api/v1/health").json()["models"])
        self.assertIn("/api/v1/kinematics/sweep", self.client.get("/api/v1/openapi.json").json()["paths"])
        self.finite_json(result)

    def test_actual_python_response_passes_browser_boundary(self):
        # Integration contract, including vehicle-side channels and unavailable
        # corner geometry; synthetic hand-written JSON would miss mismatches.
        for missing in (False, True):
            request = self.request()
            if missing:
                request["design"]["kinematics"]["axles"]["rear"]["staticCamberDeg"] = None
                request["design"]["kinematics"]["axles"]["front"]["rockerAxis"] = [None, None, None]
            response = self.client.post("/api/v1/kinematics/sweep", json=request)
            self.assertEqual(response.status_code, 200)
            code = """import {validateKinematicsSweepResponse} from './suspension-studio/src/kinematics-api.js';
let input='';for await(const chunk of process.stdin)input+=chunk;
const {response,request}=JSON.parse(input);validateKinematicsSweepResponse(response,request);"""
            process = subprocess.run([NODE,"--input-type=module","-e",code],input=json.dumps({"request":request,"response":response.json()}),text=True,capture_output=True,cwd=ROOT)
            self.assertEqual(process.returncode,0,process.stderr)

    def test_strict_schema_rejects_unknown_frames_points_and_missing_references(self):
        base = self.request()
        changes = [("coordinateSystem", "Z_UP"), ("hardpointFrame", "GLOBAL"), ("lengthUnit", "in"), ("version", 1), ("name", "   ")]
        for key, value in changes:
            payload = deepcopy(base)
            payload["design"][key] = value
            with self.subTest(key=key), self.assertRaises(ValidationError):
                KinematicsSweepRequest.model_validate(payload)
        for mode, reference in (("pitch", "x"), ("roll", "y")):
            payload = self.request(mode=mode, low=-1, high=1)
            payload["design"]["kinematics"]["reference"][reference] = None
            with self.assertRaises(ValidationError):
                KinematicsSweepRequest.model_validate(payload)
        for mutation in ("missing", "unexpected", "dimension"):
            payload = deepcopy(base)
            hp = payload["design"]["axles"]["front"]["hardpoints"]
            if mutation == "missing": del hp["uca_outer"]
            elif mutation == "unexpected": hp["guessed_axis"] = [0, 0, 0]
            else: hp["wheel_center"] = [0, -600]
            with self.assertRaises(ValidationError): KinematicsSweepRequest.model_validate(payload)

    def test_api_rejects_coercion_nonfinite_and_bad_sweep_bounds(self):
        for value in (True, "1600", None, math.inf, math.nan, 10001, 499):
            payload = self.request()
            payload["design"]["wheelbase"] = value
            response = self.client.post("/api/v1/kinematics/sweep", content=json.dumps(payload), headers={"Content-Type": "application/json"})
            self.assertEqual(response.status_code, 422, (value, response.text))
            self.finite_json(response.json())
        for target, value in (("samples", True), ("samples", "5"), ("samples", 3.5), ("samples", 2), ("samples", 82),
                              ("min", "-5"), ("max", None), ("max", 101), ("min", 5), ("mode", "yaw")):
            payload = self.request()
            payload["sweep"][target] = value
            response = self.client.post("/api/v1/kinematics/sweep", json=payload)
            self.assertEqual(response.status_code, 422, (target, value, response.text))
        for samples in (3, 81):
            payload = self.request(samples=samples)
            self.assertEqual(KinematicsSweepRequest.model_validate(payload).sweep.samples, samples)
        payload = self.request(mode="steering", low=-31, high=31)
        self.assertEqual(self.client.post("/api/v1/kinematics/sweep", json=payload).status_code, 422)

    def test_api_rejects_bad_alignment_axis_hardpoint_and_wrapper(self):
        for kind, value in (("staticCamberDeg", True), ("staticToeInDeg", "0"), ("staticCamberDeg", 46),
                            ("rockerAxis", [0, 0, 0]), ("rockerAxis", [1, 0]), ("rockerAxis", [1, 0, "0"])):
            payload = self.request()
            payload["design"]["kinematics"]["axles"]["front"][kind] = value
            self.assertEqual(self.client.post("/api/v1/kinematics/sweep", json=payload).status_code, 422)
        payload = self.request()
        payload["design"]["axles"]["front"]["hardpoints"]["wheel_center"][2] = True
        self.assertEqual(self.client.post("/api/v1/kinematics/sweep", json=payload).status_code, 422)
        payload = self.request()
        payload["unexpected"] = 1
        self.assertEqual(self.client.post("/api/v1/kinematics/sweep", json=payload).status_code, 422)
        for malformed in ("{", "[]", "null", "{\"design\":{}}"):
            response = self.client.post("/api/v1/kinematics/sweep", content=malformed, headers={"Content-Type": "application/json"})
            self.assertEqual(response.status_code, 422)

    def test_arb_carriers_preserve_links_and_derivatives_through_full_solver(self):
        for carrier in ("upright", "lower-wishbone", "upper-wishbone", "rocker"):
            d = self.design()
            for name in ("front", "rear"):
                a = d["axles"][name]; p = a["hardpoints"]
                key = {"upright":"wheel_center", "lower-wishbone":"lca_outer", "upper-wishbone":"uca_outer", "rocker":"rocker_damper"}[carrier]
                pickup = np.array(p[key], dtype=float)
                a["antiRollBar"] = {"type":"u-bar", "enabled":True, "points":{
                    "bearing":[None,None,None], "bend":(pickup+[80,0,-120]).tolist(),
                    "arm_tip":(pickup+[0,0,-120]).tolist(), "link_pickup":pickup.tolist()}}
                d["kinematics"]["axles"][name].update(arbModel="straight-transverse", arbAttachment=carrier)
            for mode, center, delta in (("heave", 3., .01), ("roll", .2, .001), ("steering", 3., .01)):
                rows = self.run_sweep(d, mode, center-delta, center+delta)["rows"]
                for name in ("front", "rear"):
                    for side in ("left", "right"):
                        prefix = f"{name}.{side}."
                        for row in rows:
                            pose = row["poses"][name][side]; arb = pose["arb"]
                            self.assertTrue(arb["available"], (carrier, mode, arb))
                            self.assertGreater(arb["pathSamples"], 1)
                            self.assertNotIn("_path", pose)
                            self.assertAlmostEqual(np.linalg.norm(np.array(arb["tip"])-arb["pickup"]), 120, delta=1e-7)
                            self.assertAlmostEqual(np.linalg.norm(np.array(arb["tip"])-arb["bend"]), 80, delta=1e-7)
                            self.assertTrue(np.allclose(arb["pickup"], pose["points"][key], atol=1e-8))
                        fd = (rows[-1]["values"][prefix+"arbAngle"]-rows[0]["values"][prefix+"arbAngle"])/(2*delta)
                        self.assertAlmostEqual(fd, rows[1]["values"][prefix+"arbAngleModeGain"], delta=2e-6)
                    if mode == "heave":
                        self.assertAlmostEqual(rows[1]["values"][name+".arbTwist"], 0, delta=1e-9)

    def test_anti_inputs_control_availability_and_preserve_mirror_symmetry(self):
        d = self.design()
        self.assertIsNone(self.run_sweep(d)["rows"][1]["values"]["front.left.brakeAnti"])
        d["kinematics"]["forceReference"] = {"cgHeightMm":300, "frontBrakeFraction":.6, "frontDriveFraction":0}
        for name in ("front", "rear"):
            d["kinematics"]["axles"][name].update(brakeReaction="upright", driveReaction="chassis")
        for row in self.run_sweep(d)["rows"]:
            for name in ("front", "rear"):
                for metric in ("brakeAnti", "driveAnti"):
                    left, right = (row["values"][f"{name}.{side}.{metric}"] for side in ("left", "right"))
                    self.assertIsNotNone(left)
                    self.assertAlmostEqual(left, right, delta=1e-9)
            self.assertEqual(row["values"]["front.left.driveAnti"], 0)

    def test_api_rejects_malformed_arb_without_server_errors(self):
        for invalid in (None, [], {"enabled":"yes"}, {"enabled":1}, {"points":{"bend":[0,1,0]}}, {"points":{"bend":[0,None]}}, {"points":{"bend":[0,-100,"0"]}}, {"points":{"unexpected":[0,0,0]}}):
            payload = self.request()
            payload["design"]["axles"]["front"]["antiRollBar"] = invalid
            response = self.client.post("/api/v1/kinematics/sweep", json=payload)
            self.assertEqual(response.status_code, 422, (invalid,response.text))


if __name__ == "__main__":
    unittest.main()
