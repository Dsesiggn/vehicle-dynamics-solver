"""Independent rigid-body invariants and differential checks, not road validation."""
from copy import deepcopy
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vehicle_dynamics.models.suspension import (
    _actuator, _constraints, _model, actuator_velocity, circular_tire_constraint,
    corner_velocity, solve_corner,
)


def fixture(topology="double-wishbone", actuation="pushrod"):
    p = {"lca_front": [125, -240, -125], "lca_rear": [-125, -240, -125],
         "lca_outer": [0, -565, -135], "tie_rod_inner": [95, -220, -195],
         "tie_rod_outer": [90, -575, -195], "wheel_center": [0, -600, -230]}
    if topology == "double-wishbone":
        p.update(uca_front=[110, -258, -330], uca_rear=[-110, -258, -330], uca_outer=[0, -540, -315])
    elif topology == "macpherson":
        p.update(strut_top=[0, -396, -550], strut_bottom=[0, -545, -290])
        actuation = "direct"
    else:
        for key in ("lca_front", "lca_rear", "lca_outer"):
            del p[key]
        p.update(lower_front_inner=[170, -240, -125], lower_front_outer=[40, -565, -135],
                 lower_rear_inner=[-180, -270, -135], lower_rear_outer=[-35, -570, -140],
                 upper_front_inner=[150, -258, -330], upper_front_outer=[30, -535, -315],
                 upper_rear_inner=[-160, -282, -315], upper_rear_outer=[-35, -540, -305])
    if topology != "macpherson":
        p["actuation_outer"] = [0, -520, -295 if actuation == "pullrod" else -165]
        if actuation == "direct":
            p["damper_top"] = [0, -360, -515]
        elif actuation == "pullrod":
            p.update(rocker_pivot=[0, -252, -120], rocker_rod=[0, -252, -195],
                     rocker_damper=[0, -187, -120], damper_top=[0, -108, -370])
        else:
            p.update(rocker_pivot=[0, -252, -410], rocker_rod=[0, -252, -335],
                     rocker_damper=[0, -187, -410], damper_top=[0, -108, -530])
    return {"topology": topology, "actuation": actuation, "actuationAttachment": "upright",
            "steered": True, "rackTravel": 60, "hardpoints": p}


def independent_matrix(axis, theta):
    x, y, z = axis
    c, s, v = math.cos(theta), math.sin(theta), 1 - math.cos(theta)
    return np.array([[c+x*x*v, x*y*v-z*s, x*z*v+y*s],
                     [y*x*v+z*s, c+y*y*v, y*z*v-x*s],
                     [z*x*v-y*s, z*y*v+x*s, c+z*z*v]])


class SuspensionTests(unittest.TestCase):
    def solved(self, a, bump=0, rack=0, **kwargs):
        p = solve_corner(a, bump, rack, **kwargs)
        self.assertTrue(p["ok"], p.get("reason"))
        return p

    def close(self, a, b, tolerance=1e-7):
        np.testing.assert_allclose(a, b, atol=tolerance, rtol=0)

    def test_static_reproduction_finite_json_and_no_input_mutation(self):
        for topology in ("double-wishbone", "macpherson", "multi-link"):
            a = fixture(topology)
            before = deepcopy(a)
            for side in ("left", "right"):
                p = self.solved(a, side=side, rocker_axis=[1, 0, 0])
                reflection = [1, -1 if side == "right" else 1, 1]
                for key, value in a["hardpoints"].items():
                    self.close(p["points"][key], np.asarray(value) * reflection)
                self.close(p["rotation"], [0, 0, 0])
                self.assertAlmostEqual(p["compression"], 0, delta=1e-8)
                self.assertEqual(p["alignmentReference"], "nominal-zero-static-alignment")
                json.dumps(p, allow_nan=False)
            self.assertEqual(a, before)

    def test_rigid_link_lengths_and_full_upright_distances(self):
        for topology in ("double-wishbone", "macpherson", "multi-link"):
            a = fixture(topology)
            for side in ("left", "right"):
                model = _model(a, side)
                for bump, rack in ((-25.4, -15), (25.4, 15), (10, -20)):
                    p = self.solved(a, bump, rack, side=side, rocker_axis=[1, 0, 0])
                    self.assertLess(p["error"], 1e-7)
                    for (inner, outer), length in zip(model["pairs"], model["lengths"]):
                        self.assertAlmostEqual(math.dist(p["points"][inner], p["points"][outer]), length, delta=1e-7)
                    moving = [key for key in a["hardpoints"] if key.endswith("_outer") or key in ("wheel_center", "strut_bottom")]
                    for i, first in enumerate(moving):
                        for second in moving[i+1:]:
                            self.assertAlmostEqual(math.dist(p["points"][first], p["points"][second]),
                                                   math.dist(a["hardpoints"][first], a["hardpoints"][second]), delta=1e-7)
                    self.assertAlmostEqual(p["points"]["wheel_center"][2], a["hardpoints"]["wheel_center"][2]-bump, delta=1e-8)
                    self.assertLess(p["diagnostics"]["rotationOrthogonalityError"], 1e-12)
                    self.assertAlmostEqual(p["diagnostics"]["rotationDeterminant"], 1, delta=1e-12)

    def test_independent_inclined_finite_steering_rotation(self):
        k = np.array([3/13, 4/13, -12/13])
        for topology in ("double-wishbone", "macpherson"):
            a = fixture(topology, "direct")
            p = a["hardpoints"]
            p.update(lca_outer=[-30, -590, -110], wheel_center=[-15, -620, -220], tie_rod_outer=[85, -570, -195])
            if topology == "double-wishbone":
                p.update(uca_outer=[30, -510, -350], actuation_outer=[0, -510, -180])
            else:
                p.update(strut_top=[30, -510, -350], strut_bottom=[-10, -540, -190])
            lower = np.asarray(p["lca_outer"])
            for degrees in (-12, 12):
                theta = math.radians(degrees)
                matrix = independent_matrix(k, theta)
                position = lambda point: lower + matrix @ (np.asarray(point) - lower)
                wc, outer, inner = position(p["wheel_center"]), position(p["tie_rod_outer"]), p["tie_rod_inner"]
                length = math.dist(p["tie_rod_outer"], inner)
                rack = outer[1]-inner[1]+math.sqrt(length**2-(outer[0]-inner[0])**2-(outer[2]-inner[2])**2)
                bump = p["wheel_center"][2]-wc[2]
                for side in ("left", "right"):
                    reflect = np.array([1, -1 if side == "right" else 1, 1])
                    result = self.solved(a, float(bump), float(rack if side == "left" else -rack), side=side)
                    self.close(result["points"]["wheel_center"], wc * reflect, 1e-6)
                    self.close(result["spindle"], (matrix @ [0, -1, 0]) * reflect, 1e-8)
                    self.close(result["rotation"], k * theta * ([-1, 1, -1] if side == "right" else [1, 1, 1]), 1e-8)
                    self.assertGreater(abs(result["camber"]), 2)

    def test_analytic_constraint_jacobian_against_central_differences(self):
        for topology in ("double-wishbone", "macpherson", "multi-link"):
            a = fixture(topology)
            p = self.solved(a, 17, -9)
            model = _model(a, "left")
            center = np.asarray(p["points"]["wheel_center"])
            rotation = Rotation.from_rotvec(p["rotation"]).as_matrix()
            _, jac = _constraints(model, center, rotation, 17, -9)
            numeric = np.zeros((6, 6))
            for j in range(6):
                h = 1e-4 if j < 3 else 1e-6
                values = []
                for sign in (-1, 1):
                    dc, dr = np.zeros(3), np.zeros(3)
                    if j < 3:
                        dc[j] = sign*h
                    else:
                        dr[j-3] = sign*h
                    r = Rotation.from_rotvec(dr).as_matrix() @ rotation
                    values.append(_constraints(model, center+dc, r, 17, -9)[0])
                numeric[:, j] = (values[1]-values[0])/(2*h)
            self.close(jac, numeric, 8e-8)

    def test_constraint_tangent_converges_to_refined_pose_differences(self):
        for topology in ("double-wishbone", "macpherson", "multi-link"):
            a = fixture(topology)
            for side in ("left", "right"):
                p = self.solved(a, 14, 8, side=side)
                v = corner_velocity(a, p)
                self.assertTrue(v["ok"], v.get("reason"))
                self.assertLess(v["residual"], 1e-12)
                errors = []
                for h in (.1, .05, .025):
                    minus = self.solved(a, 14-h, 8, side=side)
                    plus = self.solved(a, 14+h, 8, side=side)
                    vc = (np.asarray(plus["points"]["wheel_center"])-minus["points"]["wheel_center"])/(2*h)
                    rm, rp = (Rotation.from_rotvec(q["rotation"]).as_matrix() for q in (minus, plus))
                    omega = Rotation.from_matrix(rp @ rm.T).as_rotvec()/(2*h)
                    errors.append(float(np.linalg.norm(vc-v["linear"])+np.linalg.norm(omega-v["angular"])))
                    self.close(v["linear"], vc, 1e-7)
                    self.close(v["angular"], omega, 1e-9)
                self.assertLess(errors[-1], errors[0] * .2 + 1e-10)

    def test_rack_tangent_global_sign_and_locked_rear(self):
        a = fixture()
        for side in ("left", "right"):
            p = self.solved(a, 5, 9, side=side)
            v = corner_velocity(a, p, bump_rate=0, rack_rate=1)
            minus = self.solved(a, 5, 8.999, side=side)
            plus = self.solved(a, 5, 9.001, side=side)
            fd = (np.asarray(plus["points"]["wheel_center"])-minus["points"]["wheel_center"])/.002
            self.close(v["linear"], fd, 1e-7)
        a["steered"] = False
        p = self.solved(a, 0, 20)
        self.assertEqual(p["rackMm"], 0)
        self.close(corner_velocity(a, p, bump_rate=0, rack_rate=1)["linear"], [0, 0, 0])

    def test_unknown_rocker_axis_keeps_pose_and_marks_actuator_unavailable(self):
        a = fixture()
        missing = self.solved(a, 12, 5)
        complete = self.solved(a, 12, 5, rocker_axis=[1, 0, 0])
        self.assertIsNone(missing["compression"])
        self.assertFalse(missing["actuator"]["ok"])
        self.assertIn("axis", missing["actuator"]["reason"])
        self.close(missing["points"]["wheel_center"], complete["points"]["wheel_center"])
        self.assertFalse(actuator_velocity(a, missing, corner_velocity(a, missing))["ok"])

    def test_static_alignment_is_explicit_side_consistent_and_not_pose_constraint(self):
        a = fixture()
        for side in ("left", "right"):
            p = self.solved(a, side=side, static_alignment={"camberDeg": -3.5, "toeInDeg": .7})
            self.assertAlmostEqual(p["camber"], -3.5, places=12)
            self.assertAlmostEqual(p["toe"], .7, places=12)
            self.assertEqual(p["alignmentReference"], "entered-static-alignment")
            changed = self.solved(a, 15, 10, side=side, static_alignment={"camberDeg": -3.5, "toeInDeg": .7})
            nominal = self.solved(a, 15, 10, side=side)
            self.close(changed["rotation"], nominal["rotation"])
            self.close(changed["points"]["wheel_center"], nominal["points"]["wheel_center"])
        for alignment in ({"camberDeg": None, "toeInDeg": 0}, {"camberDeg": 90, "toeInDeg": 0}, {"camberDeg": True, "toeInDeg": 0}):
            self.assertFalse(solve_corner(a, static_alignment=alignment)["ok"])

    def test_arbitrary_rocker_axis_matches_independently_prescribed_rotation(self):
        a = fixture()
        p = a["hardpoints"]
        p.update(rocker_pivot=[10, -200, -300], rocker_rod=[10, -125, -300],
                 rocker_damper=[-38, -200, -264], damper_top=[100, -80, -500],
                 actuation_outer=[10, -325, -600])
        k = [.6, 0, .8]
        matrix = independent_matrix(k, .3)
        o = np.asarray(p["rocker_pivot"])
        expected_b = o + matrix @ (np.asarray(p["rocker_rod"])-o)
        expected_d = o + matrix @ (np.asarray(p["rocker_damper"])-o)
        for side in ("left", "right"):
            model = _model(a, side)
            moved = {key: value.copy() for key, value in model["points"].items()}
            reflect = model["reflection"]
            moved["actuation_outer"] = (expected_b+[0, -200, -300])*reflect
            result = _actuator(a, model, moved, k)
            self.assertTrue(result["ok"], result.get("reason"))
            self.close(moved["rocker_rod"], expected_b*reflect)
            self.close(moved["rocker_damper"], expected_d*reflect)
            self.assertAlmostEqual(result["compression"], math.dist(p["damper_top"], p["rocker_damper"])-math.dist(p["damper_top"], expected_d), delta=1e-7)

    def test_actuator_derivatives_against_central_differences_and_halving(self):
        for topology in ("double-wishbone", "macpherson", "multi-link"):
            methods = ("pushrod", "pullrod", "direct") if topology != "macpherson" else ("direct",)
            for method in methods:
                for attachment in (("upright", "lower-wishbone", "upper-wishbone") if topology == "double-wishbone" else ("upright",)):
                    a = fixture(topology, method)
                    a["actuationAttachment"] = attachment
                    for side in ("left", "right"):
                        axis = [.8, .2, .4]
                        pose = self.solved(a, 8, 4, side=side, rocker_axis=axis)
                        tangent = corner_velocity(a, pose)
                        derivative = actuator_velocity(a, pose, tangent, rocker_axis=axis)
                        self.assertTrue(derivative["ok"], derivative.get("reason"))
                        differences = []
                        for h in (.02, .01):
                            low = self.solved(a, 8-h, 4, side=side, rocker_axis=axis)
                            high = self.solved(a, 8+h, 4, side=side, rocker_axis=axis)
                            self.assertIsNotNone(low["compression"])
                            self.assertIsNotNone(high["compression"])
                            rate = (high["compression"]-low["compression"])/(2*h)
                            differences.append(abs(rate-derivative["compressionRate"]))
                            self.assertAlmostEqual(rate, derivative["compressionRate"], delta=2e-7)
                        self.assertLess(differences[-1], differences[0]*.4+2e-9)

    def test_arm_carried_pickup_is_rigid_to_real_inboard_axis(self):
        for attachment, prefix in (("upper-wishbone", "uca"), ("lower-wishbone", "lca")):
            a = fixture(actuation="direct")
            a["actuationAttachment"] = attachment
            # Offset bracket, deliberately outside the arm's defining plane.
            a["hardpoints"]["actuation_outer"][0] = 45
            p = self.solved(a, 20, 14)
            for key in (prefix+"_front", prefix+"_rear", prefix+"_outer"):
                self.assertAlmostEqual(math.dist(p["points"]["actuation_outer"], p["points"][key]),
                                       math.dist(a["hardpoints"]["actuation_outer"], a["hardpoints"][key]), delta=1e-7)

    def test_reflection_and_translation_invariance(self):
        a = fixture()
        left = self.solved(a, 17, -11, rocker_axis=[.8, .2, .4])
        right = self.solved(a, 17, 11, side="right", rocker_axis=[.8, .2, .4])
        for key, point in left["points"].items():
            self.close(right["points"][key], np.asarray(point)*[1, -1, 1], 1e-6)
        self.close(right["rotation"], np.asarray(left["rotation"])*[-1, 1, -1], 1e-8)
        shifted = deepcopy(a)
        shift = [500, 10, -50]
        shifted["hardpoints"] = {key: (np.asarray(value)+shift).tolist() for key, value in a["hardpoints"].items()}
        result = self.solved(shifted, 17, -11, rocker_axis=[.8, .2, .4])
        for key, point in left["points"].items():
            self.close(result["points"][key], np.asarray(point)+shift, 1e-6)
        self.close(result["rotation"], left["rotation"], 1e-8)

    def test_zero_residual_singular_constraint_set_is_rejected(self):
        a = fixture()
        a["hardpoints"]["uca_front"] = a["hardpoints"]["uca_rear"][:]
        result = solve_corner(a)
        self.assertFalse(result["ok"])
        self.assertIn("Jacobian", result["reason"])

    def test_degenerate_or_invalid_inputs_are_explicit_failures(self):
        for bad in (None, {}, [], True):
            self.assertFalse(solve_corner(bad)["ok"])
        a = fixture()
        for bad in (None, True, "5", math.nan, math.inf, 101):
            self.assertFalse(solve_corner(a, bad)["ok"])
        self.assertFalse(solve_corner(a, rack_mm=31)["ok"])
        self.assertFalse(solve_corner(a, side="center")["ok"])
        for key in ("wheel_center", "lca_front"):
            bad = deepcopy(a)
            bad["hardpoints"][key][0] = math.nan
            self.assertFalse(solve_corner(bad)["ok"])
        for axis in ([0, 0, 0], [None, 0, 1], [True, 0, 0]):
            self.assertFalse(self.solved(a, rocker_axis=axis)["actuator"]["ok"])

    def test_strut_parallel_to_x_has_valid_basis_not_zero_constraints(self):
        a = fixture("macpherson")
        a["hardpoints"]["strut_top"] = [200, -545, -290]
        model = _model(a, "left")
        self.assertEqual(len(model["normals"]), 2)
        self.close(np.asarray(model["normals"]) @ np.asarray(model["normals"]).T, np.eye(2), 1e-12)
        _, jac = _constraints(model, model["center"], np.eye(3), 0, 0)
        self.assertGreater(np.linalg.norm(jac[-3]), .99)
        self.assertGreater(np.linalg.norm(jac[-2]), .99)

    def test_circular_contact_helper_exact_radius_and_jacobian(self):
        gamma = math.radians(-7)
        n0 = np.array([0, -math.cos(gamma), math.sin(gamma)])
        c = np.array([10, -600, -230*math.cos(gamma)])
        constraint = circular_tire_constraint([0, 0, 1], 0, 230)
        value, jac = constraint(c, np.eye(3), n0)
        self.assertAlmostEqual(value, 0, delta=1e-12)
        for j in range(6):
            h = 1e-5 if j < 3 else 1e-6
            vals = []
            for sign in (-1, 1):
                dc, dr = np.zeros(3), np.zeros(3)
                if j < 3:
                    dc[j] = sign*h
                else:
                    dr[j-3] = sign*h
                vals.append(constraint(c+dc, Rotation.from_rotvec(dr).as_matrix(), n0)[0])
            self.assertAlmostEqual(jac[j], (vals[1]-vals[0])/(2*h), delta=5e-8)

    def test_general_last_constraint_checks_neutral_and_solves_contact(self):
        a = fixture()
        factory = lambda fraction: circular_tire_constraint([0, 0, 1], -15*fraction, 230)
        p = self.solved(a, constraint_factory=factory, continuation_steps=15)
        value, _ = factory(1)(np.asarray(p["points"]["wheel_center"]), Rotation.from_rotvec(p["rotation"]).as_matrix(), np.array([0, -1, 0]))
        self.assertAlmostEqual(value, 0, delta=1e-7)
        wrong = lambda fraction: circular_tire_constraint([0, 0, 1], 0, 240)
        result = solve_corner(a, constraint_factory=wrong)
        self.assertFalse(result["ok"])
        self.assertIn("neutral", result["reason"])

    @unittest.skipUnless(shutil.which("node"), "Node is required for existing-browser parity evidence")
    def test_existing_browser_pose_parity_explicit_legacy_rocker_axis(self):
        root = Path(__file__).resolve().parents[2]
        source = """import {axle} from './suspension-studio/src/model.js';
import {solveAxle} from './suspension-studio/src/solver.js';
const rows=[];
for(const topology of ['double-wishbone','macpherson','multi-link']) {
 const a=axle(topology);for(const [b,r] of [[-20,-12],[0,15],[20,12]]) rows.push({a,b,r,p:solveAxle(a,b,r)});
}console.log(JSON.stringify(rows));"""
        result = subprocess.run([shutil.which("node"), "--input-type=module", "-e", source], cwd=root, text=True, capture_output=True, check=True)
        for row in json.loads(result.stdout):
            for side in ("left", "right"):
                actual = self.solved(row["a"], row["b"], row["r"], side=side, rocker_axis=[1, 0, 0])
                expected = row["p"][side]
                self.assertTrue(expected["ok"])
                for key in expected["points"]:
                    self.close(actual["points"][key], expected["points"][key], 1e-4)
                self.close(actual["rotation"], expected["rotation"], 1e-6)
                self.assertAlmostEqual(actual["compression"], expected["compression"], delta=1e-4)


if __name__ == "__main__":
    unittest.main()
