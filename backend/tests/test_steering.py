"""Independent geometry/invariants and JS port parity; not physical validation.

Evidence: STEERING.md derives fixed-axis circle/sphere closure. Seward Table6.2
(printed163/PDF174) checks the separate planar ideal, also supported by Gillespie
Eq8-1/8-2 (printed277/PDF293) with the explicit project rear-center radius.
Milliken Fig19.3 (printed714/PDF377) defines the ball-height plan arm. Ambiguous
source ratio/radius conventions are not imported into these geometric tests.
"""

from copy import deepcopy
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vehicle_dynamics.models.steering import (
    ideal_outside_angle, solve_steering, sweep_steering, validate_steering_design,
)
from vehicle_dynamics.models.steering_geometry import steering_construction
from steering_fixtures import (
    construction_design, design, path_design, reflected_design, scaled_design, tilted_design,
)


def inclined_linkage_fixture():
    """Known basis k=(0,.6,.8), e1=(1,0,0), e2=(0,.8,-.6).

    C=(0,-550,-250), initial arm=-100e1+30e2. The arm and tie rod
    both have nonzero Z extent; rack joints also have different heights.
    The specified travel direction is still +Y, not along their joining line.
    """
    source = design()
    source["left"].update(
        axisUpper=[0, -640, -370], axisLower=[0, -460, -130],
        tieOuter=[-100, -526, -268], tieInner=[-110, -250, -180],
        wheelCenterZ=-247,
    )
    return source


def prescribed_spatial_outer(phi):
    """Rotate two coordinates in the known arm plane, not the inverse solver."""
    u = -100 * math.cos(phi) - 30 * math.sin(phi)
    v = -100 * math.sin(phi) + 30 * math.cos(phi)
    return [u, -550 + 0.8 * v, -250 - 0.6 * v]


def prescribed_rack_displacement(source, phi):
    outer = prescribed_spatial_outer(phi)
    inner = source["left"]["tieInner"]
    length = math.dist(source["left"]["tieOuter"], inner)
    # (Oy-Iy-q)^2 = ell^2-(Ox-Ix)^2-(Oz-Iz)^2. The plus root
    # yields q=0 at phi=0 because this left-side reference has Oy<Iy.
    return outer[1] - inner[1] + math.sqrt(
        length**2 - (outer[0] - inner[0])**2 - (outer[2] - inner[2])**2
    )


class SteeringTests(unittest.TestCase):
    def solved(self, source, angle):
        result = solve_steering(source, angle)
        self.assertTrue(result["ok"], result.get("reason"))
        return result

    def vector_close(self, actual, expected, tolerance=1e-9):
        self.assertEqual(len(actual), len(expected))
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, delta=tolerance)

    def assert_tree_close(self, actual, expected, path="root"):
        if isinstance(expected, dict):
            self.assertEqual(set(actual), set(expected), path)
            for key in expected:
                self.assert_tree_close(actual[key], expected[key], f"{path}.{key}")
        elif isinstance(expected, list):
            self.assertEqual(len(actual), len(expected), path)
            for index, (a, b) in enumerate(zip(actual, expected)):
                self.assert_tree_close(a, b, f"{path}[{index}]")
        elif isinstance(expected, (int, float)) and not isinstance(expected, bool):
            self.assertTrue(math.isfinite(actual), path)
            tolerance = 2e-8 if any(name in path for name in ("Deg", "Percent", "cotResidual", "spindle")) else 2e-7
            self.assertAlmostEqual(actual, expected, delta=tolerance + 2e-12 * abs(expected), msg=path)
        else:
            self.assertEqual(actual, expected, path)

    def test_reference_input_preserved_and_finite_json(self):
        source = tilted_design()
        source["extension"] = {"note": "preserved by callers"}
        before = deepcopy(source)
        self.assertEqual(validate_steering_design(source), [])
        result = self.solved(source, 145)
        construction = steering_construction(source, result)
        self.assertEqual(source, before)
        json.dumps({"result": result, "construction": construction}, allow_nan=False)
        construction["left"]["outer"][0] = 999
        self.assertNotEqual(result["left"]["outer"][0], 999)

    def test_metadata_name_and_container_validation(self):
        for source in (None, [], "geometry", 1, True):
            self.assertTrue(validate_steering_design(source))
            self.assertFalse(solve_steering(source, 0)["ok"])
            self.assertEqual(sweep_steering(source), [])
        for key, value in (("version", True), ("version", 2), ("kind", "suspension"),
                           ("coordinateSystem", "Z_UP"), ("hardpointFrame", "GLOBAL"),
                           ("lengthUnit", "m"), ("name", " "), ("name", "x" * 201),
                           ("name", "😀" * 101), ("left", None)):
            with self.subTest(key=key, value=value):
                source = design()
                source[key] = value
                self.assertTrue(validate_steering_design(source))

    def test_numbers_are_not_coerced_and_bounds_are_enforced(self):
        bad = (None, True, False, "1200", [], {}, float("nan"), float("inf"), -float("inf"), 10**1000)
        for value in bad:
            for target in ("wheelbase", "rackMmPerRevolution", "coordinate", "staticToeDeg"):
                with self.subTest(target=target, value=repr(value)[:25]):
                    source = design()
                    if target == "coordinate":
                        source["left"]["axisUpper"][0] = value
                    elif target == "staticToeDeg":
                        source["left"][target] = value
                    else:
                        source[target] = value
                    self.assertTrue(validate_steering_design(source))
                    self.assertFalse(solve_steering(source, 0)["ok"])
        for key, value in (("wheelbase", 0), ("frontTrack", -1), ("rearTrack", 1e7 + 1),
                           ("rackTravel", 0), ("rackMmPerRevolution", 0),
                           ("steeringLimitDeg", 0), ("steeringLimitDeg", 6), ("steeringLimitDeg", 3605)):
            source = design()
            source[key] = value
            self.assertTrue(validate_steering_design(source))
        for key in ("staticToeDeg", "staticCamberDeg"):
            for value in (-90, 90):
                source = design()
                source["left"][key] = value
                self.assertTrue(validate_steering_design(source))
        for value in (None, {}, [], [0, 0], [0, 0, 0, 0], [0, None, 0]):
            source = design()
            source["left"]["axisUpper"] = value
            self.assertTrue(validate_steering_design(source))

    def test_incomplete_mode_only_accepts_explicit_nulls(self):
        source = design()
        source["wheelbase"] = None
        source["left"]["axisUpper"][0] = None
        self.assertEqual(validate_steering_design(source, allow_incomplete=True), [])
        self.assertTrue(validate_steering_design(source))
        del source["wheelbase"]
        self.assertTrue(validate_steering_design(source, allow_incomplete=True))

    def test_independent_prescribed_planar_rotation(self):
        source = design()
        # Rotate the arm geometrically first. Intersect the horizontal rack
        # travel line with the fixed270mm rod to derive q without inverse solve.
        for degrees in (-20, -10, 10, 20):
            angle = math.radians(degrees)
            x = -100 * math.cos(angle) - 30 * math.sin(angle)
            y = -550 - 100 * math.sin(angle) + 30 * math.cos(angle)
            q = y + 250 + math.sqrt(270**2 - (x + 100)**2)
            result = self.solved(source, q * 360 / source["rackMmPerRevolution"])
            self.assertAlmostEqual(result["left"]["rotationDeg"], degrees, delta=1e-10)
            self.assertAlmostEqual(result["left"]["headingDeg"], degrees, delta=1e-10)
            self.vector_close(result["left"]["outer"], [x, y, -250])
            self.assertAlmostEqual(result["left"]["tieLength"], 270, delta=1e-12)

    def test_prescribed_spatial_rotation_with_nonhorizontal_arm_and_tie(self):
        source = inclined_linkage_fixture()
        for degrees in (-20, -12, 12, 20):
            phi = math.radians(degrees)
            expected_outer = prescribed_spatial_outer(phi)
            rack = prescribed_rack_displacement(source, phi)
            result = self.solved(source, rack * 360 / source["rackMmPerRevolution"])
            solved = result["left"]
            self.assertAlmostEqual(solved["rotationDeg"], degrees, delta=1e-10)
            self.vector_close(solved["outer"], expected_outer)
            # Initial outward spindle (0,-1,0) = -.8e2-.6k. Rotate only
            # its e1/e2 components; the axis component stays unchanged.
            expected_spindle = [0.8 * math.sin(phi), -0.64 * math.cos(phi) - 0.36,
                                0.48 * (math.cos(phi) - 1)]
            expected_heading = math.degrees(math.atan2(expected_spindle[0], -expected_spindle[1]))
            expected_camber = math.degrees(math.atan2(expected_spindle[2], math.hypot(*expected_spindle[:2])))
            self.vector_close(solved["spindle"], expected_spindle, 1e-12)
            self.assertAlmostEqual(solved["headingDeg"], expected_heading, delta=1e-10)
            self.assertAlmostEqual(solved["camberDeg"], expected_camber, delta=1e-10)
            # Its reference offset from C=(0,-550,-250) is (0,-50,3),
            # so its basis components are -41.8e2-27.6k. Left and right
            # wheel-center heights deliberately differ in this assembly.
            expected_wheel_center = [41.8 * math.sin(phi), -566.56 - 33.44 * math.cos(phi),
                                     -272.08 + 25.08 * math.cos(phi)]
            self.vector_close(solved["wheelCenter"], expected_wheel_center)
            self.assertAlmostEqual(math.dist(solved["outer"], solved["inner"]), solved["tieLength"], delta=1e-9)
            for side in ("left", "right"):
                self.vector_close(result[side]["inner"], [source[side]["tieInner"][0],
                                  source[side]["tieInner"][1] + rack, source[side]["tieInner"][2]])
            construction = steering_construction(source, result)["left"]
            self.vector_close(construction["axisFoot"], [0, -550, -250])
            # Milliken's plan point is at current outer-ball height, whereas
            # the perpendicular radius root C remains fixed on this axis.
            expected_plan_root = [0, -550 + 0.75 * (expected_outer[2] + 250), expected_outer[2]]
            self.vector_close(construction["planAxisPoint"], expected_plan_root)
            self.assertGreater(abs(construction["spatialAngleDeg"] - construction["planAngleDeg"]), 0.1)

    def test_changing_only_inner_height_changes_inclined_axis_steering(self):
        source = inclined_linkage_fixture()
        rack = prescribed_rack_displacement(source, math.radians(12))
        steering_wheel = rack * 360 / source["rackMmPerRevolution"]
        original = self.solved(source, steering_wheel)
        for inner_z in (-268, -340):
            changed = deepcopy(source)
            changed["left"]["tieInner"][2] = inner_z
            actual = self.solved(changed, steering_wheel)
            # A separate direct-distance bisection in a known local bracket
            # independently finds the rotation at this fixed rack displacement.
            # Each edited reference assembly has its own measured/adjusted rod
            # length; moving a built rack with its old fixed rod is not modeled.
            reference_length = math.dist(changed["left"]["tieOuter"], changed["left"]["tieInner"])
            moved_inner = [-110, -250 + rack, inner_z]

            def residual(phi):
                return math.dist(prescribed_spatial_outer(phi), moved_inner) - reference_length

            lower, upper = 0.0, math.radians(30)
            lower_residual = residual(lower)
            self.assertLess(lower_residual * residual(upper), 0)
            for _ in range(70):
                middle = (lower + upper) / 2
                if lower_residual * residual(middle) > 0:
                    lower, lower_residual = middle, residual(middle)
                else:
                    upper = middle
            expected_phi = (lower + upper) / 2
            expected_heading = math.degrees(math.atan2(0.8 * math.sin(expected_phi),
                                                       0.64 * math.cos(expected_phi) + 0.36))
            self.assertAlmostEqual(actual["left"]["rotationDeg"], math.degrees(expected_phi), delta=1e-10)
            self.assertAlmostEqual(actual["left"]["headingDeg"], expected_heading, delta=1e-10)
            self.assertGreater(abs(actual["left"]["headingDeg"] - original["left"]["headingDeg"]), 2)
            self.assertEqual(actual["right"], original["right"])

    def test_vertical_axis_height_offset_cancels_in_fixed_ride_closure(self):
        # With a vertical axis, outer Z never moves. The constant squared
        # tie-height difference appears on both sides of the length constraint,
        # so height alone cannot change the angle if the reference rod is resized.
        source = design()
        for steering_wheel in (-120, 120):
            original = self.solved(source, steering_wheel)
            for inner_z in (-350, -150):
                changed = deepcopy(source)
                changed["left"]["tieInner"][2] = inner_z
                actual = self.solved(changed, steering_wheel)
                self.assertAlmostEqual(actual["left"]["headingDeg"], original["left"]["headingDeg"], delta=1e-10)
                self.vector_close(actual["left"]["outer"], original["left"]["outer"])
                self.assertGreater(actual["left"]["tieLength"], original["left"]["tieLength"])

    def test_static_toe_and_camber_signs(self):
        source = tilted_design()
        result = self.solved(source, 0)
        for side, sign in (("left", 1), ("right", -1)):
            self.assertAlmostEqual(result[side]["headingDeg"], sign * source[side]["staticToeDeg"], delta=1e-12)
            self.assertAlmostEqual(result[side]["camberDeg"], source[side]["staticCamberDeg"], delta=1e-12)
            self.assertEqual(result[side]["deltaToeInDeg"], 0)
        self.assertAlmostEqual(result["metrics"]["totalToeInDeg"], 1.1)
        self.assertEqual(result["metrics"]["turn"], "mixed")
        self.assertIsNone(result["metrics"]["ackermannPercent"])

    def test_spatial_rigidity_and_closure_invariants(self):
        source = tilted_design()
        for input_angle in (-180, -75, 0, 75, 180):
            result = self.solved(source, input_angle)
            for name, sign in (("left", -1), ("right", 1)):
                side, solved = source[name], result[name]
                upper, lower = side["axisUpper"], side["axisLower"]
                wc = [0, sign * source["frontTrack"] / 2, side["wheelCenterZ"]]
                for fixed in (upper, lower):
                    self.assertAlmostEqual(math.dist(side["tieOuter"], fixed), math.dist(solved["outer"], fixed), delta=1e-9)
                    self.assertAlmostEqual(math.dist(wc, fixed), math.dist(solved["wheelCenter"], fixed), delta=1e-9)
                self.assertAlmostEqual(math.dist(side["tieOuter"], wc), math.dist(solved["outer"], solved["wheelCenter"]), delta=1e-9)
                self.assertAlmostEqual(math.dist(solved["outer"], solved["inner"]), math.dist(side["tieOuter"], side["tieInner"]), delta=1e-9)
                self.assertAlmostEqual(math.hypot(*solved["spindle"]), 1, delta=1e-12)
            if input_angle:
                self.assertGreater(abs(result["left"]["camberDeg"] - source["left"]["staticCamberDeg"]), 0.01)

    def test_axis_direction_and_mirror_are_geometrically_invariant(self):
        source = tilted_design()
        reversed_axes = deepcopy(source)
        for side in ("left", "right"):
            reversed_axes[side]["axisUpper"], reversed_axes[side]["axisLower"] = reversed_axes[side]["axisLower"], reversed_axes[side]["axisUpper"]
        mirrored = reflected_design(source)
        for angle in (-120, 45, 120):
            original = self.solved(source, angle)
            reverse = self.solved(reversed_axes, angle)
            reflection = self.solved(mirrored, -angle)
            for side, opposite in (("left", "right"), ("right", "left")):
                self.assertAlmostEqual(original[side]["rotationDeg"], -reverse[side]["rotationDeg"], delta=1e-10)
                for key in ("outer", "wheelCenter", "spindle"):
                    self.vector_close(original[side][key], reverse[side][key])
                self.assertAlmostEqual(original[side]["headingDeg"], -reflection[opposite]["headingDeg"], delta=1e-10)
                self.assertAlmostEqual(original[side]["camberDeg"], reflection[opposite]["camberDeg"], delta=1e-10)

    def test_uniform_scaling_preserves_angles_and_scales_positions(self):
        source = tilted_design()
        expected = self.solved(source, 110)
        for factor in (0.001, 1000):
            actual = self.solved(scaled_design(source, factor), 110)
            for side in ("left", "right"):
                for key in ("headingDeg", "rotationDeg", "camberDeg"):
                    self.assertAlmostEqual(actual[side][key], expected[side][key], delta=1e-9)
                self.vector_close(actual[side]["outer"], [x * factor for x in expected[side]["outer"]], 2e-7)

    def test_rear_track_does_not_change_front_solve(self):
        source = design()
        before = self.solved(source, 100)
        source["rearTrack"] = 2200
        self.assertEqual(self.solved(source, 100), before)

    def test_unreachable_path_cannot_jump_to_feasible_endpoint(self):
        for inner_x, endpoint in ((0.5, 4), (1, 4), (0.9999999, 3.753)):
            source = path_design(inner_x)
            self.solved(source, 0)
            radius = math.hypot(inner_x, 2 - endpoint)
            self.assertLess(abs(2 - inner_x), radius)
            self.assertLess(radius, 2 + inner_x)
            result = solve_steering(source, endpoint)
            self.assertFalse(result["ok"])
            self.assertIn("path", result["reason"])
            self.assertIn("toggle", result["reason"])

    def test_degenerate_geometry_and_requested_angle_fail_cleanly(self):
        for field, value in (("axisLower", [0, -550, -450]), ("tieOuter", [0, -550, -150]),
                             ("tieInner", [-100, -520, -250]), ("tieInner", [0, -550, -250])):
            source = design()
            source["left"][field] = value
            self.assertEqual(validate_steering_design(source), [])
            result = solve_steering(source, 0)
            self.assertFalse(result["ok"])
            self.assertIn("left:", result["reason"])
        for angle in (None, True, "0", [], {}, float("nan"), float("inf"), 10**1000, 3601):
            result = solve_steering(design(), angle)
            self.assertFalse(result["ok"])
            json.dumps(result, allow_nan=False)

    def test_travel_limit_and_sweep_extent_are_separate(self):
        source = design()
        source["steeringLimitDeg"] = 20
        source["rackTravel"] = 60
        self.solved(source, 180)
        self.assertFalse(solve_steering(source, 180.001)["ok"])
        rows = sweep_steering(source)
        self.assertEqual([r["steeringWheelDeg"] for r in rows], list(range(-20, 21, 5)))
        source["rackTravel"] = 2
        rows = sweep_steering(source)
        self.assertEqual(len(rows), 9)
        self.assertFalse(rows[0]["ok"])
        self.assertTrue(rows[4]["ok"])
        self.assertFalse(rows[-1]["ok"])

    def test_inclusive_numerical_boundaries_and_maximum_sweep(self):
        source = design()
        source["wheelbase"] = source["rearTrack"] = 1e7
        source["steeringLimitDeg"] = 3600
        source["rackMmPerRevolution"] = -0.01
        self.assertEqual(validate_steering_design(source), [])
        for angle in (-3600, 3600):
            self.solved(source, angle)
        self.assertFalse(solve_steering(source, math.nextafter(3600, math.inf))["ok"])
        rows = sweep_steering(source)
        self.assertEqual(len(rows), 1441)
        self.assertEqual(rows[0]["steeringWheelDeg"], -3600)
        self.assertEqual(rows[-1]["steeringWheelDeg"], 3600)

    def test_nonfinite_computed_metric_fails_whole_pose(self):
        # Positive subnormal dimensions pass the legacy input contract, but
        # the dimension ratios can overflow. The API must disclose failure
        # rather than serialize an apparently successful ambiguous null metric.
        for key, metric in (("frontTrack", "ackermannPercent"), ("wheelbase", "cotResidual")):
            source = design()
            source[key] = math.ulp(0.0)
            self.assertEqual(validate_steering_design(source), [])
            result = solve_steering(source, 180)
            self.assertFalse(result["ok"])
            self.assertIn(metric, result["reason"])
            self.assertIsNone(result["metrics"])
            self.assertEqual(steering_construction(source, result)["pose"], "static")
            json.dumps(result, allow_nan=False)

    def test_source_table_and_defined_percentage_limits(self):
        for outside, printed_inside in ((5, 5.3), (10, 11.5), (15, 18.5), (20, 26.6), (25, 35.6)):
            radius = 1600 / math.tan(math.radians(outside)) - 600
            inside = math.degrees(math.atan2(1600, radius - 600))
            self.assertAlmostEqual(inside, printed_inside, delta=0.05)
            self.assertAlmostEqual(ideal_outside_angle(inside, 1200, 1600), outside, delta=1e-12)
        source = design()
        inside = 20
        outside = math.degrees(math.atan2(1600, 1600 / math.tan(math.radians(inside)) + 1200))
        source["left"]["staticToeDeg"], source["right"]["staticToeDeg"] = outside, -inside
        metrics = self.solved(source, 0)["metrics"]
        self.assertAlmostEqual(metrics["ackermannPercent"], 100, delta=1e-12)
        self.assertAlmostEqual(metrics["cotResidual"], 0, delta=1e-14)
        for angle in (0, 0.001):
            self.assertIsNone(self.solved(design(), angle)["metrics"]["ackermannPercent"])

    def test_construction_known_inclined_angles_and_plan_root(self):
        source = construction_design()
        planar = steering_construction(source)
        self.assertAlmostEqual(planar["left"]["planAngleDeg"], 135)
        self.vector_close(planar["intersection"]["point"], [-2, 0, 0])
        self.assertEqual(planar["intersection"]["rearAxleOffsetMm"], 8)
        source["left"] = {"axisUpper": [0, 0, 0], "axisLower": [0, 1, 1],
                          "tieOuter": [1, 1, 0], "tieInner": [1, 3, 2]}
        inclined = steering_construction(source)["left"]
        self.vector_close(inclined["axisFoot"], [0, 0.5, 0.5])
        self.vector_close(inclined["planAxisPoint"], [0, 0, 0])
        self.assertAlmostEqual(inclined["spatialAngleDeg"], 90)
        self.assertAlmostEqual(inclined["planAngleDeg"], 135)

    def test_construction_degeneracies_and_failed_pose(self):
        source = construction_design()
        source["left"]["tieOuter"] = [-1, -2, 0]
        source["right"]["tieOuter"] = [-1, 2, 0]
        self.assertEqual(steering_construction(source)["intersection"]["status"], "parallel")
        source["right"] = {"axisUpper": [4, -2, -10], "axisLower": [4, -2, 10],
                           "tieOuter": [3, -2, 0], "tieInner": [3, 0, 0]}
        self.assertEqual(steering_construction(source)["intersection"]["status"], "coincident")
        source["left"] = {"axisUpper": [-1, 0, 0], "axisLower": [1, 0, 1e-12],
                          "tieOuter": [0, 0, 1], "tieInner": [0, 1, 1]}
        horizontal = steering_construction(source)
        self.assertIsNone(horizontal["left"]["planAxisPoint"])
        self.assertAlmostEqual(horizontal["left"]["spatialAngleDeg"], 90)
        self.assertEqual(horizontal["intersection"]["status"], "undefined")
        regular = design()
        self.assertEqual(steering_construction(regular, {"ok": False}), steering_construction(regular))
        self.assertEqual(steering_construction(regular, {"ok": True})["intersection"]["status"], "undefined")

    def test_construction_roundoff_does_not_invent_an_on_axis_arm(self):
        source = construction_design()
        source["left"] = {
            "axisUpper": [4.426767549011856, 7.615198688581586, 0.9577909926883876],
            "axisLower": [13.839551915880293, 14.874140101019293, 2.773008595686406],
            "tieOuter": [11.431112197229188, 13.016802024242264, 2.3085506836956355],
            "tieInner": [5, 6, 7],
        }
        for inner in ([5, 6, 7], None):
            source["left"]["tieInner"] = inner
            result = steering_construction(source)
            self.assertIsNone(result["left"]["planAxisPoint"])
            self.assertIsNone(result["left"]["spatialAngleDeg"])
            self.assertEqual(result["intersection"]["status"], "undefined")

    def test_construction_affine_geometry_and_endpoint_span(self):
        source = construction_design()
        original = steering_construction(source)
        translation = [30000, -40000, 50000]
        for factor in (0.001, 1000):
            transformed = deepcopy(source)
            transformed["wheelbase"] *= factor
            for side in ("left", "right"):
                for key in ("axisUpper", "axisLower", "tieOuter", "tieInner"):
                    transformed[side][key] = [factor * value + translation[i] for i, value in enumerate(source[side][key])]
            result = steering_construction(transformed)
            self.assertAlmostEqual(result["left"]["planAngleDeg"], original["left"]["planAngleDeg"], delta=1e-6)
            self.vector_close(result["intersection"]["point"], [-2 * factor + translation[0], translation[1], 0], 1e-6)
        source["left"]["tieOuter"] = [1e-8, -2, 0]
        source["left"]["tieInner"] = [1e-8, -1, 0]
        short_axis = steering_construction(source)["left"]
        source["left"]["axisUpper"][2] *= 1e6
        source["left"]["axisLower"][2] *= 1e6
        long_axis = steering_construction(source)["left"]
        self.assertIsNotNone(short_axis["planAngleDeg"])
        self.assertEqual(short_axis["planAngleDeg"], long_axis["planAngleDeg"])

    def test_javascript_parity_for_solve_sweep_and_construction(self):
        node = os.environ.get("NODE") or shutil.which("node")
        if not node:
            self.skipTest("Set NODE to run JavaScript port parity.")
        source_dir = Path(__file__).resolve().parents[2] / "suspension-studio/src"
        designs = [design(), tilted_design(), reflected_design(tilted_design()),
                   scaled_design(tilted_design(), 0.001), scaled_design(tilted_design(), 1000)]
        reversed_axes = tilted_design()
        for side in ("left", "right"):
            reversed_axes[side]["axisUpper"], reversed_axes[side]["axisLower"] = reversed_axes[side]["axisLower"], reversed_axes[side]["axisUpper"]
        designs.append(reversed_axes)
        cases = [[source, angle] for source in designs for angle in range(-180, 181, 5)]
        cases += [[path_design(x), angle] for x in (0.5, 1, 0.9999999) for angle in (0, 0.5, 1, 2, 3.753, 4)]
        stop = design()
        stop["rackTravel"] = 2
        cases += [[stop, angle] for angle in (-180, -6, -5, 0, 5, 6, 180)]
        construction_cases = [construction_design(), None, {}, {"left": {"axisUpper": [True, 0, 0]}}]
        script = """
import { readFileSync } from 'node:fs';
const s = await import(process.argv[1]);
const g = await import(process.argv[2]);
const input = JSON.parse(readFileSync(0, 'utf8'));
const rows = input.cases.map(([d,a]) => { const result=s.solveSteering(d,a); return {result,construction:g.steeringConstruction(d,result)}; });
process.stdout.write(JSON.stringify({ rows, sweep:s.sweepSteering(input.sweep), constructions:input.constructions.map(d=>g.steeringConstruction(d)) }));
"""
        completed = subprocess.run(
            [node, "--input-type=module", "-e", script, (source_dir / "steering.js").as_uri(), (source_dir / "steering-geometry.js").as_uri()],
            input=json.dumps({"cases": cases, "sweep": stop, "constructions": construction_cases}),
            text=True, capture_output=True, check=True, timeout=30,
        )
        expected = json.loads(completed.stdout)
        self.assertEqual(len(expected["rows"]), 463)
        for index, ((source, angle), reference) in enumerate(zip(cases, expected["rows"])):
            with self.subTest(case=index, angle=angle):
                result = solve_steering(source, angle)
                self.assert_tree_close({"result": result, "construction": steering_construction(source, result)}, reference)
        self.assert_tree_close(sweep_steering(stop), expected["sweep"])
        for source, reference in zip(construction_cases, expected["constructions"]):
            self.assert_tree_close(steering_construction(source), reference)


if __name__ == "__main__":
    unittest.main()
