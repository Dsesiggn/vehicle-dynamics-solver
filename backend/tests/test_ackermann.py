"""Independent geometry, source-table checks and legacy JavaScript parity."""

import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vehicle_dynamics.models.ackermann import calculate_ackermann


class AckermannTests(unittest.TestCase):
    def test_seward_table_6_2(self):
        # Recorded source: printed p.163 / supplied PDF p.174, L=1600, t=1200 mm.
        # Independently recover input angles from the outside-wheel triangle;
        # the book inside angles are only printed to 0.1 degree.
        for outside, book_inside in [(5, 5.3), (10, 11.5), (15, 18.5), (20, 26.6), (25, 35.6)]:
            with self.subTest(outside=outside):
                radius = 1600 / math.tan(math.radians(outside)) - 600
                inside = math.degrees(math.atan2(1600, radius - 600))
                self.assertAlmostEqual(inside, book_inside, delta=0.05)
                sample = calculate_ackermann(1600, 1200, [inside])[0]
                self.assertAlmostEqual(sample["outside_angle_deg"], outside, delta=1e-12)

    def test_independent_wheel_normal_intersections(self):
        # In front-axle-local XY, wheel normals are [-sin(h), cos(h)].
        # Intersection with rear axle X=-L is y + L/tan(h).
        for direction in (-1, 1):
            for radius in (1500, 2500, 10000, 100000):
                inside = direction * math.degrees(math.atan2(1600, radius - 600))
                sample = calculate_ackermann(1600, 1200, [inside])[0]
                left_center = -600 + 1600 / math.tan(math.radians(sample["left_heading_deg"]))
                right_center = 600 + 1600 / math.tan(math.radians(sample["right_heading_deg"]))
                self.assertAlmostEqual(left_center, direction * radius, delta=2e-10)
                self.assertAlmostEqual(right_center, direction * radius, delta=2e-10)

    def test_sign_reflection_and_side_assignment(self):
        negative, positive = calculate_ackermann(1600, 1200, [-20, 20])
        self.assertEqual(positive["turn_direction"], "right")
        self.assertEqual(negative["turn_direction"], "left")
        self.assertEqual(positive["right_heading_deg"], 20)
        self.assertEqual(negative["left_heading_deg"], -20)
        self.assertEqual(positive["left_heading_deg"], -negative["right_heading_deg"])
        self.assertEqual(positive["outside_angle_deg"], -negative["outside_angle_deg"])

    def test_zero_and_negative_zero_are_exact(self):
        for sample in calculate_ackermann(1600, 1200, [0, -0.0]):
            self.assertEqual(sample["turn_direction"], "straight")
            for key, value in sample.items():
                if key != "turn_direction":
                    self.assertEqual(value, 0.0)
                    self.assertEqual(math.copysign(1, value), 1)

    def test_small_angle_limit(self):
        for inside in (1e-10, -1e-10):
            sample = calculate_ackermann(1600, 1200, [inside])[0]
            self.assertAlmostEqual(sample["outside_angle_deg"], inside, delta=2e-22)

    def test_subnormal_radian_conversion_preserves_tiny_headings(self):
        # These degree values survive JSON/Python but lose precision (or become
        # zero) on conversion to radians. For supported dimensions the true
        # inside/outside difference is smaller than floating-point resolution.
        for magnitude in (math.ulp(0.0), 1e-322, 1e-320, 1e-308):
            for wheelbase, track in ((1e-6, 1e7), (1e7, 1e-6)):
                for direction in (-1, 1):
                    inside = direction * magnitude
                    sample = calculate_ackermann(wheelbase, track, [inside])[0]
                    self.assertEqual(sample["inside_angle_deg"], inside)
                    self.assertEqual(sample["outside_angle_deg"], inside)
                    self.assertEqual(sample["left_heading_deg"], inside)
                    self.assertEqual(sample["right_heading_deg"], inside)
                    self.assertEqual(sample["turn_direction"], "right" if direction > 0 else "left")

    def test_maximum_supported_dimension_ratio(self):
        wheelbase, track = 1e-6, 1e7
        for inside in (1e-12, 1, 88.9):
            # Independent right triangles: outer lateral offset is the inner
            # offset plus the full track. No small-angle approximation is used.
            inner_offset = wheelbase / math.tan(math.radians(inside))
            expected = math.degrees(math.atan2(wheelbase, inner_offset + track))
            actual = calculate_ackermann(wheelbase, track, [inside])[0]["outside_angle_deg"]
            self.assertGreater(actual, 0)
            self.assertAlmostEqual(actual, expected, delta=1e-26)

    def test_near_angle_limit_and_monotonicity(self):
        angles = [0.001, 1, 10, 30, 60, 88, math.nextafter(89.0, 0.0)]
        samples = calculate_ackermann(1600, 1200, angles)
        outside = [sample["outside_angle_deg"] for sample in samples]
        self.assertTrue(all(0 < outer < inner for outer, inner in zip(outside, angles)))
        self.assertTrue(all(a < b for a, b in zip(outside, outside[1:])))
        self.assertTrue(all(math.isfinite(value) for value in outside))

    def test_common_length_scaling(self):
        expected = calculate_ackermann(1600, 1200, [-88, -20, 0, 20, 88])
        for factor in (1e-6, 1e-3, 1000):
            actual = calculate_ackermann(1600 * factor, 1200 * factor, [-88, -20, 0, 20, 88])
            for a, b in zip(actual, expected):
                self.assertAlmostEqual(a["outside_angle_deg"], b["outside_angle_deg"], delta=1e-12)

    def test_order_input_preservation_and_json_compatibility(self):
        angles = [25, -8, 0, 25]
        before = angles.copy()
        result = calculate_ackermann(1600, 1200, angles)
        self.assertEqual(angles, before)
        self.assertEqual([sample["inside_angle_deg"] for sample in result], angles)
        self.assertEqual(json.loads(json.dumps(result, allow_nan=False)), result)

    def test_dimension_bounds_and_strict_types(self):
        invalid = [0, -1, 1e-200, math.nextafter(1e-6, 0.0), 1e7 + 1, float("nan"), float("inf"), -float("inf"), None, True, False, "1600", [], {}, 10**1000]
        for value in invalid:
            for dimension in (0, 1):
                with self.subTest(value=repr(value), dimension=dimension):
                    dimensions = [1600, 1200]
                    dimensions[dimension] = value
                    with self.assertRaises(ValueError):
                        calculate_ackermann(*dimensions, [20])
        self.assertTrue(calculate_ackermann(1e7, 1e7, [20]))

    def test_lower_dimension_boundary(self):
        expected = calculate_ackermann(1, 1, [20])[0]["outside_angle_deg"]
        actual = calculate_ackermann(1e-6, 1e-6, [20])[0]["outside_angle_deg"]
        self.assertEqual(actual, expected)
        for wheelbase, track in ((1e-200, 1e7), (1e7, 1e-200)):
            with self.assertRaises(ValueError):
                calculate_ackermann(wheelbase, track, [1e-200])

    def test_angle_bounds_and_strict_types(self):
        for value in (-90, -89, 89, 90, float("nan"), float("inf"), -float("inf"), None, True, False, "20", [], {}, 10**1000):
            with self.subTest(value=repr(value)):
                with self.assertRaises(ValueError):
                    calculate_ackermann(1600, 1200, [value])

    def test_sample_container_and_count(self):
        for angles in (None, (), (20,), "20", 20, {"angle": 20}, [], [0] * 722):
            with self.subTest(angles=repr(angles)[:80]):
                with self.assertRaises(ValueError):
                    calculate_ackermann(1600, 1200, angles)
        self.assertEqual(len(calculate_ackermann(1600, 1200, [0] * 721)), 721)

    def test_javascript_reference_parity(self):
        node = os.environ.get("NODE") or shutil.which("node")
        if not node:
            self.skipTest("Set NODE to a Node.js executable to verify the unchanged JavaScript reference.")
        source = Path(__file__).resolve().parents[2] / "suspension-studio/src/steering.js"
        cases = []
        for wheelbase, track in ((1600, 1200), (1525, 1170), (1e7, 1), (1, 1e7)):
            for inside in [-88.999999, -1e-10, 0, 1e-10, 88.999999, *range(-88, 89)]:
                cases.append([wheelbase, track, inside])
        script = """
import { readFileSync } from 'node:fs';
const { idealOutsideAngle } = await import(process.argv[1]);
const cases = JSON.parse(readFileSync(0, 'utf8'));
process.stdout.write(JSON.stringify(cases.map(([L,t,a]) => Math.sign(a)*idealOutsideAngle(Math.abs(a),t,L))));
"""
        completed = subprocess.run(
            [node, "--input-type=module", "-e", script, source.as_uri()],
            input=json.dumps(cases), text=True, capture_output=True, check=True, timeout=30,
        )
        expected = json.loads(completed.stdout)
        for (wheelbase, track, inside), outside in zip(cases, expected):
            actual = calculate_ackermann(wheelbase, track, [inside])[0]["outside_angle_deg"]
            self.assertAlmostEqual(actual, outside, delta=3e-13)
        self.assertEqual(len(expected), len(cases))


if __name__ == "__main__":
    unittest.main()
