"""Independent synthetic geometry checks, not measured-vehicle validation."""

import json
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vehicle_dynamics.models.kinematic_geometry import (
    circle_contact, instantaneous_screw_axis, axis_plane_intersection,
    front_view_instant_center, side_view_instant_center,
    kinematic_roll_center, kinematic_pitch_center, steering_geometry,
)


def axis(point, direction=(1, 0, 0), pitch=0):
    return {"available": True, "point": list(point), "direction": list(direction),
            "pitchMmPerRad": pitch}


class KinematicGeometryTests(unittest.TestCase):
    def close_vector(self, actual, expected, tolerance=1e-9):
        self.assertEqual(len(actual), len(expected))
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, delta=tolerance)

    def test_circle_known_three_four_five_support(self):
        # Spindle (0,-3/5,4/5); lowest circle direction is (0,4/5,3/5).
        result = circle_contact([10, -600, -250], [0, -0.6, 0.8], 250)
        self.assertTrue(result["available"])
        self.close_vector(result["point"], [10, -400, -100])
        radius = np.array(result["point"]) - [10, -600, -250]
        self.assertAlmostEqual(np.linalg.norm(radius), 250)
        self.assertAlmostEqual(np.dot(radius, [0, -0.6, 0.8]), 0)
        # An independent parameterization of the circle confirms maximal Z.
        for t in np.linspace(0, 2 * math.pi, 129):
            point_z = -250 + 150 * math.cos(t)
            self.assertGreaterEqual(result["point"][2] + 1e-10, point_z)

    def test_circle_spindle_sign_scale_and_translation_invariants(self):
        left = circle_contact([0, -600, -250], [0, -1, 0], 250)
        right = circle_contact([0, 600, -250], [0, 2, 0], 250)
        self.close_vector(left["point"], [0, -600, 0])
        self.close_vector(right["point"], [0, 600, 0])
        base = circle_contact([10, 20, 30], [0, -3, 4], 100)
        flipped = circle_contact([10, 20, 30], [0, 6, -8], 100)
        self.close_vector(base["point"], flipped["point"])

    def test_circle_invalid_and_nonunique_inputs_are_unavailable(self):
        for center, spindle, radius in [([0, 0, 0], [0, 0, 1], 200),
                                       ([0, 0, 0], [0, 0, 0], 200),
                                       ([0, 0, 0], [0, 1, 0], 0),
                                       ([0, 0], [0, 1, 0], 200),
                                       ([0, math.nan, 0], [0, 1, 0], 200)]:
            self.assertFalse(circle_contact(center, spindle, radius)["available"])

    def test_inclined_screw_axis_known_velocity_field(self):
        # A=(10,20,-30), k=(3/5,0,4/5), |w|=3, pitch=7.
        # C-A=(4,5,-3) is perpendicular to k. Direct component cross product
        # w cross (C-A)=(-12,15,9), plus pitch*w=(12.6,0,16.8).
        result = instantaneous_screw_axis([14, 25, -33], [0.6, 15, 25.8], [1.8, 0, 2.4])
        self.assertTrue(result["available"])
        self.close_vector(result["point"], [10, 20, -30])
        self.close_vector(result["direction"], [0.6, 0, 0.8])
        self.assertAlmostEqual(result["pitchMmPerRad"], 7)
        self.close_vector(result["parallelVelocity"], [12.6, 0, 16.8])
        # The axis is not motionless when screw pitch is nonzero.
        for distance in [-200, 0, 300]:
            p = [10 + .6 * distance, 20, -30 + .8 * distance]
            dx, dy, dz = [p[i] - [14, 25, -33][i] for i in range(3)]
            velocity = [.6 - 2.4 * dy, 15 + 2.4 * dx - 1.8 * dz, 25.8 + 1.8 * dy]
            self.close_vector(velocity, [12.6, 0, 16.8], 1e-10)

    def test_screw_axis_parameter_and_origin_invariants(self):
        center = np.array([14., 25., -33.])
        velocity = np.array([.6, 15., 25.8])
        omega = np.array([1.8, 0., 2.4])
        shift = np.array([1e6, -2e6, 3e6])
        for scale in [-2.5, 1e-6, 10]:
            result = instantaneous_screw_axis(center + shift, velocity * scale, omega * scale)
            self.assertTrue(result["available"])
            self.close_vector(result["point"], np.array([10, 20, -30]) + shift, 1e-8)
            self.assertAlmostEqual(result["pitchMmPerRad"], 7)
            self.close_vector(result["parallelVelocity"], scale * np.array([12.6, 0, 16.8]))

    def test_pure_translation_and_stationary_twists_do_not_invent_axes(self):
        for linear in ([0, 0, -1], [0, 0, 0]):
            result = instantaneous_screw_axis([0, -600, -250], linear, [0, 0, 0])
            self.assertFalse(result["available"])
            self.assertIn("no unique finite", result["reason"])
        self.assertFalse(instantaneous_screw_axis([0, 0, 0], [0, 0, math.inf], [1, 0, 0])["available"])

    def test_front_and_side_plane_intersections_are_not_axis_projections(self):
        known = axis([10, 20, 30], [1, 2, 3])
        self.close_vector(front_view_instant_center(known, [15, 9, 0])["point"], [15, 30, 45])
        self.close_vector(side_view_instant_center(known, [0, 12, 0])["point"], [6, 12, 18])
        self.assertFalse(front_view_instant_center(axis([0, 0, 0], [0, 1, 0]), [0, 0, 0])["available"])
        self.assertFalse(side_view_instant_center(axis([0, 0, 0]), [0, 0, 0])["available"])
        self.assertFalse(axis_plane_intersection(known, [0, 0, 0], [0, 0, 0])["available"])

    def test_symmetric_roll_center_hand_calculation(self):
        # Each side rises 280 mm over 2100 mm toward its IC; at centerline
        # the 600-mm run gives height 600*280/2100 = 80 mm.
        result = kinematic_roll_center(axis([0, 1500, -280]), [0, -600, 0],
                                       axis([0, -1500, -280]), [0, 600, 0])
        self.assertTrue(result["available"], result)
        self.close_vector(result["point"], [0, 0, -80])
        self.assertLess(result["planeResidualMm"], 1e-10)

    def test_asymmetric_roll_center_has_real_lateral_migration(self):
        # z=-(y+600)/7 and z=5*(y-600)/44 meet at
        # y=-5400/79, z=-6000/79; no forced centerline restriction.
        result = kinematic_roll_center(axis([0, 1500, -300]), [0, -600, 0],
                                       axis([0, -1600, -250]), [0, 600, 0])
        self.assertTrue(result["available"])
        self.close_vector(result["point"], [0, -5400/79, -6000/79])

    def test_pitch_center_independent_side_view_construction(self):
        # Front line z=x/8, rear line z=-(x+1600)/8.
        result = kinematic_pitch_center(axis([-1600, -600, -200], [0, 1, 0]), [0, -600, 0],
                                        axis([0, -600, -200], [0, 1, 0]), [-1600, -600, 0])
        self.assertTrue(result["available"])
        self.close_vector(result["point"], [-800, -600, -100])

    def test_spatial_axle_plane_and_global_translation(self):
        # Contact chord has changing X and Z. The vertical reference plane
        # enforces x=10-(y+600)/40, so y=40 implies x=-6.
        wanted = np.array([-6., 40., -60.])
        ca, cb = np.array([10., -600., 20.]), np.array([-20., 600., -10.])
        shift = np.array([-1530., 15., 80.])
        for offset in (np.zeros(3), shift):
            a = axis(wanted + [100, 0, 0] + offset, pitch=2.5)
            b = axis(wanted + [-100, 0, 0] + offset, pitch=-1.25)
            result = kinematic_roll_center(a, ca + offset, b, cb + offset)
            self.assertTrue(result["available"], result)
            self.close_vector(result["point"], wanted + offset)
            self.assertEqual(result["screwPitchesMmPerRad"], [2.5, -1.25])

    def test_center_missing_degenerate_and_conditioned_planes(self):
        left_contact, right_contact = [0, -600, 0], [0, 600, 0]
        cases = [
            ({"available": False, "reason": "Translation."}, axis([0, -1500, -280])),
            (axis(left_contact), axis([0, -1500, -280])),
            (axis([0, 1500, 0]), axis([0, -1500, 0])),
            (axis([0, 1500, -1e-7]), axis([0, -1500, -1e-7])),
        ]
        for a, b in cases:
            with self.subTest(a=a, b=b):
                result = kinematic_roll_center(a, left_contact, b, right_contact)
                self.assertFalse(result["available"], result)
        same_contact = kinematic_roll_center(axis([0, 1500, -280]), left_contact,
                                             axis([0, -1500, -280]), left_contact)
        self.assertFalse(same_contact["available"])

    def test_bounded_zero_pitch_limit_approaches_ground_but_zero_is_undefined(self):
        # Analytic family: opposing tiny X rotations with vC=(0,0,-1),
        # IC distance 1/epsilon. Ground-center height tends 150000*epsilon.
        for epsilon in [1e-4, 1e-5, 1e-6]:
            left = instantaneous_screw_axis([0, -600, -250], [0, 0, -1], [epsilon, 0, 0])
            right = instantaneous_screw_axis([0, 600, -250], [0, 0, -1], [-epsilon, 0, 0])
            result = kinematic_roll_center(left, [0, -600, 0], right, [0, 600, 0])
            self.assertTrue(result["available"], result)
            self.close_vector(result["point"], [0, 0, -150000 * epsilon], 1e-8)
        undefined = instantaneous_screw_axis([0, -600, -250], [0, 0, -1], [0, 0, 0])
        self.assertFalse(kinematic_roll_center(undefined, [0, -600, 0], undefined, [0, 600, 0])["available"])

    def test_physical_steering_geometry_hand_values_and_mirror(self):
        for side, s in [("left", -1), ("right", 1)]:
            result = steering_geometry([0, s*500, -100], [-20, s*480, -300],
                                       [0, s*600, 0], [0, s, 0], side)
            self.assertTrue(result["available"])
            self.assertAlmostEqual(result["casterDeg"], math.degrees(math.atan(.1)))
            self.assertAlmostEqual(result["kingpinInclinationDeg"], math.degrees(math.atan(.1)))
            self.assertAlmostEqual(result["mechanicalTrailMm"], 10)
            self.assertAlmostEqual(result["scrubRadiusMm"], 90)
            self.close_vector(result["axisGroundPoint"], [10, s*510, 0])

    def test_trail_and_scrub_follow_wheel_heading_not_vehicle_axes(self):
        angle = math.radians(30)
        f = np.array([math.cos(angle), math.sin(angle), 0])
        outward = np.array([math.sin(angle), -math.cos(angle), 0])
        ground = np.array([10., -510., 0.])
        contact = ground - 12 * f + 30 * outward
        result = steering_geometry([0, -500, -100], [-20, -480, -300], contact,
                                   outward, "left")
        self.assertTrue(result["available"])
        self.assertAlmostEqual(result["mechanicalTrailMm"], 12)
        self.assertAlmostEqual(result["scrubRadiusMm"], 30)

    def test_steering_intersection_degeneracy_and_json_serialization(self):
        for lower, upper, spindle in [([0, 0, 0], [0, 0, 0], [0, -1, 0]),
                                      ([0, 0, -100], [10, 0, -100], [0, -1, 0]),
                                      ([0, 0, -100], [0, 0, -200], [0, 0, 1])]:
            self.assertFalse(steering_geometry(lower, upper, [0, -600, 0], spindle, "left")["available"])
        json.dumps(kinematic_roll_center(axis([0, 1500, -280]), [0, -600, 0],
                                         axis([0, -1500, -280]), [0, 600, 0]), allow_nan=False)
        json.dumps(instantaneous_screw_axis([14, 25, -33], [.6, 15, 25.8], [1.8, 0, 2.4]), allow_nan=False)


if __name__ == "__main__":
    unittest.main()
