"""Strict HTTP boundaries and public fixed-axis steering response contract."""

from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fastapi.testclient import TestClient
from vehicle_dynamics.app import app
from steering_fixtures import design, path_design, tilted_design


class SteeringApiTests(unittest.TestCase):
    def setUp(self):
        self.client = TestClient(app)

    def post_solve(self, source=None, angle=0):
        return self.client.post("/api/v1/steering/solve", json={"design": source if source is not None else design(), "steeringWheelDeg": angle})

    def metadata(self, body):
        for key, value in (("model", "fixed_axis_steering_v1"), ("coordinate_system", "SAE_J670_Z_DOWN"),
                           ("hardpoint_frame", "AXLE_LOCAL"), ("length_unit", "mm"), ("angle_unit", "deg")):
            self.assertEqual(body[key], value)

    def test_success_contract_and_construction_use_solved_pose(self):
        response = self.post_solve(tilted_design(), 120)
        self.assertEqual(response.status_code, 200, response.text)
        body = response.json()
        self.metadata(body)
        result = body["result"]
        self.assertTrue(result["ok"])
        self.assertEqual(result["steeringWheelDeg"], 120)
        self.assertEqual(result["rack"], -20)
        self.assertEqual(result["construction"]["pose"], "solved")
        for side in ("left", "right"):
            self.assertEqual(result["construction"][side]["outer"], result[side]["outer"])
            self.assertAlmostEqual(math.dist(result[side]["outer"], result[side]["inner"]), result[side]["tieLength"], delta=1e-9)
        json.dumps(body, allow_nan=False)

    def test_five_degree_sweep_and_failed_rows_are_retained(self):
        source = design()
        source["steeringLimitDeg"] = 20
        source["rackTravel"] = 2
        response = self.client.post("/api/v1/steering/sweep", json={"design": source})
        self.assertEqual(response.status_code, 200, response.text)
        body = response.json()
        self.metadata(body)
        self.assertEqual(body["step_deg"], 5)
        self.assertEqual([row["steeringWheelDeg"] for row in body["rows"]], list(range(-20, 21, 5)))
        self.assertFalse(body["rows"][0]["ok"])
        self.assertTrue(body["rows"][4]["ok"])
        self.assertFalse(body["rows"][-1]["ok"])
        self.assertEqual(body["rows"][0]["construction"]["pose"], "static")

    def test_physical_failures_are_200_without_fabricated_wheel_results(self):
        degenerate = design()
        degenerate["left"]["axisLower"] = deepcopy(degenerate["left"]["axisUpper"])
        limited = design()
        limited["rackTravel"] = 2
        for source, angle in ((degenerate, 0), (limited, 180), (path_design(0.5), 4), (path_design(1), 4)):
            response = self.post_solve(source, angle)
            self.assertEqual(response.status_code, 200, response.text)
            result = response.json()["result"]
            self.assertFalse(result["ok"])
            self.assertTrue(result["reason"])
            for key in ("left", "right", "metrics"):
                self.assertIsNone(result[key])
            self.assertEqual(result["construction"]["pose"], "static")

    def test_numerically_unrepresentable_metric_is_explicit_200_failure(self):
        for key, metric in (("frontTrack", "ackermannPercent"), ("wheelbase", "cotResidual")):
            source = design()
            source[key] = math.ulp(0.0)
            response = self.post_solve(source, 180)
            self.assertEqual(response.status_code, 200, response.text)
            result = response.json()["result"]
            self.assertFalse(result["ok"])
            self.assertIn(metric, result["reason"])
            self.assertIsNone(result["metrics"])
            self.assertEqual(result["construction"]["pose"], "static")
            json.dumps(response.json(), allow_nan=False)

    def test_missing_metadata_values_and_points_are_422(self):
        bad_cases = []
        for key in ("coordinateSystem", "kind", "name", "wheelbase", "left"):
            source = design()
            del source[key]
            bad_cases.append(source)
        for key, value in (("version", True), ("lengthUnit", "m"), ("wheelbase", 0),
                           ("frontTrack", "1200"), ("rearTrack", None), ("rackMmPerRevolution", False),
                           ("rackTravel", 1e7 + 1), ("steeringLimitDeg", 6)):
            source = design()
            source[key] = value
            bad_cases.append(source)
        for field, value in (("axisUpper", [0, 0]), ("tieInner", [0, None, 0]), ("tieOuter", [0, True, 0]),
                             ("staticToeDeg", 90), ("staticCamberDeg", "0"), ("wheelCenterZ", None)):
            source = design()
            source["left"][field] = value
            bad_cases.append(source)
        for index, source in enumerate(bad_cases):
            for endpoint, payload in (("solve", {"design": source, "steeringWheelDeg": 0}), ("sweep", {"design": source})):
                with self.subTest(case=index, endpoint=endpoint):
                    response = self.client.post(f"/api/v1/steering/{endpoint}", json=payload)
                    self.assertEqual(response.status_code, 422, response.text)
                    self.assertIn("detail", response.json())

    def test_requested_angle_and_wrapper_are_strict(self):
        for angle in (None, True, False, "0", [], {}, 3600.001, -3600.001):
            self.assertEqual(self.post_solve(angle=angle).status_code, 422)
        for payload in ({}, {"design": design()}, {"design": design(), "steeringWheelDeg": 0, "extra": 1}, [], None):
            response = self.client.post("/api/v1/steering/solve", json=payload)
            self.assertEqual(response.status_code, 422, response.text)
        self.assertEqual(self.client.post("/api/v1/steering/sweep", json={"design": design(), "step_deg": 10}).status_code, 422)

    def test_nonfinite_malformed_and_huge_numbers_are_client_errors(self):
        for value in (float("nan"), float("inf"), -float("inf"), 10**1000):
            for target in ("angle", "coordinate", "dimension"):
                source = design()
                angle = 0
                if target == "angle":
                    angle = value
                elif target == "coordinate":
                    source["left"]["tieInner"][0] = value
                else:
                    source["wheelbase"] = value
                raw = json.dumps({"design": source, "steeringWheelDeg": angle})
                response = self.client.post("/api/v1/steering/solve", content=raw, headers={"Content-Type": "application/json"})
                self.assertEqual(response.status_code, 422, response.text)
                json.dumps(response.json(), allow_nan=False)
        self.assertEqual(self.client.post("/api/v1/steering/solve", content="{bad json", headers={"Content-Type": "application/json"}).status_code, 422)

    def test_extension_fields_remain_compatible_without_affecting_geometry(self):
        source = design()
        expected = self.post_solve(source, 90).json()
        source["extension"] = {"notes": "saved by another editor"}
        source["left"]["extension"] = [1, 2, 3]
        response = self.post_solve(source, 90)
        self.assertEqual(response.status_code, 200, response.text)
        self.assertEqual(response.json(), expected)

    def test_openapi_and_health_advertise_new_model(self):
        health = self.client.get("/api/v1/health").json()
        self.assertIn("fixed_axis_steering_v1", health["models"])
        paths = self.client.get("/api/v1/openapi.json").json()["paths"]
        self.assertIn("/api/v1/steering/solve", paths)
        self.assertIn("/api/v1/steering/sweep", paths)


if __name__ == "__main__":
    unittest.main()
