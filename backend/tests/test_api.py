"""Transport, validation, and explicit public-file boundary checks."""
import unittest
from fastapi.testclient import TestClient
from vehicle_dynamics.app import app


class ApiTests(unittest.TestCase):
    def setUp(self):
        self.client = TestClient(app)
        self.payload = {"wheelbase_mm": 1600, "front_track_mm": 1200,
                        "inside_angles_deg": [-30, 0, 30]}

    def test_response_contract(self):
        result = self.client.post("/api/v1/ackermann", json=self.payload)
        self.assertEqual(result.status_code, 200)
        body = result.json()
        self.assertEqual(body["model"], "ideal_ackermann_v1")
        self.assertEqual(body["coordinate_system"], "SAE_J670_Z_DOWN")
        self.assertEqual([s["turn_direction"] for s in body["samples"]], ["left", "straight", "right"])

    def test_invalid_values_are_rejected(self):
        for key, value in [("wheelbase_mm", 0), ("wheelbase_mm", "1600"),
                           ("wheelbase_mm", 1e-200),
                           ("front_track_mm", True), ("inside_angles_deg", []),
                           ("inside_angles_deg", [89]), ("inside_angles_deg", [None]),
                           ("inside_angles_deg", ["10"]), ("inside_angles_deg", [True]),
                           ("inside_angles_deg", [0] * 722), ("unexpected", 1)]:
            with self.subTest(key=key, value=str(value)[:80]):
                self.assertEqual(self.client.post("/api/v1/ackermann", json={**self.payload, key: value}).status_code, 422)

    def test_nonfinite_and_malformed_json_produce_client_error(self):
        for raw in ['{"wheelbase_mm":NaN,"front_track_mm":1200,"inside_angles_deg":[10]}',
                    '{"wheelbase_mm":1600,"front_track_mm":1200,"inside_angles_deg":[Infinity]}',
                    '{bad json']:
            result = self.client.post("/api/v1/ackermann", content=raw,
                                      headers={"Content-Type": "application/json"})
            self.assertEqual(result.status_code, 422)
            self.assertIn("detail", result.json())

    def test_health_and_frontend_assets(self):
        self.assertEqual(self.client.get("/api/v1/health").json()["status"], "ok")
        for path in ["/", "/suspension-studio/", "/suspension-studio/steering.html",
                     "/suspension-studio/src/steering.js", "/api/v1/openapi.json",
                     "/suspension-studio/docs/STEERING.md",
                     "/suspension-studio/README.md",
                     "/suspension-studio/vendor/three/three.module.js",
                     "/suspension-studio/vendor/plotly/plotly.min.js"]:
            with self.subTest(path=path):
                self.assertEqual(self.client.get(path).status_code, 200)

    def test_private_files_not_served(self):
        for path in ["/AGENTS.md", "/backend/vehicle_dynamics/app.py", "/Dynamics_Dashboard.html",
                     "/suspension-studio/package.json", "/suspension-studio/tests/model.test.mjs",
                     "/suspension-studio/src/%2e%2e/%2e%2e/AGENTS.md", "/api/v1/missing"]:
            with self.subTest(path=path):
                self.assertEqual(self.client.get(path).status_code, 404)


if __name__ == "__main__":
    unittest.main()
