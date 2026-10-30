"""Deployment identity and browser freshness, independently of numerical models."""
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from fastapi.testclient import TestClient
from vehicle_dynamics.app import create_app


class DeploymentTests(unittest.TestCase):
    def health_with_environment(self, values):
        with patch.dict(os.environ, values, clear=True):
            client = TestClient(create_app())
        # Request metadata must not be able to claim a different deployed source.
        return client.get("/api/v1/health?source_commit=" + "c" * 40,
                          headers={"APP_COMMIT_SHA": "d" * 40}).json()

    def test_source_commit_uses_host_environment_with_explicit_precedence(self):
        render, generic = "a" * 40, "b" * 40
        result = self.health_with_environment({"RENDER_GIT_COMMIT": render, "APP_COMMIT_SHA": generic})
        self.assertEqual(result["source_commit"], render)
        self.assertEqual(result["source_commit_source"], "RENDER_GIT_COMMIT")
        result = self.health_with_environment({"APP_COMMIT_SHA": generic})
        self.assertEqual(result["source_commit"], generic)
        self.assertEqual(result["source_commit_source"], "APP_COMMIT_SHA")

    def test_unset_or_invalid_source_is_unknown_without_local_version_guesses(self):
        for values in ({}, {"RENDER_GIT_COMMIT": ""}, {"APP_COMMIT_SHA": "main"},
                       {"RENDER_GIT_COMMIT": "a" * 7}, {"APP_COMMIT_SHA": "g" * 40},
                       {"RENDER_GIT_COMMIT": "a" * 41}, {"APP_COMMIT_SHA": "a" * 40 + "\nextra"}):
            with self.subTest(environment=values):
                result = self.health_with_environment(values)
                self.assertIsNone(result["source_commit"])
                self.assertIsNone(result["source_commit_source"])
                self.assertEqual(result["status"], "ok")

    def test_valid_generic_fallback_and_full_sha256_are_preserved(self):
        result = self.health_with_environment({"RENDER_GIT_COMMIT": "invalid", "APP_COMMIT_SHA": "  " + "AB" * 32 + "\n"})
        self.assertEqual(result["source_commit"], "ab" * 32)
        self.assertEqual(result["source_commit_source"], "APP_COMMIT_SHA")

    def test_mutable_pages_source_methods_and_api_responses_are_not_stored(self):
        client = TestClient(create_app())
        for path in ("/", "/suspension-studio", "/suspension-studio/",
                     "/suspension-studio/index.html", "/suspension-studio/steering.html",
                     "/suspension-studio/engineering.html", "/suspension-studio/src/steering-app.js",
                     "/suspension-studio/src/steering.css", "/suspension-studio/docs/STEERING.md",
                     "/api/v1/health", "/api/v1/openapi.json", "/api/v1/missing",
                     "/suspension-studio/src/missing.js"):
            with self.subTest(path=path):
                response = client.get(path, follow_redirects=False)
                self.assertEqual(response.headers["cache-control"], "no-store")
                self.assertNotIn("etag", response.headers)
                self.assertNotIn("last-modified", response.headers)
        payload = {"wheelbase_mm": 1600, "front_track_mm": 1200, "inside_angles_deg": [30]}
        success = client.post("/api/v1/ackermann", json=payload)
        invalid = client.post("/api/v1/steering/sweep", json={"design": {}})
        self.assertEqual(success.status_code, 200)
        self.assertEqual(invalid.status_code, 422)
        for response in (success, invalid):
            self.assertEqual(response.headers["cache-control"], "no-store")

    def test_changed_module_is_fresh_even_with_old_validators_and_identical_file_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            frontend = Path(directory)
            (frontend / "src").mkdir()
            (frontend / "vendor").mkdir()
            source = frontend / "src" / "module.js"
            vendor = frontend / "vendor" / "fixture.js"
            source.write_text("export const revision = 1;")
            vendor.write_text(source.read_text())
            stamp = source.stat().st_mtime_ns
            os.utime(vendor, ns=(stamp, stamp))
            client = TestClient(create_app(frontend))
            # Vendor has the old standard file validators. Match file size and
            # mtime exactly, then change only source content as a deploy can do.
            old = client.get("/suspension-studio/vendor/fixture.js")
            headers = {"If-None-Match": old.headers["etag"],
                       "If-Modified-Since": old.headers["last-modified"]}
            source.write_text("export const revision = 2;")
            os.utime(source, ns=(stamp, stamp))
            current = client.get("/suspension-studio/src/module.js", headers=headers)
            self.assertEqual(current.status_code, 200)
            self.assertEqual(current.text, "export const revision = 2;")
            self.assertEqual(current.headers["cache-control"], "no-store")
            # Bundled libraries keep their existing conditional-request behavior.
            unchanged = client.get("/suspension-studio/vendor/fixture.js", headers=headers)
            self.assertEqual(unchanged.status_code, 304)
            self.assertNotIn("cache-control", unchanged.headers)


if __name__ == "__main__":
    unittest.main()
