from __future__ import annotations

import subprocess
import sys
import unittest
from pathlib import Path

from lib.occid_bus import occid
from lib.uav_client import UavClient


class FakeBus:
    def __init__(self) -> None:
        self.client_id = "program"
        self.request_counter = 0
        self.published: list[tuple[str, dict]] = []

    def publish(self, topic: str, payload: dict) -> None:
        self.published.append((topic, payload))


class FakeRuntime:
    def __init__(self) -> None:
        self.bus = FakeBus()
        self.state = {}
        self.wait_calls: list[tuple[str, float]] = []

    def _wait_response(self, request_id: str, timeout_s: float) -> dict:
        self.wait_calls.append((request_id, timeout_s))
        return {"request_id": request_id, "ok": True, "data": {}}


class UavClientRoutingTests(unittest.TestCase):
    def setUp(self) -> None:
        self.runtime = FakeRuntime()
        self.uav = UavClient(
            self.runtime,
            {"id": "uav_controller", "topic_ns": "UAV"},
            response_timeout_s=10.0,
        )

    def test_convenience_command_dispatches_without_waiting(self) -> None:
        request_id = self.uav.arm()
        self.assertEqual(request_id, "req-1")
        self.assertEqual(self.runtime.wait_calls, [])
        topic, envelope = self.runtime.bus.published[-1]
        self.assertEqual(topic, "uav_controller/UAV/REQUEST")
        self.assertEqual(envelope["data"]["request_id"], "req-1")
        self.assertEqual(envelope["data"]["command"]["_occid_model"], "ArmCommand")

    def test_execute_waits_only_when_explicitly_requested(self) -> None:
        response = self.uav.execute(occid.ArmCommand(), timeout_s=3.0)
        self.assertTrue(response["ok"])
        self.assertEqual(self.runtime.wait_calls, [("req-1", 3.0)])

    def test_high_rate_attitude_uses_input_path_without_request_id(self) -> None:
        self.uav.set_attitude(0.1, -0.2, 0.3, 0.4)
        topic, envelope = self.runtime.bus.published[-1]
        self.assertEqual(topic, "uav_controller/UAV/INPUT")
        self.assertEqual(envelope["data"]["_occid_model"], "ControlAttitudeSetpoint")
        self.assertNotIn("request_id", envelope["data"])
        self.assertEqual(self.runtime.wait_calls, [])

    def test_generic_command_is_not_accepted_by_uav_service(self) -> None:
        with self.assertRaises(TypeError):
            self.uav.send(occid.Command())

    def test_direct_control_lifecycle_is_acknowledged_control_plane(self) -> None:
        begin = self.uav.begin_direct_control(occid.DirectControlMode.ATTITUDE_THRUST)
        end = self.uav.end_direct_control()
        self.assertTrue(begin["ok"])
        self.assertTrue(end["ok"])
        self.assertEqual(self.runtime.wait_calls, [("req-1", 10.0), ("req-2", 10.0)])
        models = [item[1]["data"]["command"]["_occid_model"] for item in self.runtime.bus.published]
        self.assertEqual(models, ["BeginDirectControlCommand", "EndDirectControlCommand"])


class RuntimeImportTests(unittest.TestCase):
    def test_main_prioritizes_mpfc_lib_when_occid_path_is_first(self) -> None:
        repo_root = Path(__file__).resolve().parents[1]
        occid_root = repo_root.parent / "occid"
        code = "\n".join(
            [
                "import runpy, sys",
                f"sys.path.insert(0, {str(occid_root)!r})",
                f"runpy.run_path({str(repo_root / 'main.py')!r}, run_name='mpfc_import_probe')",
                "import lib.common",
                f"assert lib.common.__file__.startswith({str(repo_root)!r}), lib.common.__file__",
            ]
        )
        result = subprocess.run(
            [sys.executable, "-c", code],
            capture_output=True,
            text=True,
            timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
