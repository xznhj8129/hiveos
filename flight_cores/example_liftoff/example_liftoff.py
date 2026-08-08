#!/usr/bin/env python3
"""OCCID telemetry snapshot program for the Liftoff simulator backend."""

from __future__ import annotations

import time
from typing import Any, Dict

from lib.common import apply_cfg
from lib.core_base import CoreBase
from lib.occid_bus import occid
from lib.occid_topics import ATTITUDE, FLIGHT_CONTROL, LOCATION, POWER, RC_TELEMETRY
from lib.uav_client import UavClient


class ExampleLiftoffCore(CoreBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)
        self.uav = UavClient(self, cfg["interface"], float(self.response_timeout_s))
        self.state_keys = [FLIGHT_CONTROL, LOCATION, ATTITUDE, RC_TELEMETRY, POWER]
        self.init_bus(float(self.poll_interval_s), self.uav.state_topics(self.state_keys), self.uav.response_topic)

    def _print_snapshot(self) -> None:
        print("\n=== Liftoff/OCCID Snapshot ===", flush=True)
        print(f"flight_control: {self.uav.state(FLIGHT_CONTROL, occid.FlightControlState)}", flush=True)
        print(f"location: {self.uav.state(LOCATION, occid.LocationState)}", flush=True)
        print(f"attitude: {self.uav.state(ATTITUDE, occid.EulerAngles)}", flush=True)
        print(f"rc_telemetry: {self.uav.state(RC_TELEMETRY, occid.ControlAxisSet)}", flush=True)
        print(f"power: {self.uav.state(POWER, occid.ElectricalResourceState)}", flush=True)

    def run(self) -> None:
        self.send_online()
        self.wait_until(
            lambda: self.uav.flight_control() is not None,
            float(self.state_timeout_s),
            RuntimeError("Liftoff OCCID state timeout"),
        )
        print(f"[CORE] {self.client_id} liftoff_uav_state_online=True", flush=True)
        last_print = 0.0
        try:
            while True:
                self._pump_once()
                now = time.monotonic()
                if now - last_print < float(self.print_interval_s):
                    continue
                last_print = now
                self._print_snapshot()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()


def run_core(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    ExampleLiftoffCore(cfg, bus_config).run()
