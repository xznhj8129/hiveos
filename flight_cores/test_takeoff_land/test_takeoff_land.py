#!/usr/bin/env python3
"""OCCID-native takeoff/goto/RTL/land acceptance core."""

from __future__ import annotations

import math
import time
import traceback
from typing import Any, Dict

from lib.common import apply_cfg, build_envelope, build_request_topic, build_response_topic, build_state_topics, build_topic_base
from lib.core_base import CoreBase
from lib.geo_utils import GPSposition, gps_distance_m, vector_to_gps
from lib.occid_bus import get_occid_state, occid, send_occid_command
from lib.occid_topics import ANGULAR_VELOCITY, ATTITUDE, FLIGHT_CONTROL, LOCATION


class MissionAbort(RuntimeError):
    pass


class TakeoffLandCore(CoreBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)
        mission_cfg = cfg["mission"]
        apply_cfg(self, mission_cfg)
        self.poll_interval_s = float(mission_cfg["poll_interval_s"])
        interface_cfg = cfg["interface"]
        self.interface_id = interface_cfg["id"]
        self.interface_ns = interface_cfg["topic_ns"]

        base = build_topic_base(self.interface_id, self.interface_ns)
        self.request_topic = build_request_topic(self.interface_id, self.interface_ns)
        self.response_topic = build_response_topic(self.interface_id, self.interface_ns)
        self.state_topics = build_state_topics(base, self.state_keys)
        self.init_bus(self.poll_interval_s, self.state_topics, self.response_topic)

    def _flight_control(self) -> Any | None:
        return get_occid_state(self.state, FLIGHT_CONTROL, occid.FlightControlState)

    def _location(self) -> Any | None:
        return get_occid_state(self.state, LOCATION, occid.LocationState)

    def _attitude(self) -> Any | None:
        return get_occid_state(self.state, ATTITUDE, occid.EulerAngles)

    def _angular_velocity(self) -> Any | None:
        return get_occid_state(self.state, ANGULAR_VELOCITY, occid.AngularVelocityVector)

    def _send_command(self, command: Any) -> str:
        return send_occid_command(self.bus, self.request_topic, command)

    def _require_ok(self, request_id: str, label: str) -> Dict[str, Any]:
        response = self._wait_response(request_id, float(self.response_timeout_s))
        if not response.get("ok"):
            raise MissionAbort(f"{label} failed resp={response}")
        return response

    def _relative_altitude(self, location: Any | None = None) -> float | None:
        state = self._location() if location is None else location
        if state is None or state.altitude is None:
            return None
        return state.altitude.relative_m

    def _gps_position(self, location: Any) -> GPSposition:
        if location.position is None:
            raise MissionAbort("location state has no global position")
        relative_altitude = self._relative_altitude(location)
        if relative_altitude is None:
            raise MissionAbort("location state has no relative altitude")
        return GPSposition(
            float(location.position.lat),
            float(location.position.lon),
            float(relative_altitude),
        )

    def _validate_frame_contract(self) -> None:
        attitude = self._attitude()
        angular_velocity = self._angular_velocity()
        if attitude is None or angular_velocity is None:
            raise MissionAbort("attitude/angular-velocity state unavailable")
        if attitude.body_frame != occid.BodyReferenceFrame.FRD:
            raise MissionAbort(f"attitude body frame must be FRD actual={attitude.body_frame}")
        if attitude.reference_frame != occid.InertialReferenceFrame.NED:
            raise MissionAbort(f"attitude reference frame must be NED actual={attitude.reference_frame}")
        if angular_velocity.frame != occid.BodyReferenceFrame.FRD:
            raise MissionAbort(f"angular velocity frame must be FRD actual={angular_velocity.frame}")
        print(
            f"[CORE] {self.client_id} frames body=FRD reference=NED "
            f"attitude_rad=({attitude.roll_rad:.4f},{attitude.pitch_rad:.4f},{attitude.yaw_rad:.4f})",
            flush=True,
        )

    def run(self) -> None:
        self.send_online()
        try:
            self.wait_until(
                lambda: (
                    self._flight_control() is not None
                    and self._flight_control().navigation_validity is not None
                    and bool(self._flight_control().navigation_validity.home_position_ok)
                    and bool(self._flight_control().navigation_validity.global_position_ok)
                    and self._location() is not None
                    and self._attitude() is not None
                    and self._angular_velocity() is not None
                ),
                float(self.state_timeout_s),
                MissionAbort("initial OCCID flight state wait timed out"),
            )
            self._validate_frame_contract()

            flight = self._flight_control()
            print(
                f"[CORE] {self.client_id} health home_ok={flight.navigation_validity.home_position_ok} "
                f"global_ok={flight.navigation_validity.global_position_ok} mode={flight.standard_mode} "
                f"native_mode={flight.native_mode_name}",
                flush=True,
            )
            home_pos = self._gps_position(self._location())

            request_id = self._send_command(
                occid.SetTakeoffAltitudeCommand(relative_altitude_m=float(self.takeoff_altitude_m))
            )
            print(
                f"[CORE] {self.client_id} cmd SetTakeoffAltitudeCommand alt_m={self.takeoff_altitude_m} req={request_id}",
                flush=True,
            )
            self._require_ok(request_id, "set_takeoff_altitude")

            self.wait_until(
                lambda: (
                    self._flight_control() is not None
                    and self._flight_control().readiness is not None
                    and bool(self._flight_control().readiness.arm_ready)
                ),
                float(self.state_timeout_s),
                MissionAbort(f"arm readiness wait timed out flight_control={self._flight_control()}"),
            )
            print(f"[CORE] {self.client_id} arm_ready readiness={self._flight_control().readiness}", flush=True)

            request_id = self._send_command(occid.ArmCommand())
            print(f"[CORE] {self.client_id} cmd ArmCommand req={request_id}", flush=True)
            self._require_ok(request_id, "arm")

            self.wait_until(
                lambda: self._flight_control() is not None and bool(self._flight_control().armed),
                float(self.state_timeout_s),
                MissionAbort("armed wait timed out"),
            )
            print(f"[CORE] {self.client_id} armed", flush=True)

            self.wait_until(
                lambda: (
                    self._flight_control() is not None
                    and bool(self._flight_control().armed)
                    and self._flight_control().readiness is not None
                    and bool(self._flight_control().readiness.takeoff_ready)
                ),
                float(self.state_timeout_s),
                MissionAbort(f"takeoff readiness wait timed out flight_control={self._flight_control()}"),
            )
            print(f"[CORE] {self.client_id} takeoff_ready readiness={self._flight_control().readiness}", flush=True)

            request_id = self._send_command(occid.TakeoffCommand())
            print(f"[CORE] {self.client_id} cmd TakeoffCommand req={request_id}", flush=True)
            self._require_ok(request_id, "takeoff")

            self.wait_until(
                lambda: (
                    self._flight_control() is not None
                    and bool(self._flight_control().in_air)
                    and self._relative_altitude() is not None
                    and float(self._relative_altitude())
                    >= float(self.takeoff_altitude_m) * float(self.takeoff_altitude_ok_fraction)
                ),
                float(self.state_timeout_s),
                MissionAbort("in-air wait timed out"),
            )
            print(f"[CORE] {self.client_id} in_air alt_m={self._relative_altitude()}", flush=True)
            self.pump_for(float(self.post_takeoff_wait_s))

            goto_start = self._gps_position(self._location())
            goto_target = vector_to_gps(goto_start, dist=float(self.go_north_distance_m), az=0.0)
            request_id = self._send_command(
                occid.GoToCommand(
                    position=occid.GlobalPosition(
                        lat=float(goto_target.lat),
                        lon=float(goto_target.lon),
                        alt=float(self.takeoff_altitude_m),
                        alt_frame=occid.AltitudeDatum.RELATIVE,
                    ),
                    yaw_rad=math.radians(float(self.goto_yaw_deg)),
                )
            )
            print(
                f"[CORE] {self.client_id} cmd GoToCommand north_m={self.go_north_distance_m} "
                f"lat={goto_target.lat} lon={goto_target.lon} req={request_id}",
                flush=True,
            )
            self._require_ok(request_id, "go_to")

            target_distance_m = None
            deadline = time.monotonic() + float(self.goto_timeout_s)
            while True:
                location = self._location()
                if location is not None:
                    target_distance_m = gps_distance_m(self._gps_position(location), goto_target)
                    if target_distance_m <= float(self.goto_arrival_radius_m):
                        break
                if time.monotonic() > deadline:
                    raise MissionAbort("go-to arrival wait timed out")
                self._pump_once(deadline)
            print(
                f"[CORE] {self.client_id} arrived target_dist_m={target_distance_m} threshold_m={self.goto_arrival_radius_m}",
                flush=True,
            )

            request_id = self._send_command(
                occid.GoToCommand(
                    position=occid.GlobalPosition(
                        lat=float(goto_target.lat),
                        lon=float(goto_target.lon),
                        alt=float(self.target_altitude_m),
                        alt_frame=occid.AltitudeDatum.RELATIVE,
                    ),
                    yaw_rad=math.radians(float(self.goto_yaw_deg)),
                )
            )
            print(
                f"[CORE] {self.client_id} cmd GoToCommand alt_m={self.target_altitude_m} req={request_id}",
                flush=True,
            )
            self._require_ok(request_id, "go_to_altitude")

            self.wait_until(
                lambda: (
                    self._relative_altitude() is not None
                    and abs(float(self._relative_altitude()) - float(self.target_altitude_m))
                    <= float(self.altitude_tolerance_m)
                ),
                float(self.altitude_change_timeout_s),
                MissionAbort("altitude change wait timed out"),
            )
            print(
                f"[CORE] {self.client_id} altitude reached alt_m={self._relative_altitude()} "
                f"target_m={self.target_altitude_m} tol_m={self.altitude_tolerance_m}",
                flush=True,
            )

            request_id = self._send_command(occid.ReturnToLaunchCommand())
            print(f"[CORE] {self.client_id} cmd ReturnToLaunchCommand req={request_id}", flush=True)
            self._require_ok(request_id, "rtl")

            home_distance_m = None
            deadline = time.monotonic() + float(self.rtl_timeout_s)
            while True:
                location = self._location()
                if location is not None:
                    home_distance_m = gps_distance_m(self._gps_position(location), home_pos)
                    if home_distance_m <= float(self.home_arrival_radius_m):
                        break
                if time.monotonic() > deadline:
                    raise MissionAbort("rtl home wait timed out")
                self._pump_once(deadline)
            print(
                f"[CORE] {self.client_id} home reached dist_m={home_distance_m} threshold_m={self.home_arrival_radius_m}",
                flush=True,
            )

            request_id = self._send_command(occid.LandCommand())
            print(f"[CORE] {self.client_id} cmd LandCommand req={request_id}", flush=True)
            self._require_ok(request_id, "land")

            self.wait_until(
                lambda: (
                    self._flight_control() is not None
                    and not bool(self._flight_control().in_air)
                    and self._relative_altitude() is not None
                    and float(self._relative_altitude()) <= float(self.land_altitude_threshold_m)
                ),
                float(self.land_timeout_s),
                MissionAbort("landed wait timed out"),
            )
            print(
                f"[CORE] {self.client_id} landed in_air={self._flight_control().in_air} alt_m={self._relative_altitude()}",
                flush=True,
            )
            self.publish_shutdown()

        except MissionAbort as exc:
            flight = self._flight_control()
            if flight is not None and bool(flight.in_air):
                pass  # Airborne abort/recovery policy remains separate work.
            abort_topic = f"DIAG/{self.client_id}/ABORT"
            self.client.publish(
                abort_topic,
                build_envelope(self.client_id, abort_topic, {"event": "ABORT", "reason": str(exc)}),
            )
            print(f"[CORE] {self.client_id} mission_abort reason={exc}", flush=True)
            self.publish_shutdown()
        except RuntimeError:
            error_topic = f"DIAG/{self.client_id}/ERROR"
            self.client.publish(
                error_topic,
                build_envelope(
                    self.client_id,
                    error_topic,
                    {"event": "ERROR", "traceback": traceback.format_exc().strip()},
                ),
            )
            raise
        except KeyboardInterrupt:
            abort_topic = f"DIAG/{self.client_id}/ABORT"
            flight = self._flight_control()
            self.client.publish(
                abort_topic,
                build_envelope(self.client_id, abort_topic, {"event": "ABORT", "reason": "KeyboardInterrupt"}),
            )
            print(f"[CORE] {self.client_id} keyboard_interrupt in_air={None if flight is None else flight.in_air}", flush=True)
            raise
        finally:
            self.stop()


def run_core(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    TakeoffLandCore(cfg, bus_config).run()
