"""Convenience API for OCCID-native UAV programs.

This is an SDK facade, not a second semantic protocol. Every operation creates or
consumes OCCID models and sends them to the configured UAV controller plugin.
"""

from __future__ import annotations

from typing import Any, Iterable

from lib.common import build_request_topic, build_response_topic, build_state_topics, build_topic_base
from lib.occid_bus import get_occid_state, occid, send_occid_command
from lib.occid_topics import ANGULAR_VELOCITY, ATTITUDE, FLIGHT_CONTROL, LOCATION


class UavCommandError(RuntimeError):
    pass


class UavClient:
    """Thin program-facing UAV API backed entirely by OCCID commands/state."""

    DEFAULT_STATE_KEYS = (FLIGHT_CONTROL, LOCATION, ATTITUDE, ANGULAR_VELOCITY)

    def __init__(self, runtime: Any, interface: dict[str, Any], response_timeout_s: float) -> None:
        self.runtime = runtime
        self.interface_id = str(interface["id"])
        self.topic_ns = str(interface["topic_ns"])
        self.response_timeout_s = float(response_timeout_s)
        self.base_topic = build_topic_base(self.interface_id, self.topic_ns)
        self.request_topic = build_request_topic(self.interface_id, self.topic_ns)
        self.response_topic = build_response_topic(self.interface_id, self.topic_ns)

    def state_topics(self, keys: Iterable[str] | None = None) -> dict[str, str]:
        selected = list(self.DEFAULT_STATE_KEYS if keys is None else keys)
        return build_state_topics(self.base_topic, selected)

    def state(self, key: str, expected_type: type | tuple[type, ...] | None = None) -> Any:
        return get_occid_state(self.runtime.state, key, expected_type)

    def flight_control(self) -> Any | None:
        return self.state(FLIGHT_CONTROL, occid.FlightControlState)

    def location(self) -> Any | None:
        return self.state(LOCATION, occid.LocationState)

    def attitude(self) -> Any | None:
        return self.state(ATTITUDE, occid.EulerAngles)

    def angular_velocity(self) -> Any | None:
        return self.state(ANGULAR_VELOCITY, occid.AngularVelocityVector)

    def send(self, command: Any) -> str:
        return send_occid_command(self.runtime.bus, self.request_topic, command)

    def execute(self, command: Any, timeout_s: float | None = None) -> dict[str, Any]:
        request_id = self.send(command)
        response = self.runtime._wait_response(
            request_id,
            self.response_timeout_s if timeout_s is None else float(timeout_s),
        )
        if not response.get("ok"):
            raise UavCommandError(f"{type(command).__name__} failed response={response}")
        return response

    def arm(self) -> dict[str, Any]:
        return self.execute(occid.ArmCommand())

    def disarm(self) -> dict[str, Any]:
        return self.execute(occid.DisarmCommand())

    def set_takeoff_altitude(self, relative_altitude_m: float) -> dict[str, Any]:
        return self.execute(
            occid.SetTakeoffAltitudeCommand(relative_altitude_m=float(relative_altitude_m))
        )

    def takeoff(self) -> dict[str, Any]:
        return self.execute(occid.TakeoffCommand())

    def land(self) -> dict[str, Any]:
        return self.execute(occid.LandCommand())

    def return_to_launch(self) -> dict[str, Any]:
        return self.execute(occid.ReturnToLaunchCommand())

    def go_to(
        self,
        latitude_deg: float,
        longitude_deg: float,
        altitude_m: float,
        *,
        altitude_datum: Any = None,
        yaw_rad: float | None = None,
    ) -> dict[str, Any]:
        datum = occid.AltitudeDatum.RELATIVE if altitude_datum is None else altitude_datum
        return self.execute(
            occid.GoToCommand(
                position=occid.GlobalPosition(
                    lat=float(latitude_deg),
                    lon=float(longitude_deg),
                    alt=float(altitude_m),
                    alt_frame=datum,
                ),
                yaw_rad=None if yaw_rad is None else float(yaw_rad),
            )
        )

    def set_mode(
        self,
        *,
        standard_mode: Any | None = None,
        native_mode_name: str | None = None,
        native_mode_code: int | None = None,
    ) -> dict[str, Any]:
        return self.execute(
            occid.SetModeCommand(
                standard_mode=standard_mode,
                native_mode_name=native_mode_name,
                native_mode_code=native_mode_code,
            )
        )

    def start_offboard(self) -> dict[str, Any]:
        return self.execute(occid.StartOffboardCommand())

    def stop_offboard(self) -> dict[str, Any]:
        return self.execute(occid.StopOffboardCommand())

    def set_attitude(
        self,
        roll_rad: float,
        pitch_rad: float,
        yaw_rad: float,
        thrust_normalized: float,
        *,
        body_frame: Any = None,
        reference_frame: Any = None,
    ) -> dict[str, Any]:
        body = occid.BodyReferenceFrame.FRD if body_frame is None else body_frame
        reference = occid.InertialReferenceFrame.NED if reference_frame is None else reference_frame
        return self.execute(
            occid.SetControlAttitudeCommand(
                setpoint=occid.ControlAttitudeSetpoint(
                    roll_rad=float(roll_rad),
                    pitch_rad=float(pitch_rad),
                    yaw_rad=float(yaw_rad),
                    thrust_normalized=float(thrust_normalized),
                    body_frame=body,
                    reference_frame=reference,
                )
            )
        )

    def set_control_override(self, override: Any) -> dict[str, Any]:
        if not isinstance(override, occid.ControlOverride):
            raise TypeError(f"expected ControlOverride, got {type(override).__name__}")
        return self.execute(occid.SetControlOverrideCommand(override=override))
