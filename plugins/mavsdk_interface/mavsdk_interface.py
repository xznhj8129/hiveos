#!/usr/bin/env python3
"""MAVSDK endpoint adapter: OCCID <-> MAVSDK/PX4/ArduPilot."""

from __future__ import annotations

import asyncio
import logging
import queue
import threading
import time
import traceback
from typing import Any, Dict

from grpc import StatusCode
from grpc.aio import AioRpcError
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.info import InfoError
from mavsdk.offboard import Attitude, OffboardError

from interop.common import radians_to_degrees
from interop.mavsdk import (
    MavsdkPositionFields,
    angular_velocity_from_body_rates,
    attitude_from_euler_degrees,
    attitude_setpoint_to_fields,
    gnss_fix_type_from_native_value,
    position_to_location_state,
    standard_mode_from_native_name,
)
from lib.common import apply_cfg, build_request_topic, build_response_topic, build_state_scheduler_topics, build_topic_base
from lib.occid_bus import decode_occid_command, occid, pack_occid
from lib.occid_topics import (
    ANGULAR_VELOCITY,
    ATTITUDE,
    CONTROL_OUTPUT,
    CONTROL_OVERRIDE,
    FIRMWARE,
    FLIGHT_CONTROL,
    GNSS,
    IMU,
    LOCATION,
    POWER,
)
from lib.plugin_base import PluginBase
from lib.state_scheduler import StateScheduler

REQUEST_QUEUE_TIMEOUT_S = 0.05
POLL_INTERVAL_S = 0.1


class UnsupportedCommand(RuntimeError):
    pass


class MavsdkInterface(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)
        if self.conn_type != "udp":
            raise RuntimeError(f"unsupported conn_type {self.conn_type}")
        self.system_address = self.conn_str
        self.is_ardupilot = str(self.mav_dialect).upper() == "ARDUPILOT"
        if self.mavsdk_log_debug:
            logging.basicConfig(level=logging.DEBUG, force=True)
            logging.getLogger("mavsdk").setLevel(logging.DEBUG)
            logging.getLogger("mavsdk.system").setLevel(logging.DEBUG)
            logging.getLogger("mavsdk.async_plugin_manager").setLevel(logging.DEBUG)

        base = build_topic_base(self.client_id, self.topic_ns)
        self.request_topic = build_request_topic(self.client_id, self.topic_ns)
        self.response_topic = build_response_topic(self.client_id, self.topic_ns)
        self.client.subscribe(self.request_topic)
        self.init_bus(POLL_INTERVAL_S)
        self.state_scheduler = StateScheduler(
            self.client,
            self.client_id,
            build_state_scheduler_topics(base, self.state_intervals),
        )

        self.drone = System()
        self.request_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        self.stop_event = threading.Event()
        self.loop_error: BaseException | None = None
        self.loop_error_trace: str | None = None
        self.loop_thread: threading.Thread | None = None
        self.shutdown_requested = False

        self.last_abs_alt_m: float | None = None
        self.last_rel_alt_m: float | None = None
        self.last_attitude: Any | None = None
        self.location_state: Any | None = None
        self.gnss_state = occid.GnssSolution()
        self.nav_validity = occid.NavigationValidity()
        self.readiness = occid.NavReadinessState(mode_problems=[], health_problems=[])
        self.flight_control = occid.FlightControlState(
            native_active_mode_codes=[],
            native_active_mode_names=[],
            navigation_validity=self.nav_validity,
            readiness=self.readiness,
        )

        initial = dict(self.control_output_initial)
        self.control_output = occid.ControlAxisSet(
            roll=float(initial.get("roll", initial.get("Roll", 0.0))),
            pitch=float(initial.get("pitch", initial.get("Pitch", 0.0))),
            yaw=float(initial.get("yaw", initial.get("Yaw", 0.0))),
            throttle=float(initial.get("throttle", initial.get("Throttle", 0.0))),
            aux=[float(value) for value in initial.get("aux", initial.get("Aux", []))],
        )
        self.control_override: Any | None = None
        self.control_override_lock = threading.Lock()
        self.control_override_updated_at = 0.0
        self.manual_control_started = False
        self.offboard_attitude_started = False

    def _stream_enabled(self, key: str) -> bool:
        return key in self.state_scheduler.topics

    def _publish_model(self, key: str, model: Any) -> None:
        if not self._stream_enabled(key):
            return
        self.state_scheduler.update(key, pack_occid(model))

    def _publish_flight_control(self, **updates: Any) -> None:
        self.flight_control = self.flight_control.model_copy(update=updates)
        self._publish_model(FLIGHT_CONTROL, self.flight_control)

    def _publish_raw_readiness(self) -> None:
        self._publish_flight_control(readiness=self.readiness, navigation_validity=self.nav_validity)

    def _override_is_fresh(self) -> bool:
        if self.control_override is None:
            return False
        return time.monotonic() - self.control_override_updated_at <= float(self.control_override_timeout_s)

    def _merge_override(self, override: Any) -> Any:
        update: dict[str, Any] = {}
        for name in ("roll", "pitch", "yaw", "throttle"):
            value = getattr(override, name)
            if value is not None:
                update[name] = float(value)
        aux = list(self.control_output.aux)
        for channel in override.aux:
            index = int(channel.channel_index)
            if index < 0:
                raise ValueError(f"negative control channel index {index}")
            while len(aux) <= index:
                aux.append(0.0)
            if channel.value is not None:
                aux[index] = float(channel.value)
        update["aux"] = aux
        return self.control_output.model_copy(update=update)

    def _respond(self, request_id: str, command: Any, ok: bool, data: Dict[str, Any] | None = None, error: str | None = None) -> None:
        payload = {} if data is None else dict(data)
        if error is not None:
            payload["error"] = error
        self.enqueue_response(request_id, type(command).__name__, ok, payload)

    async def _process_requests(self) -> None:
        while not self.stop_event.is_set():
            try:
                request = await asyncio.to_thread(self.request_queue.get, timeout=REQUEST_QUEUE_TIMEOUT_S)
            except queue.Empty:
                await asyncio.sleep(POLL_INTERVAL_S)
                continue
            await self._handle_command(request)

    async def _handle_set_mode(self, command: Any) -> None:
        mode = command.standard_mode
        if mode == occid.StandardFlightMode.POSITION_HOLD:
            await self.drone.action.hold()
            return
        if mode == occid.StandardFlightMode.SAFE_RECOVERY:
            await self.drone.action.return_to_launch()
            return
        if mode == occid.StandardFlightMode.LAND:
            await self.drone.action.land()
            return
        if mode == occid.StandardFlightMode.TAKEOFF:
            await self.drone.action.takeoff()
            return
        raise UnsupportedCommand(
            f"MAVSDK adapter cannot set mode standard={mode} native_name={command.native_mode_name} "
            f"native_code={command.native_mode_code}"
        )

    async def _handle_command(self, request: Dict[str, Any]) -> None:
        request_id, command = decode_occid_command(request)
        try:
            if isinstance(command, occid.ArmCommand):
                await self.drone.action.arm()
            elif isinstance(command, occid.DisarmCommand):
                await self.drone.action.disarm()
            elif isinstance(command, occid.TakeoffCommand):
                await self.drone.action.takeoff()
            elif isinstance(command, occid.LandCommand):
                await self.drone.action.land()
            elif isinstance(command, occid.ReturnToLaunchCommand):
                await self.drone.action.return_to_launch()
            elif isinstance(command, occid.SetTakeoffAltitudeCommand):
                await self.drone.action.set_takeoff_altitude(float(command.relative_altitude_m))
            elif isinstance(command, occid.GoToCommand):
                position = command.position
                if position.alt_frame == occid.AltitudeDatum.SEA_LEVEL:
                    target_abs_alt_m = float(position.alt)
                elif position.alt_frame == occid.AltitudeDatum.RELATIVE:
                    if self.last_abs_alt_m is None or self.last_rel_alt_m is None:
                        raise UnsupportedCommand("relative GoTo requires current absolute and relative altitude")
                    target_abs_alt_m = float(self.last_abs_alt_m) + (float(position.alt) - float(self.last_rel_alt_m))
                else:
                    raise UnsupportedCommand(f"unsupported GoTo altitude datum {position.alt_frame}")
                if command.yaw_rad is not None:
                    yaw_deg = radians_to_degrees(float(command.yaw_rad), "yaw_rad")
                elif self.last_attitude is not None:
                    yaw_deg = radians_to_degrees(float(self.last_attitude.yaw_rad), "yaw_rad")
                else:
                    yaw_deg = 0.0
                await self.drone.action.goto_location(
                    float(position.lat), float(position.lon), target_abs_alt_m, yaw_deg
                )
            elif isinstance(command, occid.SetModeCommand):
                await self._handle_set_mode(command)
            elif isinstance(command, occid.SetControlAttitudeCommand):
                setpoint = command.setpoint
                fields = attitude_setpoint_to_fields(setpoint)
                await self.drone.offboard.set_attitude(
                    Attitude(
                        fields.roll_deg,
                        fields.pitch_deg,
                        fields.yaw_deg,
                        fields.thrust_value,
                    )
                )
                self._publish_flight_control(attitude_setpoint=setpoint)
                print(
                    f"[PLUGIN] {self.client_id} offboard_set_attitude "
                    f"roll_rad={setpoint.roll_rad} pitch_rad={setpoint.pitch_rad} "
                    f"yaw_rad={setpoint.yaw_rad} thrust={setpoint.thrust_normalized}",
                    flush=True,
                )
            elif isinstance(command, occid.StartOffboardCommand):
                await self.drone.offboard.start()
                self.offboard_attitude_started = True
                print(f"[PLUGIN] {self.client_id} offboard_start=True", flush=True)
            elif isinstance(command, occid.StopOffboardCommand):
                await self.drone.offboard.stop()
                self.offboard_attitude_started = False
                print(f"[PLUGIN] {self.client_id} offboard_stop=True", flush=True)
            elif isinstance(command, occid.SetControlOverrideCommand):
                if not self.consume_control_override:
                    raise UnsupportedCommand("control override disabled")
                with self.control_override_lock:
                    self.control_override = command.override
                    self.control_override_updated_at = time.monotonic()
                    self.control_output = self._merge_override(command.override)
                    control_output = self.control_output
                self._publish_model(CONTROL_OVERRIDE, command.override)
                self._publish_model(CONTROL_OUTPUT, control_output)
                self._publish_flight_control(override_active=True)
            elif isinstance(command, occid.SelectMissionCommand):
                raise UnsupportedCommand("mission selection is not implemented by the MAVSDK adapter")
            else:
                raise UnsupportedCommand(f"unsupported OCCID command {type(command).__name__}")
            self._respond(request_id, command, True)
        except (ActionError, OffboardError, UnsupportedCommand, ValueError, TypeError) as exc:
            self._respond(request_id, command, False, error=str(exc))

    async def _watch_in_air(self) -> None:
        async for in_air in self.drone.telemetry.in_air():
            self._publish_flight_control(in_air=bool(in_air))
            if self.stop_event.is_set():
                return

    async def _watch_armed(self) -> None:
        async for armed in self.drone.telemetry.armed():
            self._publish_flight_control(armed=bool(armed))
            if self.stop_event.is_set():
                return

    async def _watch_health(self) -> None:
        async for health in self.drone.telemetry.health():
            self.nav_validity = self.nav_validity.model_copy(
                update={
                    "local_position_ok": bool(health.is_local_position_ok),
                    "global_position_ok": bool(health.is_global_position_ok),
                    "home_position_ok": bool(health.is_home_position_ok),
                }
            )
            self.readiness = self.readiness.model_copy(
                update={
                    "gyro_ok": bool(health.is_gyrometer_calibration_ok),
                    "accel_ok": bool(health.is_accelerometer_calibration_ok),
                    "mag_ok": bool(health.is_magnetometer_calibration_ok),
                    "local_position_ok": bool(health.is_local_position_ok),
                    "global_position_ok": bool(health.is_global_position_ok),
                    "home_position_ok": bool(health.is_home_position_ok),
                    "armable": bool(health.is_armable),
                    "can_arm_or_run": bool(health.is_armable),
                }
            )
            self._publish_raw_readiness()
            if self.stop_event.is_set():
                return

    async def _watch_status_text(self) -> None:
        async for status_text in self.drone.telemetry.status_text():
            text = status_text.text
            if self.is_ardupilot and " is using GPS" in text:
                if not self.readiness.ekf_using_gps:
                    print(f"[PLUGIN] {self.client_id} ardupilot_status_text text={text}", flush=True)
                self.readiness = self.readiness.model_copy(update={"ekf_using_gps": True})
                self._publish_raw_readiness()
            if self.stop_event.is_set():
                return

    async def _watch_position(self) -> None:
        async for position in self.drone.telemetry.position():
            self.last_abs_alt_m = float(position.absolute_altitude_m)
            self.last_rel_alt_m = float(position.relative_altitude_m)
            self.location_state = position_to_location_state(
                MavsdkPositionFields(
                    latitude_deg=float(position.latitude_deg),
                    longitude_deg=float(position.longitude_deg),
                    absolute_altitude_m=self.last_abs_alt_m,
                    relative_altitude_m=self.last_rel_alt_m,
                ),
                navigation_validity=self.nav_validity,
            )
            self._publish_model(LOCATION, self.location_state)
            if self.stop_event.is_set():
                return

    async def _watch_attitude(self) -> None:
        async for attitude in self.drone.telemetry.attitude_euler():
            self.last_attitude = attitude_from_euler_degrees(
                float(attitude.roll_deg),
                float(attitude.pitch_deg),
                float(attitude.yaw_deg),
            )
            self._publish_model(ATTITUDE, self.last_attitude)
            if self.stop_event.is_set():
                return

    async def _watch_angular_velocity(self) -> None:
        async for velocity in self.drone.telemetry.attitude_angular_velocity_body():
            state = angular_velocity_from_body_rates(
                float(velocity.roll_rad_s),
                float(velocity.pitch_rad_s),
                float(velocity.yaw_rad_s),
            )
            self._publish_model(ANGULAR_VELOCITY, state)
            if self.stop_event.is_set():
                return

    async def _watch_gps_info(self) -> None:
        async for gps_info in self.drone.telemetry.gps_info():
            native_fix = gps_info.fix_type.value if hasattr(gps_info.fix_type, "value") else gps_info.fix_type
            self.gnss_state = self.gnss_state.model_copy(
                update={
                    "fix_type": gnss_fix_type_from_native_value(int(native_fix)),
                    "fix_code": int(native_fix),
                    "satellites_used": int(gps_info.num_satellites),
                }
            )
            self._publish_model(GNSS, self.gnss_state)
            if self.stop_event.is_set():
                return

    async def _watch_raw_gps(self) -> None:
        async for raw_gps in self.drone.telemetry.raw_gps():
            self.gnss_state = self.gnss_state.model_copy(
                update={
                    "position": occid.GlobalPosition(
                        lat=float(raw_gps.latitude_deg),
                        lon=float(raw_gps.longitude_deg),
                        alt=float(raw_gps.absolute_altitude_m),
                        alt_frame=occid.AltitudeDatum.SEA_LEVEL,
                    ),
                    "hdop": float(raw_gps.hdop),
                    "vdop": float(raw_gps.vdop),
                    "ground_speed_ms": float(raw_gps.velocity_m_s),
                    "ground_course_deg": float(raw_gps.cog_deg),
                    "yaw_deg": float(raw_gps.yaw_deg),
                }
            )
            self._publish_model(GNSS, self.gnss_state)
            if self.stop_event.is_set():
                return

    async def _watch_battery(self) -> None:
        async for battery in self.drone.telemetry.battery():
            remaining = battery.remaining_percent
            remaining_pct = None if remaining is None else float(remaining) * 100.0
            state = occid.ElectricalResourceState(
                battery_id=int(battery.id),
                voltage_v=None if battery.voltage_v is None else float(battery.voltage_v),
                current_a=None if battery.current_battery_a is None else float(battery.current_battery_a),
                consumed_ah=None if battery.capacity_consumed_ah is None else float(battery.capacity_consumed_ah),
                remaining_pct=remaining_pct,
                temperature_deg_c=None if battery.temperature_degc is None else float(battery.temperature_degc),
            )
            self._publish_model(POWER, state)
            if self.stop_event.is_set():
                return

    async def _watch_flight_mode(self) -> None:
        async for flight_mode in self.drone.telemetry.flight_mode():
            mode_name = flight_mode.name if hasattr(flight_mode, "name") else str(flight_mode)
            self.readiness = self.readiness.model_copy(update={"mode_name": mode_name})
            self._publish_flight_control(
                standard_mode=standard_mode_from_native_name(mode_name),
                native_mode_name=mode_name,
                native_active_mode_names=[mode_name],
                readiness=self.readiness,
            )
            if self.stop_event.is_set():
                return

    async def _watch_imu(self) -> None:
        async for imu in self.drone.telemetry.imu():
            # Acceleration/magnetic vectors are intentionally omitted until OCCID has
            # a body-frame linear-vector type. Do not mislabel FRD data as inertial.
            state = occid.ImuSample(
                angular_velocity=angular_velocity_from_body_rates(
                    float(imu.angular_velocity_frd.forward_rad_s),
                    float(imu.angular_velocity_frd.right_rad_s),
                    float(imu.angular_velocity_frd.down_rad_s),
                ),
                temperature_deg_c=float(imu.temperature_degc),
                timestamp_us=int(imu.timestamp_us),
                frame=occid.BodyReferenceFrame.FRD,
            )
            self._publish_model(IMU, state)
            if self.stop_event.is_set():
                return

    async def _manual_control_loop(self) -> None:
        while not self.stop_event.is_set():
            if not self._override_is_fresh():
                if self.manual_control_started:
                    self._publish_flight_control(override_active=False)
                self.manual_control_started = False
                await asyncio.sleep(float(self.control_override_send_interval_s))
                continue
            with self.control_override_lock:
                output = self.control_output
                override = self.control_override
            await self.drone.manual_control.set_manual_control_input(
                float(output.pitch or 0.0),
                float(output.roll or 0.0),
                float(output.throttle or 0.0),
                float(output.yaw or 0.0),
            )
            if not self.manual_control_started:
                await self.drone.manual_control.start_altitude_control()
                self.manual_control_started = True
            if override is not None:
                self._publish_model(CONTROL_OVERRIDE, override)
            self._publish_model(CONTROL_OUTPUT, output)
            await asyncio.sleep(float(self.control_override_send_interval_s))

    async def _publish_fc_info(self) -> None:
        if not self._stream_enabled(FIRMWARE):
            return
        for _ in range(25):
            if self.stop_event.is_set():
                return
            try:
                product = await self.drone.info.get_product()
                version = await self.drone.info.get_version()
                state = occid.FirmwareInfo(
                    name=str(product.product_name or product.vendor_name or self.mav_dialect),
                    version=occid.Version(
                        major=int(version.flight_sw_major),
                        minor=int(version.flight_sw_minor),
                        patch=int(version.flight_sw_patch),
                    ),
                    build=str(version.flight_sw_git_hash),
                )
                self._publish_model(FIRMWARE, state)
                return
            except InfoError:
                await asyncio.sleep(0.2)

    async def _async_main(self) -> None:
        print(f"[PLUGIN] {self.client_id} connecting type={self.conn_type} address={self.system_address}", flush=True)
        await self.drone.connect(system_address=self.system_address)
        print(f"[PLUGIN] {self.client_id} connected type={self.conn_type} address={self.system_address}", flush=True)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                break
            if self.stop_event.is_set():
                return

        await self._publish_fc_info()
        self.send_online()
        self._publish_flight_control()
        if self.consume_control_override:
            self._publish_model(CONTROL_OUTPUT, self.control_output)

        tasks = [asyncio.create_task(self._process_requests())]
        if self._stream_enabled(FLIGHT_CONTROL):
            tasks.extend(
                [
                    asyncio.create_task(self._watch_in_air()),
                    asyncio.create_task(self._watch_armed()),
                    asyncio.create_task(self._watch_health()),
                    asyncio.create_task(self._watch_status_text()),
                    asyncio.create_task(self._watch_flight_mode()),
                ]
            )
        if self._stream_enabled(LOCATION):
            tasks.append(asyncio.create_task(self._watch_position()))
        if self._stream_enabled(ATTITUDE):
            tasks.append(asyncio.create_task(self._watch_attitude()))
        if self._stream_enabled(ANGULAR_VELOCITY):
            tasks.append(asyncio.create_task(self._watch_angular_velocity()))
        if self._stream_enabled(GNSS):
            tasks.extend([asyncio.create_task(self._watch_gps_info()), asyncio.create_task(self._watch_raw_gps())])
        if self._stream_enabled(POWER):
            tasks.append(asyncio.create_task(self._watch_battery()))
        if self._stream_enabled(IMU):
            tasks.append(asyncio.create_task(self._watch_imu()))
        if self.consume_control_override:
            tasks.append(asyncio.create_task(self._manual_control_loop()))

        try:
            while not self.stop_event.is_set():
                for task in tasks:
                    if task.done():
                        exc = task.exception()
                        if exc:
                            if self.stop_event.is_set():
                                return
                            raise exc
                await asyncio.sleep(POLL_INTERVAL_S)
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

    def _loop_runner(self) -> None:
        try:
            asyncio.run(self._async_main())
        except BaseException as exc:
            self.loop_error = exc
            self.loop_error_trace = traceback.format_exc().strip()

    def run(self) -> None:
        if self.loop_thread is None:
            self.stop_event.clear()
            self.loop_thread = threading.Thread(target=self._loop_runner, name="mavsdk-loop", daemon=True)
            self.loop_thread.start()

        try:
            while True:
                self.state_scheduler.flush()
                self.flush_queue(self.response_queue, self.response_topic)
                if self.loop_error:
                    raise self.loop_error
                try:
                    topic, payload = self._pump_once()
                except SystemExit:
                    self.shutdown_requested = True
                    self.stop_event.set()
                    break
                if topic == self.request_topic:
                    self.request_queue.put(payload["data"])
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_event.set()
            if self.loop_thread:
                self.loop_thread.join(timeout=5.0)
                self.loop_thread = None
            self.flush_queue(self.response_queue, self.response_topic)
            if self.loop_error:
                if self.shutdown_requested and isinstance(self.loop_error, AioRpcError):
                    if self.loop_error.code() == StatusCode.UNAVAILABLE:
                        self.stop()
                        self.drone._stop_mavsdk_server()
                        return
                trace = self.loop_error_trace or traceback.format_exception_only(
                    type(self.loop_error), self.loop_error
                )[-1].strip()
                self.publish_error(trace)
                raise self.loop_error
            self.stop()
            self.drone._stop_mavsdk_server()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    MavsdkInterface(cfg, bus_config).run()
