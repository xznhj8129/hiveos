#!/usr/bin/env python3
"""MSP endpoint adapter: OCCID <-> INAV/Betaflight MSP."""

from __future__ import annotations

import math
import queue
import threading
import time
import traceback
from typing import Any, Dict

from mspapi2.lib import InavEnums, boxes
from mspapi2.msp_api import MSPApi
from mspapi2.msp_serial import MSPSerial

from lib.common import apply_cfg, build_request_topic, build_response_topic, build_state_scheduler_topics, build_topic_base
from lib.occid_bus import decode_occid_command, occid, pack_occid
from lib.occid_topics import (
    ANGULAR_VELOCITY,
    ATTITUDE,
    CONTROL_OUTPUT,
    CONTROL_OVERRIDE,
    FLIGHT_CONTROL,
    GNSS,
    IMU,
    LOCATION,
    POWER,
    RC_TELEMETRY,
    RUNTIME_LOAD,
)
from lib.plugin_base import PluginBase
from lib.reference_frames import fru_to_frd_vector
from lib.state_scheduler import StateScheduler
from lib.uav import scale_float_pwm, scale_pwm_float

REQUEST_QUEUE_TIMEOUT_S = 0.05
POLL_INTERVAL_S = 0.1


class UnsupportedCommand(RuntimeError):
    pass


def _standard_mode(native_names: list[str]) -> Any:
    names = {name.upper().replace("_", " ") for name in native_names}
    if any(name in names for name in {"NAV POSHOLD", "POSHOLD", "LOITER"}):
        return occid.StandardFlightMode.POSITION_HOLD
    if any(name in names for name in {"RTH", "NAV RTH"}):
        return occid.StandardFlightMode.SAFE_RECOVERY
    if any(name in names for name in {"NAV WP", "MISSION"}):
        return occid.StandardFlightMode.MISSION
    if any(name in names for name in {"NAV LAND", "LAND"}):
        return occid.StandardFlightMode.LAND
    if any(name in names for name in {"NAV CRUISE", "CRUISE"}):
        return occid.StandardFlightMode.CRUISE
    if any(name in names for name in {"ALT HOLD", "ALTHOLD"}):
        return occid.StandardFlightMode.ALTITUDE_HOLD
    return occid.StandardFlightMode.NON_STANDARD


def _gnss_fix_type(fix: Any) -> Any:
    name = getattr(fix, "name", str(fix)).upper()
    if "RTK_FIXED" in name:
        return occid.GnssFixType.RTK_FIXED
    if "RTK_FLOAT" in name:
        return occid.GnssFixType.RTK_FLOAT
    if "DGPS" in name:
        return occid.GnssFixType.DGPS
    if "3D" in name:
        return occid.GnssFixType.FIX_3D
    if "2D" in name:
        return occid.GnssFixType.FIX_2D
    if "NO_FIX" in name or "NONE" in name:
        return occid.GnssFixType.NO_FIX
    return occid.GnssFixType.NONE


class MspInterface(PluginBase):
    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)
        if self.conn_type not in {"serial", "tcp"}:
            raise RuntimeError(f"unsupported conn_type {self.conn_type}")

        base = build_topic_base(self.client_id, self.topic_ns)
        self.request_topic = build_request_topic(self.client_id, self.topic_ns)
        self.response_topic = build_response_topic(self.client_id, self.topic_ns)
        self.client.subscribe(self.request_topic)
        self.init_bus(POLL_INTERVAL_S)
        self.state_scheduler = StateScheduler(
            self.client,
            self.client_id,
            build_state_scheduler_topics(base, cfg["state_intervals"]),
        )

        self.serial_transport = MSPSerial(
            self.conn_str,
            self.conn_bitrate,
            read_timeout=float(self.conn_read_timeout_s),
            write_timeout=float(self.conn_write_timeout_s),
            tcp=self.conn_type == "tcp",
            max_retries=int(self.conn_max_retries),
            reconnect_delay=float(self.conn_reconnect_delay_s),
        )
        self.api = MSPApi(port=self.conn_str, baudrate=self.conn_bitrate, serial_transport=self.serial_transport)
        self.api_lock = threading.Lock()
        with self.api_lock:
            self.api.open()
            self.api_version = self.api.get_api_version()
            self.fc_variant = self.api.get_fc_variant()
            self.board_info = self.api.get_board_info()
            self.sensor_config = self.api.get_sensor_config()
            self.rx_config = self.api.get_rx_config()
            self.rx_map = self.api.get_rx_map()
            mode_ranges = self.api.get_mode_ranges()
        print(
            f"[PLUGIN_CONN] id={self.client_id} type={self.conn_type} conn={self.conn_str} baud={self.conn_bitrate}",
            flush=True,
        )

        self.mode_channels: Dict[str, Dict[str, Any]] = {}
        for entry in mode_ranges:
            aux_index = entry["auxChannelIndex"]
            channel_index = aux_index + 4
            channel_name = self.api.chmap[channel_index] if channel_index < len(self.api.chmap) else f"ch{channel_index + 1}"
            pwm_start, pwm_end = entry["pwmRange"]
            self.mode_channels[entry["mode"]] = {"channel": channel_name, "pwm": int((pwm_start + pwm_end) / 2)}

        self.arm_mode_name = "ARM"
        self.override_mode_name = "MSP RC OVERRIDE"
        self.takeoff_altitude_m: float | None = None
        self.control_override: Any | None = None
        self.control_override_lock = threading.Lock()
        self.control_override_updated_at = 0.0
        self.latest_flight_control: Any | None = None
        self.latest_location: Any | None = None
        self.latest_rc: Any | None = None

        self.request_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        self.stop_event = threading.Event()
        self.loop_error: BaseException | None = None
        self.loop_error_trace: str | None = None
        self.loop_error_source: str | None = None
        self.loop_error_published = False
        self.worker_threads: Dict[str, threading.Thread] = {}
        self.shutdown_requested = False

        self._activate_override()
        self._refresh_state()

    def _capture_loop_error(self, source: str, exc: BaseException) -> None:
        if self.loop_error is not None:
            self.stop_event.set()
            return
        self.loop_error = exc
        self.loop_error_source = source
        self.loop_error_trace = traceback.format_exc().strip()
        print(
            f"[PLUGIN_ERROR] id={self.client_id} source={source} conn_type={self.conn_type} "
            f"conn={self.conn_str} reconnects={self.serial_transport.reconnects} serial_diag={self.serial_transport.last_diag}",
            flush=True,
        )
        print(self.loop_error_trace, flush=True)
        try:
            self.publish_error(self.loop_error_trace)
            self.loop_error_published = True
        except Exception as publish_error:
            print(f"[PLUGIN_ERROR] id={self.client_id} publish_error={publish_error}", flush=True)
        self.stop_event.set()

    def _stream_enabled(self, key: str) -> bool:
        return key in self.state_scheduler.topics

    def _publish_model(self, key: str, model: Any) -> None:
        if self._stream_enabled(key):
            self.state_scheduler.update(key, pack_occid(model))

    def _respond(self, request_id: str, command: Any, ok: bool, data: Dict[str, Any] | None = None, error: str | None = None) -> None:
        payload = {} if data is None else dict(data)
        if error is not None:
            payload["error"] = error
        self.enqueue_response(request_id, type(command).__name__, ok, payload)

    def _activate_override(self) -> None:
        if self.override_mode_name in self.mode_channels:
            channel = self.mode_channels[self.override_mode_name]["channel"]
            pwm = self.mode_channels[self.override_mode_name]["pwm"]
            with self.api_lock:
                self.api.set_rc_channels({channel: pwm})

    def _apply_mode(self, mode_name: str) -> None:
        if mode_name not in self.mode_channels:
            raise UnsupportedCommand(f"mode {mode_name} not configured")
        channel = self.mode_channels[mode_name]["channel"]
        pwm = self.mode_channels[mode_name]["pwm"]
        with self.api_lock:
            self.api.set_rc_channels({channel: pwm})

    def _clear_mode(self, mode_name: str) -> None:
        if mode_name not in self.mode_channels:
            raise UnsupportedCommand(f"mode {mode_name} not configured")
        channel = self.mode_channels[mode_name]["channel"]
        with self.api_lock:
            self.api.set_rc_channels({channel: self.rx_config["rxMinUsec"]})

    def _find_mode(self, candidates: tuple[str, ...]) -> str:
        for candidate in candidates:
            if candidate in self.mode_channels:
                return candidate
        raise UnsupportedCommand(f"none of native modes configured candidates={candidates}")

    def _set_standard_mode(self, mode: Any) -> None:
        if mode == occid.StandardFlightMode.POSITION_HOLD:
            self._apply_mode(self._find_mode(("NAV POSHOLD", "POSHOLD")))
            return
        if mode == occid.StandardFlightMode.SAFE_RECOVERY:
            self._apply_mode(self._find_mode(("RTH", "NAV RTH", "NAV_RTH")))
            return
        if mode == occid.StandardFlightMode.MISSION:
            self._apply_mode(self._find_mode(("NAV WP", "NAV_WP")))
            return
        if mode == occid.StandardFlightMode.LAND:
            self._apply_mode(self._find_mode(("NAV LAND", "LAND")))
            return
        raise UnsupportedCommand(f"MSP adapter does not map standard mode {mode}")

    def _set_throttle(self, value: int) -> None:
        with self.api_lock:
            self.api.set_rc_channels({"throttle": value})

    def _rc_telemetry(self, rc_channels: Any) -> Any:
        if type(rc_channels) is dict:
            aux: list[float] = []
            primary_names = set(self.override_channels.values())
            for channel_name in self.api.chmap[4:]:
                if channel_name in primary_names or channel_name not in rc_channels:
                    continue
                aux.append(scale_pwm_float(rc_channels[channel_name], self.rx_config["rxMinUsec"], self.rx_config["rxMaxUsec"]))
            return occid.ControlAxisSet(
                roll=scale_pwm_float(rc_channels[self.override_channels["roll"]], self.rx_config["rxMinUsec"], self.rx_config["rxMaxUsec"]),
                pitch=scale_pwm_float(rc_channels[self.override_channels["pitch"]], self.rx_config["rxMinUsec"], self.rx_config["rxMaxUsec"]),
                yaw=scale_pwm_float(rc_channels[self.override_channels["yaw"]], self.rx_config["rxMinUsec"], self.rx_config["rxMaxUsec"]),
                throttle=scale_pwm_float(rc_channels[self.override_channels["throt"]], self.rx_config["rxMinUsec"], self.rx_config["rxMaxUsec"]),
                aux=aux,
            )
        if type(rc_channels) is list and len(rc_channels) >= 4:
            return occid.ControlAxisSet(
                roll=float(rc_channels[0]),
                pitch=float(rc_channels[1]),
                yaw=float(rc_channels[3]),
                throttle=float(rc_channels[2]),
                aux=[float(value) for value in rc_channels[4:]],
            )
        raise RuntimeError(f"unsupported rc_channels type {type(rc_channels).__name__}")

    def _override_is_fresh(self) -> bool:
        return self.control_override is not None and time.monotonic() - self.control_override_updated_at <= float(self.control_override_timeout_s)

    def _merge_override(self, base: Any, override: Any) -> Any:
        update: dict[str, Any] = {}
        for name in ("roll", "pitch", "yaw", "throttle"):
            value = getattr(override, name)
            if value is not None:
                update[name] = float(value)
        aux = list(base.aux)
        for channel in override.aux:
            index = int(channel.channel_index)
            while len(aux) <= index:
                aux.append(0.0)
            if channel.value is not None:
                aux[index] = float(channel.value)
        update["aux"] = aux
        return base.model_copy(update=update)

    def _build_override_channels(self) -> Dict[str, int]:
        with self.control_override_lock:
            override = self.control_override
        if override is None:
            return {}
        channels: Dict[str, int] = {}
        primary = {
            "roll": self.override_channels["roll"],
            "pitch": self.override_channels["pitch"],
            "yaw": self.override_channels["yaw"],
            "throttle": self.override_channels["throt"],
        }
        for field, channel_name in primary.items():
            value = getattr(override, field)
            if value is not None:
                channels[channel_name] = scale_float_pwm(float(value), self.rx_config["rxMinUsec"], self.rx_config["rxMaxUsec"])
        for aux in override.aux:
            index = 4 + int(aux.channel_index)
            if aux.value is None or index >= len(self.api.chmap):
                continue
            channels[self.api.chmap[index]] = scale_float_pwm(float(aux.value), self.rx_config["rxMinUsec"], self.rx_config["rxMaxUsec"])
        return channels

    def _refresh_state(self) -> None:
        with self.api_lock:
            status = self.api.get_inav_status()
            analog = self.api.get_inav_analog()
            alt = self.api.get_altitude()
            gps = self.api.get_raw_gps()
            nav_status = self.api.get_nav_status()
            attitude = self.api.get_attitude()
            imu = self.api.get_imu()
            rc_channels = self.api.get_rc_channels()

        is_armed = InavEnums.armingFlag_e.ARMED in status["armingFlags"]
        relative_alt_m = alt["estimatedAltitude"]
        is_in_air = is_armed and relative_alt_m is not None and relative_alt_m >= float(self.in_air_alt_threshold)
        global_ok = gps["fixType"] == InavEnums.gpsFixType_e.GPS_FIX_3D and gps["numSat"] >= int(self.home_min_satellites)
        active_modes = status["activeModes"]
        active_mode_names = [mode.name for mode in active_modes]
        active_mode_ids = [int(mode.value) for mode in active_modes]
        override_active = boxes.BoxEnum.BOXMSPRCOVERRIDE in active_modes
        failsafe = boxes.BoxEnum.BOXFAILSAFE in active_modes

        nav_validity = occid.NavigationValidity(
            local_position_ok=relative_alt_m is not None,
            global_position_ok=bool(global_ok),
            home_position_ok=bool(global_ok),
        )
        readiness = occid.NavReadinessState(
            local_position_ok=nav_validity.local_position_ok,
            global_position_ok=nav_validity.global_position_ok,
            home_position_ok=nav_validity.home_position_ok,
            armable=not failsafe,
            can_arm_or_run=not failsafe,
            mode_name=active_mode_names[0] if active_mode_names else None,
            mode_problems=[],
            health_problems=[],
        )
        runtime_load = occid.RuntimeLoadState(
            cpu_load=None if status.get("cpuLoad") is None else int(status["cpuLoad"]),
            cycle_time_us=None if status.get("cycleTime") is None else int(status["cycleTime"]),
        )
        flight_control = occid.FlightControlState(
            armed=is_armed,
            in_air=is_in_air,
            override_active=override_active,
            failsafe=failsafe,
            standard_mode=_standard_mode(active_mode_names),
            native_mode_name=active_mode_names[0] if active_mode_names else None,
            native_active_mode_codes=active_mode_ids,
            native_active_mode_names=active_mode_names,
            native_nav_state_code=None if nav_status.get("navState") is None else int(nav_status["navState"]),
            navigation_validity=nav_validity,
            readiness=readiness,
            runtime_load=runtime_load,
        )
        self.latest_flight_control = flight_control
        self._publish_model(FLIGHT_CONTROL, flight_control)
        self._publish_model(RUNTIME_LOAD, runtime_load)

        absolute_alt_m = gps.get("altitude")
        location = occid.LocationState(
            inertial_frame=occid.InertialReferenceFrame.NED,
            body_frame=occid.BodyReferenceFrame.FRD,
            position=occid.GlobalPosition(
                lat=float(gps["latitude"]),
                lon=float(gps["longitude"]),
                alt=float(absolute_alt_m or 0.0),
                alt_frame=occid.AltitudeDatum.SEA_LEVEL,
            ),
            altitude=occid.AltitudeState(
                absolute_m=None if absolute_alt_m is None else float(absolute_alt_m),
                relative_m=None if relative_alt_m is None else float(relative_alt_m),
                datum=occid.AltitudeDatum.RELATIVE,
            ),
            navigation_validity=nav_validity,
        )
        self.latest_location = location
        self._publish_model(LOCATION, location)

        attitude_state = occid.EulerAngles(
            roll_rad=math.radians(float(attitude["roll"])),
            pitch_rad=math.radians(float(attitude["pitch"])),
            yaw_rad=math.radians(float(attitude["yaw"])),
            body_frame=occid.BodyReferenceFrame.FRD,
            reference_frame=occid.InertialReferenceFrame.NED,
        )
        self._publish_model(ATTITUDE, attitude_state)

        gyro = imu["gyro"]
        gyro_x, gyro_y, gyro_z = fru_to_frd_vector(
            math.radians(float(gyro["X"])),
            math.radians(float(gyro["Y"])),
            math.radians(float(gyro["Z"])),
        )
        angular_velocity = occid.AngularVelocityVector(
            x_rad_s=gyro_x,
            y_rad_s=gyro_y,
            z_rad_s=gyro_z,
            frame=occid.BodyReferenceFrame.FRD,
        )
        self._publish_model(ANGULAR_VELOCITY, angular_velocity)
        self._publish_model(
            IMU,
            occid.ImuSample(angular_velocity=angular_velocity, frame=occid.BodyReferenceFrame.FRD),
        )

        native_fix = gps["fixType"]
        fix_code = getattr(native_fix, "value", native_fix)
        gnss = occid.GnssSolution(
            fix_type=_gnss_fix_type(native_fix),
            fix_code=int(fix_code),
            satellites_used=int(gps["numSat"]),
            position=occid.GlobalPosition(
                lat=float(gps["latitude"]),
                lon=float(gps["longitude"]),
                alt=float(absolute_alt_m or 0.0),
                alt_frame=occid.AltitudeDatum.SEA_LEVEL,
            ),
            altitude=location.altitude,
            ground_speed_ms=None if gps.get("speed") is None else float(gps["speed"]),
            ground_course_deg=None if gps.get("groundCourse") is None else float(gps["groundCourse"]),
            hdop=None if gps.get("hdop") is None else float(gps["hdop"]),
        )
        self._publish_model(GNSS, gnss)

        power = occid.ElectricalResourceState(
            voltage_v=analog.get("vbat"),
            current_a=analog.get("amperage"),
            power_w=analog.get("powerDraw"),
            consumed_mah=analog.get("mAhDrawn"),
            consumed_mwh=analog.get("mWhDrawn"),
            remaining_pct=analog.get("percentageRemaining"),
            remaining_capacity=analog.get("remainingCapacity"),
            rssi=analog.get("rssi"),
        )
        self._publish_model(POWER, power)

        rc = self._rc_telemetry(rc_channels)
        self.latest_rc = rc
        self._publish_model(RC_TELEMETRY, rc)
        with self.control_override_lock:
            override = self.control_override
        if override is not None:
            self._publish_model(CONTROL_OVERRIDE, override)
        output = self._merge_override(rc, override) if override is not None and self._override_is_fresh() else rc
        self._publish_model(CONTROL_OUTPUT, output)

    def _state_loop(self) -> None:
        try:
            while not self.stop_event.is_set():
                self._refresh_state()
                time.sleep(float(self.state_poll_interval_s))
        except BaseException as exc:
            self._capture_loop_error("state_loop", exc)

    def _override_loop(self) -> None:
        try:
            last_send = 0.0
            while not self.stop_event.is_set():
                elapsed = time.monotonic() - last_send
                if elapsed < float(self.override_send_interval):
                    time.sleep(float(self.override_send_interval) - elapsed)
                    continue
                last_send = time.monotonic()
                if not self._override_is_fresh():
                    continue
                if self.latest_flight_control is None or not bool(self.latest_flight_control.override_active):
                    continue
                channels = self._build_override_channels()
                if channels:
                    with self.api_lock:
                        self.api.set_rc_channels(channels)
        except BaseException as exc:
            self._capture_loop_error("override_loop", exc)

    def _set_goto(self, command: Any) -> None:
        position = command.position
        if position.alt_frame != occid.AltitudeDatum.RELATIVE:
            raise UnsupportedCommand(f"INAV GoTo currently requires RELATIVE altitude actual={position.alt_frame}")
        waypoint_index = int(self.go_to_waypoint["WaypointIndex"])
        action_enum = InavEnums.navWaypointActions_e(int(self.go_to_waypoint["Action"]))
        with self.api_lock:
            self.api.set_waypoint(
                waypointIndex=waypoint_index,
                action=action_enum,
                latitude=float(position.lat),
                longitude=float(position.lon),
                altitude=float(position.alt),
                param1=int(self.go_to_waypoint["Param1"]),
                param2=int(self.go_to_waypoint["Param2"]),
                param3=int(self.go_to_waypoint["Param3"]),
                flag=int(self.go_to_waypoint["Flag"]),
            )

    def _handle_command(self, request: Dict[str, Any]) -> None:
        request_id, command = decode_occid_command(request)
        try:
            if isinstance(command, occid.SetTakeoffAltitudeCommand):
                self.takeoff_altitude_m = float(command.relative_altitude_m)
            elif isinstance(command, occid.SetControlOverrideCommand):
                with self.control_override_lock:
                    self.control_override = command.override
                    self.control_override_updated_at = time.monotonic()
            elif isinstance(command, occid.ReturnToLaunchCommand):
                self._apply_mode(self._find_mode(("RTH", "NAV RTH", "NAV_RTH")))
            elif isinstance(command, occid.SetModeCommand):
                if command.native_mode_name is not None:
                    self._apply_mode(str(command.native_mode_name))
                elif command.standard_mode is not None:
                    self._set_standard_mode(command.standard_mode)
                else:
                    raise UnsupportedCommand("SetModeCommand requires standard or native mode")
            elif isinstance(command, occid.ArmCommand):
                self._activate_override()
                self._apply_mode(self.arm_mode_name)
                self._set_throttle(self.rx_config["rxMinUsec"])
            elif isinstance(command, occid.DisarmCommand):
                self._clear_mode(self.arm_mode_name)
            elif isinstance(command, occid.TakeoffCommand):
                self._takeoff()
            elif isinstance(command, occid.LandCommand):
                self._land()
            elif isinstance(command, occid.GoToCommand):
                self._set_goto(command)
            else:
                raise UnsupportedCommand(f"unsupported OCCID command {type(command).__name__}")
            self._respond(request_id, command, True)
        except (UnsupportedCommand, ValueError, TypeError) as exc:
            self._respond(request_id, command, False, error=str(exc))

    def _process_requests(self) -> None:
        try:
            while not self.stop_event.is_set():
                try:
                    request = self.request_queue.get(timeout=REQUEST_QUEUE_TIMEOUT_S)
                except queue.Empty:
                    continue
                self._handle_command(request)
        except BaseException as exc:
            self._capture_loop_error("process_requests", exc)

    def _takeoff(self) -> None:
        if self.takeoff_altitude_m is None:
            raise UnsupportedCommand("takeoff altitude not set")
        self._activate_override()
        self._apply_mode(self.arm_mode_name)
        start = time.monotonic()
        while not self.stop_event.is_set():
            self._set_throttle(int(self.takeoff_throttle))
            altitude_m = None
            if self.latest_location is not None and self.latest_location.altitude is not None:
                altitude_m = self.latest_location.altitude.relative_m
            if altitude_m is not None and altitude_m >= self.takeoff_altitude_m:
                self._set_throttle(int(self.hover_throttle))
                return
            if time.monotonic() - start > float(self.takeoff_timeout_s):
                raise UnsupportedCommand("takeoff timeout")
            time.sleep(float(self.state_poll_interval_s))

    def _land(self) -> None:
        start = time.monotonic()
        self._activate_override()
        self._apply_mode(self.arm_mode_name)
        while not self.stop_event.is_set():
            self._set_throttle(int(self.landing_throttle))
            altitude_m = None
            if self.latest_location is not None and self.latest_location.altitude is not None:
                altitude_m = self.latest_location.altitude.relative_m
            if altitude_m is not None and altitude_m <= float(self.in_air_alt_threshold):
                self._set_throttle(self.rx_config["rxMinUsec"])
                self._clear_mode(self.arm_mode_name)
                return
            if time.monotonic() - start > float(self.landing_timeout_s):
                raise UnsupportedCommand("landing timeout")
            time.sleep(float(self.state_poll_interval_s))

    def run(self) -> None:
        if not self.worker_threads:
            self.stop_event.clear()
            for name, target in {
                "msp-request": self._process_requests,
                "msp-state": self._state_loop,
                "msp-override": self._override_loop,
            }.items():
                thread = threading.Thread(target=target, name=name, daemon=True)
                thread.start()
                self.worker_threads[name] = thread
        self.send_online()
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
            for thread in self.worker_threads.values():
                thread.join(timeout=5.0)
            self.worker_threads = {}
            self.flush_queue(self.response_queue, self.response_topic)
            with self.api_lock:
                self.api.close()
            if self.loop_error:
                trace = self.loop_error_trace or traceback.format_exception_only(type(self.loop_error), self.loop_error)[-1].strip()
                if not self.shutdown_requested and not self.loop_error_published:
                    self.publish_error(trace)
                raise self.loop_error
            self.stop()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    MspInterface(cfg, bus_config).run()
