#!/usr/bin/env python3
"""OCCID-native UAV service plugin.

Programs talk to this plugin as the stable UAV API. It applies reusable
vehicle-level policy and forwards OCCID commands/state to the selected endpoint
adapter without introducing another semantic vocabulary.
"""

from __future__ import annotations

import time
import traceback
from typing import Any, Dict

from lib.common import (
    apply_cfg,
    build_envelope,
    build_event_topics,
    build_request_topic,
    build_response_topic,
    build_state_topics,
    build_topic_base,
)
from lib.occid_bus import decode_occid_command, occid, pack_occid, send_occid_command, unpack_occid
from lib.occid_topics import FLIGHT_CONTROL, LOCATION
from lib.plugin_base import PluginBase


class ControllerActionError(RuntimeError):
    pass


class UavController(PluginBase):
    """Backend-independent UAV service built on OCCID commands and state."""

    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)
        self.poll_interval_s = float(cfg["poll_interval_s"])
        self.response_timeout_s = float(cfg["response_timeout_s"])
        self.vehicle = dict(cfg["vehicle"])
        self.backend = dict(cfg["backend"])
        self.backend_state_keys = list(cfg["backend_state_keys"])
        self.backend_event_keys = list(cfg.get("backend_event_keys", []))
        self.arm_ready_since: float | None = None
        self.takeoff_ready_since: float | None = None
        self.backend_flight_control: Any | None = None

        base = build_topic_base(self.client_id, self.topic_ns)
        self.request_topic = build_request_topic(self.client_id, self.topic_ns)
        self.response_topic = build_response_topic(self.client_id, self.topic_ns)
        self.state_publish_topics = build_state_topics(base, self.backend_state_keys)
        self.event_topics = build_event_topics(base, self.backend_event_keys)
        self.client.subscribe(self.request_topic)

        backend_base = build_topic_base(self.backend["id"], self.backend["topic_ns"])
        self.backend_request_topic = build_request_topic(self.backend["id"], self.backend["topic_ns"])
        self.backend_response_topic = build_response_topic(self.backend["id"], self.backend["topic_ns"])
        self.backend_state_topics = build_state_topics(backend_base, self.backend_state_keys)
        self.backend_state_topic_to_key = {topic: key for key, topic in self.backend_state_topics.items()}
        self.backend_event_topics = build_event_topics(backend_base, self.backend_event_keys)
        self.backend_event_topic_to_key = {topic: key for key, topic in self.backend_event_topics.items()}
        self.init_bus(self.poll_interval_s, self.backend_state_topics, self.backend_response_topic)
        self.responses = self.bus.responses
        for topic in self.backend_event_topics.values():
            self.client.subscribe(topic)

    def _publish_state(self, key: str, payload: Any) -> None:
        topic = self.state_publish_topics[key]
        self.client.publish(topic, build_envelope(self.client_id, topic, payload))

    def _is_ardupilot(self) -> bool:
        return str(self.vehicle.get("autopilot", "")).upper() == occid.AutopilotType.ARDUPILOT.name

    def _location_available(self) -> bool:
        payload = self.state.get(LOCATION)
        if payload is None:
            return False
        try:
            return isinstance(unpack_occid(payload), occid.LocationState)
        except (TypeError, ValueError, KeyError):
            return False

    def _apply_readiness_policy(self, flight_control: Any) -> Any:
        readiness = flight_control.readiness
        if readiness is None:
            readiness = occid.NavReadinessState(mode_problems=[], health_problems=[])
        nav = flight_control.navigation_validity
        if nav is None:
            nav = occid.NavigationValidity()

        arm_candidate = bool(readiness.armable)
        takeoff_candidate = (
            arm_candidate
            and self._location_available()
            and bool(nav.local_position_ok)
            and bool(nav.global_position_ok)
            and bool(nav.home_position_ok)
        )

        arm_hold_s = float(self.arm_ready_hold_s)
        takeoff_hold_s = float(self.takeoff_ready_hold_s)
        if self._is_ardupilot():
            arm_hold_s = float(self.ardupilot_arm_ready_hold_s)
            takeoff_hold_s = float(self.ardupilot_takeoff_ready_hold_s)
            takeoff_candidate = takeoff_candidate and bool(readiness.ekf_using_gps)

        now = time.monotonic()
        if arm_candidate:
            if self.arm_ready_since is None:
                self.arm_ready_since = now
        else:
            self.arm_ready_since = None

        if takeoff_candidate:
            if self.takeoff_ready_since is None:
                self.takeoff_ready_since = now
        else:
            self.takeoff_ready_since = None

        readiness = readiness.model_copy(
            update={
                "arm_ready": (
                    arm_candidate
                    and self.arm_ready_since is not None
                    and now - self.arm_ready_since >= arm_hold_s
                ),
                "takeoff_ready": (
                    takeoff_candidate
                    and self.takeoff_ready_since is not None
                    and now - self.takeoff_ready_since >= takeoff_hold_s
                ),
            }
        )
        return flight_control.model_copy(update={"readiness": readiness, "navigation_validity": nav})

    def _publish_controller_flight_control(self) -> None:
        if self.backend_flight_control is None or FLIGHT_CONTROL not in self.state_publish_topics:
            return
        model = self._apply_readiness_policy(self.backend_flight_control)
        self._publish_state(FLIGHT_CONTROL, pack_occid(model))

    def _pump_controller_once(self, deadline: float | None = None) -> tuple[Any, Any]:
        topic, payload = self._pump_once(deadline)
        if topic in self.backend_state_topic_to_key:
            state_key = self.backend_state_topic_to_key[topic]
            state_payload = payload["data"]
            if state_key == FLIGHT_CONTROL:
                model = unpack_occid(state_payload)
                if not isinstance(model, occid.FlightControlState):
                    raise RuntimeError(
                        f"backend flight_control payload must be FlightControlState actual={type(model).__name__}"
                    )
                self.backend_flight_control = model
                self._publish_controller_flight_control()
            else:
                self._publish_state(state_key, state_payload)
                if state_key == LOCATION:
                    self._publish_controller_flight_control()
        elif topic in self.backend_event_topic_to_key:
            self._publish_event(self.backend_event_topic_to_key[topic], payload["data"])
        return topic, payload

    def _wait_backend_response(self, request_id: str, timeout_s: float) -> Dict[str, Any]:
        deadline = time.monotonic() + float(timeout_s)
        while True:
            if request_id in self.responses:
                return self.responses.pop(request_id)
            if time.monotonic() > deadline:
                raise ControllerActionError(f"backend response wait timed out request_id={request_id}")
            self._pump_controller_once(deadline)

    def _send_backend_command(self, command: Any) -> Dict[str, Any]:
        backend_request_id = send_occid_command(self.bus, self.backend_request_topic, command)
        return self._wait_backend_response(backend_request_id, self.response_timeout_s)

    def _handle_request(self, request: Dict[str, Any]) -> None:
        request_id, command = decode_occid_command(request)
        try:
            backend_response = self._send_backend_command(command)
            self.enqueue_response(
                request_id,
                type(command).__name__,
                bool(backend_response.get("ok")),
                dict(backend_response.get("data") or {}),
            )
        except (ControllerActionError, TypeError, ValueError) as exc:
            self.enqueue_response(request_id, type(command).__name__, False, {"error": str(exc)})

    def run(self) -> None:
        self.send_online()
        try:
            while True:
                self.flush_queue(self.response_queue, self.response_topic)
                deadline = time.monotonic() + self.poll_interval_s
                topic, payload = self._pump_controller_once(deadline)
                if topic is None:
                    continue
                if topic == self.request_topic:
                    self._handle_request(payload["data"])
                    self.flush_queue(self.response_queue, self.response_topic)
        except RuntimeError:
            self.publish_error(traceback.format_exc().strip())
            raise
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    UavController(cfg, bus_config).run()
