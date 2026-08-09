#!/usr/bin/env python3
"""OCCID-native Sigma execution ingress for MPFC.

This plugin is deliberately a sibling of ``uav_controller``. Sigma sends an
already-created authoritative OCCID Execution plus the immutable work records it
references. The ingress validates that bundle, selects a local handler, reports
remote semantic acceptance before any vehicle side effect, and then delegates
vehicle behavior through the existing UavClient -> uav_controller -> endpoint
stack.

The first Block 1 handler is MoveTask. Endpoint command acknowledgement is not
execution completion: completion is reported only after live LocationState shows
the vehicle at the requested destination within configured tolerances.
"""
from __future__ import annotations

import base64
import math
import time
import traceback
import uuid
from dataclasses import dataclass
from typing import Any, Dict

from lib.common import (
    apply_cfg,
    build_event_topics,
    build_request_topic,
    build_response_topic,
    build_topic_base,
)
from lib.occid_bus import occid
from lib.plugin_base import PluginBase
from lib.uav_client import UavClient


EXECUTE_ACTION = "EXECUTE_OCCID"
EVENT_KEY = "execution"
EARTH_RADIUS_M = 6371008.8


def _id_text(value: Any) -> str:
    return f"{value.id_type.name}:{value.value}"


def _decode_native_b64(value: str) -> Any:
    try:
        payload = base64.b64decode(value, validate=True)
    except Exception as exc:
        raise ValueError("invalid base64 OCCID payload") from exc
    return occid.decode_model(payload)


def _encode_native_b64(model: Any) -> str:
    return base64.b64encode(model.encode()).decode("ascii")


def _distance_m(a: Any, b: Any) -> float:
    """Great-circle horizontal distance for two OCCID GlobalPosition values."""
    lat1 = math.radians(float(a.lat))
    lat2 = math.radians(float(b.lat))
    dlat = lat2 - lat1
    dlon = math.radians(float(b.lon) - float(a.lon))
    hav = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    )
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(hav)))


@dataclass(frozen=True)
class ExecutionBundle:
    execution: Any
    assignment: Any
    task: Any
    plan: Any | None = None
    objective: Any | None = None


def decode_execution_bundle(params: Dict[str, Any]) -> ExecutionBundle:
    """Decode and relationship-check one Sigma OCCID Native work bundle."""
    required = ("execution_b64", "assignment_b64", "task_b64")
    missing = [key for key in required if not params.get(key)]
    if missing:
        raise ValueError(f"execution bundle missing fields: {', '.join(missing)}")

    execution = _decode_native_b64(str(params["execution_b64"]))
    assignment = _decode_native_b64(str(params["assignment_b64"]))
    task = _decode_native_b64(str(params["task_b64"]))
    plan = (
        None
        if not params.get("plan_b64")
        else _decode_native_b64(str(params["plan_b64"]))
    )
    objective = (
        None
        if not params.get("objective_b64")
        else _decode_native_b64(str(params["objective_b64"]))
    )

    if not isinstance(execution, occid.Execution):
        raise TypeError(f"execution_b64 is {type(execution).__name__}, expected Execution")
    if not isinstance(assignment, occid.Assignment):
        raise TypeError(f"assignment_b64 is {type(assignment).__name__}, expected Assignment")
    if not isinstance(task, occid.Task):
        raise TypeError(f"task_b64 is {type(task).__name__}, expected Task")
    if plan is not None and not isinstance(plan, occid.Plan):
        raise TypeError(f"plan_b64 is {type(plan).__name__}, expected Plan")
    if objective is not None and not isinstance(objective, occid.Objective):
        raise TypeError(f"objective_b64 is {type(objective).__name__}, expected Objective")

    if execution.assignment_id != assignment.assignment_id:
        raise ValueError(
            "Execution.assignment_id does not match supplied Assignment: "
            f"{_id_text(execution.assignment_id)} != {_id_text(assignment.assignment_id)}"
        )
    if assignment.task_id != task.task_id:
        raise ValueError(
            "Assignment.task_id does not match supplied Task: "
            f"{_id_text(assignment.task_id)} != {_id_text(task.task_id)}"
        )

    if assignment.plan_id is not None:
        if plan is None:
            raise ValueError("Assignment references a Plan but plan_b64 was not supplied")
        if assignment.plan_id != plan.plan_id:
            raise ValueError(
                "Assignment.plan_id does not match supplied Plan: "
                f"{_id_text(assignment.plan_id)} != {_id_text(plan.plan_id)}"
            )
        if task.task_id not in plan.task_ids:
            raise ValueError("supplied Plan does not contain the assigned Task")
    elif plan is not None:
        raise ValueError("plan_b64 supplied for an Assignment with no plan_id")

    if objective is not None:
        if plan is None:
            raise ValueError("objective_b64 requires a supplied Plan in this Block 1 ingress")
        if objective.objective_id not in plan.objective_ids:
            raise ValueError("supplied Objective is not referenced by the supplied Plan")

    return ExecutionBundle(
        execution=execution,
        assignment=assignment,
        task=task,
        plan=plan,
        objective=objective,
    )


class ExecutionIngress(PluginBase):
    """Correlated high-level execution ingress backed by existing MPFC UAV services."""

    def __init__(self, cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
        super().__init__(cfg, bus_config)
        apply_cfg(self, cfg)
        self.poll_interval_s = float(cfg["poll_interval_s"])
        self.response_timeout_s = float(cfg["response_timeout_s"])
        self.state_timeout_s = float(cfg["state_timeout_s"])
        self.execution_timeout_s = float(cfg["execution_timeout_s"])
        self.progress_interval_s = float(cfg["progress_interval_s"])
        self.arrival_radius_m = float(cfg["arrival_radius_m"])
        self.arrival_altitude_tolerance_m = float(cfg["arrival_altitude_tolerance_m"])
        self.executor_id = occid.StringID.model_validate(cfg["executor_id"])
        self.asset_id = occid.StringID.model_validate(cfg["asset_id"])

        base = build_topic_base(self.client_id, self.topic_ns)
        self.request_topic = build_request_topic(self.client_id, self.topic_ns)
        self.response_topic = build_response_topic(self.client_id, self.topic_ns)
        self.event_topics = build_event_topics(base, list(cfg.get("event_keys") or [EVENT_KEY]))
        if EVENT_KEY not in self.event_topics:
            raise RuntimeError(f"execution ingress requires event key {EVENT_KEY!r}")
        self.client.subscribe(self.request_topic)

        self.uav = UavClient(self, dict(cfg["interface"]), self.response_timeout_s)
        self.init_bus(
            self.poll_interval_s,
            state_topics=self.uav.state_topics(),
            response_topic=self.uav.response_topic,
        )
        self.active_execution_id: Any | None = None

    def _record_meta(self, bundle: ExecutionBundle) -> Any:
        now = time.time()
        return occid.RecordMeta(
            record_id=occid.StringID(
                id_type=occid.IdentifierType.DB_ID,
                value=str(uuid.uuid4()),
            ),
            revision=0,
            created_ts=now,
            updated_ts=now,
            origin_system=f"mpfc.{self.client_id}",
            provenance=[
                bundle.execution.record.record_id.value,
                bundle.assignment.record.record_id.value,
                bundle.task.record.record_id.value,
            ],
        )

    def _task_delta(
        self,
        bundle: ExecutionBundle,
        phase: Any,
        *,
        progress: float | None = None,
    ) -> Any:
        return occid.TaskDelta(
            record=self._record_meta(bundle),
            task_id=bundle.task.task_id,
            task_rev=bundle.task.record.revision,
            phase=phase,
            progress=progress,
            owner_id=self.asset_id,
            updated_ts=time.time(),
        )

    def _publish_execution_event(
        self,
        bundle: ExecutionBundle,
        state: str,
        *,
        task_delta: Any | None = None,
        progress: float | None = None,
        error: str | None = None,
        detail: Dict[str, Any] | None = None,
    ) -> None:
        payload: Dict[str, Any] = {
            "state": state,
            "execution_id": bundle.execution.execution_id.model_dump(mode="json"),
            "assignment_id": bundle.assignment.assignment_id.model_dump(mode="json"),
            "task_id": bundle.task.task_id.model_dump(mode="json"),
            "executor_id": self.executor_id.model_dump(mode="json"),
            "reported_at": time.time(),
        }
        if progress is not None:
            payload["progress"] = float(progress)
        if error is not None:
            payload["error"] = str(error)
        if detail:
            payload["detail"] = dict(detail)
        if task_delta is not None:
            payload["task_delta_b64"] = _encode_native_b64(task_delta)
        self._publish_event(EVENT_KEY, payload)

    def _reject_busy_request(self, request: Dict[str, Any]) -> None:
        request_id = str(request.get("request_id", "unknown"))
        action = str(request.get("action", "unknown"))
        self.enqueue_response(
            request_id,
            action,
            False,
            {
                "accepted": False,
                "error": "execution ingress is busy",
                "active_execution_id": (
                    None
                    if self.active_execution_id is None
                    else self.active_execution_id.model_dump(mode="json")
                ),
            },
        )
        self.flush_queue(self.response_queue, self.response_topic)

    def _pump_with_ingress(self, deadline: float | None = None) -> tuple[Any, Any]:
        topic, payload = self._pump_once(deadline)
        if topic == self.request_topic and payload is not None:
            self._reject_busy_request(payload["data"])
            return None, None
        return topic, payload

    def _wait_response(self, request_id: str, timeout_s: float) -> Dict[str, Any]:
        deadline = time.monotonic() + float(timeout_s)
        while True:
            if request_id in self.bus.responses:
                return self.bus.responses.pop(request_id)
            if time.monotonic() > deadline:
                raise RuntimeError(f"timeout waiting for UAV result id={request_id}")
            self._pump_with_ingress(deadline)

    def _wait_for_position(self, timeout_s: float) -> Any:
        deadline = time.monotonic() + float(timeout_s)
        while True:
            location = self.uav.location()
            if location is not None and location.position is not None:
                return location.position
            if time.monotonic() > deadline:
                raise RuntimeError("timed out waiting for UAV LocationState.position")
            self._pump_with_ingress(deadline)

    def _arrival_metrics(self, current: Any, destination: Any) -> tuple[float, float]:
        horizontal_m = _distance_m(current, destination)
        if current.alt_frame != destination.alt_frame:
            raise RuntimeError(
                "cannot evaluate arrival across different altitude datums: "
                f"current={current.alt_frame.name} destination={destination.alt_frame.name}"
            )
        altitude_error_m = abs(float(current.alt) - float(destination.alt))
        return horizontal_m, altitude_error_m

    def _execute_move(self, bundle: ExecutionBundle) -> None:
        task = bundle.task
        if not isinstance(task, occid.MoveTask):
            raise TypeError(f"unsupported task handler {type(task).__name__}; Block 1 supports MoveTask")

        current = self._wait_for_position(self.state_timeout_s)
        initial_distance_m, _ = self._arrival_metrics(current, task.destination)
        denominator = max(initial_distance_m, self.arrival_radius_m, 0.01)

        command = occid.GoToCommand(position=task.destination)
        endpoint_result = self.uav.execute(command, timeout_s=self.response_timeout_s)
        running = self._task_delta(bundle, occid.TaskPhase.RUNNING, progress=0.0)
        self._publish_execution_event(
            bundle,
            "RUNNING",
            task_delta=running,
            progress=0.0,
            detail={"endpoint_result": endpoint_result},
        )

        deadline = time.monotonic() + self.execution_timeout_s
        last_progress_publish = 0.0
        while True:
            current = self.uav.location()
            if current is not None and current.position is not None:
                horizontal_m, altitude_error_m = self._arrival_metrics(
                    current.position,
                    task.destination,
                )
                progress = max(0.0, min(1.0, 1.0 - (horizontal_m / denominator)))
                if (
                    horizontal_m <= self.arrival_radius_m
                    and altitude_error_m <= self.arrival_altitude_tolerance_m
                ):
                    complete = self._task_delta(
                        bundle,
                        occid.TaskPhase.DONE_OK,
                        progress=1.0,
                    )
                    self._publish_execution_event(
                        bundle,
                        "COMPLETED",
                        task_delta=complete,
                        progress=1.0,
                        detail={
                            "horizontal_error_m": horizontal_m,
                            "altitude_error_m": altitude_error_m,
                        },
                    )
                    return

                now = time.monotonic()
                if now - last_progress_publish >= self.progress_interval_s:
                    delta = self._task_delta(
                        bundle,
                        occid.TaskPhase.RUNNING,
                        progress=progress,
                    )
                    self._publish_execution_event(
                        bundle,
                        "PROGRESS",
                        task_delta=delta,
                        progress=progress,
                        detail={
                            "horizontal_error_m": horizontal_m,
                            "altitude_error_m": altitude_error_m,
                        },
                    )
                    last_progress_publish = now

            if time.monotonic() > deadline:
                raise RuntimeError(
                    f"MoveTask timed out after {self.execution_timeout_s:.1f}s without arrival"
                )
            self._pump_with_ingress(min(deadline, time.monotonic() + self.poll_interval_s))

    def _handle_execute(self, request: Dict[str, Any]) -> None:
        request_id = str(request.get("request_id", "unknown"))
        action = str(request.get("action", "unknown"))
        if action != EXECUTE_ACTION:
            self.enqueue_response(
                request_id,
                action,
                False,
                {"accepted": False, "error": f"unsupported action {action!r}"},
            )
            self.flush_queue(self.response_queue, self.response_topic)
            return

        bundle: ExecutionBundle | None = None
        try:
            params = request.get("params")
            if type(params) is not dict:
                raise ValueError("EXECUTE_OCCID params must be an object")
            bundle = decode_execution_bundle(params)
            if bundle.execution.executor_id != self.executor_id:
                raise ValueError(
                    "Execution.executor_id does not address this MPFC ingress: "
                    f"{_id_text(bundle.execution.executor_id)} != {_id_text(self.executor_id)}"
                )
            if bundle.assignment.assignee_id != self.asset_id:
                raise ValueError(
                    "Assignment.assignee_id does not address this MPFC asset: "
                    f"{_id_text(bundle.assignment.assignee_id)} != {_id_text(self.asset_id)}"
                )
            if not isinstance(bundle.task, occid.MoveTask):
                raise TypeError(
                    f"no local handler for {type(bundle.task).__name__}; Block 1 supports MoveTask"
                )

            # This is remote semantic acceptance, not execution completion. Flush
            # it before the first UAV side effect so Sigma can distinguish the
            # boundary truthfully.
            self.active_execution_id = bundle.execution.execution_id
            self.enqueue_response(
                request_id,
                action,
                True,
                {
                    "accepted": True,
                    "execution_id": bundle.execution.execution_id.model_dump(mode="json"),
                    "handler": type(bundle.task).__name__,
                },
            )
            self.flush_queue(self.response_queue, self.response_topic)
            accepted = self._task_delta(
                bundle,
                occid.TaskPhase.DISPATCHED,
                progress=0.0,
            )
            self._publish_execution_event(
                bundle,
                "REMOTE_ACCEPTED",
                task_delta=accepted,
                progress=0.0,
                detail={"handler": type(bundle.task).__name__},
            )

            self._execute_move(bundle)
        except Exception as exc:
            if bundle is None:
                self.enqueue_response(
                    request_id,
                    action,
                    False,
                    {"accepted": False, "error": str(exc)},
                )
                self.flush_queue(self.response_queue, self.response_topic)
            else:
                failed = self._task_delta(bundle, occid.TaskPhase.DONE_FAIL)
                self._publish_execution_event(
                    bundle,
                    "FAILED",
                    task_delta=failed,
                    error=str(exc),
                )
                self.publish_error(traceback.format_exc().strip())
        finally:
            self.active_execution_id = None

    def run(self) -> None:
        self.send_online()
        try:
            while True:
                self.flush_queue(self.response_queue, self.response_topic)
                topic, payload = self._pump_once(
                    time.monotonic() + self.poll_interval_s
                )
                if topic == self.request_topic and payload is not None:
                    self._handle_execute(payload["data"])
        except KeyboardInterrupt:
            pass
        except RuntimeError:
            self.publish_error(traceback.format_exc().strip())
            raise
        finally:
            self.stop()


def run_plugin(cfg: Dict[str, Any], bus_config: Dict[str, Any]) -> None:
    ExecutionIngress(cfg, bus_config).run()
