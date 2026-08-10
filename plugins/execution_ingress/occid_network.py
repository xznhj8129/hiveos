"""Network-facing OCCID execution record assembly for execution_ingress.

HiveLink delivers individual canonical OCCID models. This helper keeps only the
small amount of node-local correlation needed to assemble the existing MPFC
execution bundle without exposing MPFC's private MQTT request vocabulary on the
network.
"""
from __future__ import annotations

import base64
from dataclasses import dataclass
from typing import Any, Dict

from lib.common import build_envelope
from lib.occid_bus import occid, pack_occid, unpack_occid


@dataclass(frozen=True)
class NetworkExecutionRequest:
    source: str
    dispatch_id: str
    request: Dict[str, Any]


def _key(value: Any) -> str:
    return f"{value.id_type.name}:{value.value}"


def _b64(model: Any) -> str:
    return base64.b64encode(model.encode()).decode("ascii")


class OccidNetworkBridge:
    def __init__(
        self,
        client: Any,
        client_id: str,
        *,
        in_topic: str = "OCCID/IN",
        out_topic: str = "OCCID/OUT",
    ) -> None:
        self.client = client
        self.client_id = client_id
        self.in_topic = str(in_topic)
        self.out_topic = str(out_topic)
        self.records: dict[tuple[str, str, str], Any] = {}
        self.pending: dict[tuple[str, str], Any] = {}
        self.client.subscribe(self.in_topic)

    def _store(self, source: str, model: Any) -> None:
        if isinstance(model, occid.Plan):
            self.records[(source, "plan", _key(model.plan_id))] = model
        elif isinstance(model, occid.Task):
            self.records[(source, "task", _key(model.task_id))] = model
        elif isinstance(model, occid.Assignment):
            self.records[(source, "assignment", _key(model.assignment_id))] = model
        elif isinstance(model, occid.Objective):
            self.records[(source, "objective", _key(model.objective_id))] = model
        elif isinstance(model, occid.Execution):
            dispatch_id = self.dispatch_id(model)
            self.pending[(source, dispatch_id)] = model

    @staticmethod
    def dispatch_id(execution: Any) -> str:
        if not execution.external_job_refs:
            raise ValueError("network Execution must carry an exact dispatch id in external_job_refs")
        return str(execution.external_job_refs[-1].value)

    def _assemble(self, source: str, execution: Any) -> NetworkExecutionRequest | None:
        dispatch_id = self.dispatch_id(execution)
        assignment = self.records.get((source, "assignment", _key(execution.assignment_id)))
        if assignment is None:
            return None
        task = self.records.get((source, "task", _key(assignment.task_id)))
        if task is None:
            return None
        plan = None
        if assignment.plan_id is not None:
            plan = self.records.get((source, "plan", _key(assignment.plan_id)))
            if plan is None:
                return None

        params: Dict[str, Any] = {
            "execution_b64": _b64(execution),
            "assignment_b64": _b64(assignment),
            "task_b64": _b64(task),
        }
        if plan is not None:
            params["plan_b64"] = _b64(plan)
            for objective_id in plan.objective_ids:
                objective = self.records.get((source, "objective", _key(objective_id)))
                if objective is not None:
                    params["objective_b64"] = _b64(objective)
                    break

        return NetworkExecutionRequest(
            source=source,
            dispatch_id=dispatch_id,
            request={
                "request_id": dispatch_id,
                "action": "EXECUTE_OCCID",
                "params": params,
            },
        )

    def ingest(self, envelope: Dict[str, Any]) -> list[NetworkExecutionRequest]:
        data = envelope.get("data")
        if type(data) is not dict:
            raise ValueError("OCCID/IN data must be an object")
        source = str(data.get("source") or "")
        if not source:
            raise ValueError("OCCID/IN is missing source node id")
        model = unpack_occid(data["model"])
        self._store(source, model)

        ready: list[NetworkExecutionRequest] = []
        for (pending_source, dispatch_id), execution in list(self.pending.items()):
            if pending_source != source:
                continue
            assembled = self._assemble(pending_source, execution)
            if assembled is None:
                continue
            ready.append(assembled)
            self.pending.pop((pending_source, dispatch_id), None)
        return ready

    def send_model(self, dest: str, model: Any) -> None:
        topic_data = {
            "dest": str(dest),
            "model": pack_occid(model),
        }
        self.client.publish(
            self.out_topic,
            build_envelope(self.client_id, self.out_topic, topic_data),
        )

    def send_acceptance(
        self,
        dest: str,
        *,
        execution_id: Any,
        dispatch_id: str,
        executor_id: Any,
        accepted: bool,
        retryable: bool = False,
        reason: str | None = None,
        reported_at: float,
    ) -> Any:
        report = occid.ExecutionAcceptance(
            execution_id=execution_id,
            dispatch_id=occid.StringID(
                id_type=occid.IdentifierType.DB_ID,
                value=str(dispatch_id),
            ),
            executor_id=executor_id,
            accepted=bool(accepted),
            retryable=bool(retryable),
            reason=reason,
            reported_at=float(reported_at),
        )
        self.send_model(dest, report)
        return report
