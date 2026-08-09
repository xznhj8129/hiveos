from __future__ import annotations

import base64
import time
import unittest
import uuid

from lib.occid_bus import occid
from plugins.execution_ingress.execution_ingress import (
    _arrival_metrics,
    _distance_m,
    decode_execution_bundle,
)


def sid(value: str) -> object:
    return occid.StringID(id_type=occid.IdentifierType.DB_ID, value=value)


def record(origin: str = "test") -> object:
    now = time.time()
    return occid.RecordMeta(
        record_id=sid(str(uuid.uuid4())),
        revision=0,
        created_ts=now,
        updated_ts=now,
        origin_system=origin,
        provenance=[],
    )


def native_b64(model: object) -> str:
    return base64.b64encode(model.encode()).decode("ascii")


class ExecutionIngressContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.asset_id = sid("uav1")
        self.executor_id = sid("mpfc:uav1")
        self.task = occid.MoveTask(
            record=record(),
            task_id=sid("task-1"),
            task_type=occid.TaskType.MOVE,
            destination=occid.GlobalPosition(
                lat=47.397742,
                lon=8.545594,
                alt=20.0,
                alt_frame=occid.AltitudeDatum.RELATIVE,
            ),
        )
        self.assignment = occid.Assignment(
            record=record(),
            assignment_id=sid("assignment-1"),
            task_id=self.task.task_id,
            assignee_id=self.asset_id,
            plan_id=sid("plan-1"),
            authority="sigma.fixture",
            assigned_by=sid("sigma.fixture"),
            assigned_at=time.time(),
            status=occid.AssignmentStatus.ASSIGNED,
            constraints=[],
        )
        self.plan = occid.Plan(
            record=record(),
            plan_id=self.assignment.plan_id,
            name="test plan",
            objective_ids=[],
            task_ids=[self.task.task_id],
            actor_ids=[self.asset_id],
            resource_ids=[],
            assignments=[],
            steps=[],
            routes=[],
            constraints=[],
            contingencies=[],
            approval_state=occid.PlanApprovalState.APPROVED,
        )
        self.execution = occid.Execution(
            record=record(),
            execution_id=sid("execution-1"),
            assignment_id=self.assignment.assignment_id,
            executor_id=self.executor_id,
            phase=occid.ExecutionPhase.CREATED,
            external_job_refs=[],
        )

    def params(self, assignment=None, task=None, plan=None) -> dict:
        return {
            "execution_b64": native_b64(self.execution),
            "assignment_b64": native_b64(assignment or self.assignment),
            "task_b64": native_b64(task or self.task),
            "plan_b64": native_b64(plan or self.plan),
        }

    def test_valid_bundle_preserves_sigma_execution_identity(self) -> None:
        bundle = decode_execution_bundle(self.params())
        self.assertEqual(bundle.execution.execution_id, self.execution.execution_id)
        self.assertEqual(bundle.assignment.assignment_id, self.assignment.assignment_id)
        self.assertEqual(bundle.task.task_id, self.task.task_id)
        self.assertIs(type(bundle.task), occid.MoveTask)

    def test_mismatched_task_relationship_is_rejected(self) -> None:
        bad_assignment = self.assignment.model_copy(update={"task_id": sid("other-task")})
        with self.assertRaisesRegex(ValueError, "Assignment.task_id"):
            decode_execution_bundle(self.params(assignment=bad_assignment))

    def test_plan_reference_requires_plan_payload(self) -> None:
        params = self.params()
        params.pop("plan_b64")
        with self.assertRaisesRegex(ValueError, "plan_b64"):
            decode_execution_bundle(params)

    def test_unapproved_plan_is_rejected_independently(self) -> None:
        draft = self.plan.model_copy(
            update={"approval_state": occid.PlanApprovalState.DRAFT}
        )
        with self.assertRaisesRegex(ValueError, "not approved"):
            decode_execution_bundle(self.params(plan=draft))

    def test_non_executable_assignment_is_rejected_independently(self) -> None:
        proposed = self.assignment.model_copy(
            update={"status": occid.AssignmentStatus.PROPOSED}
        )
        with self.assertRaisesRegex(ValueError, "not executable"):
            decode_execution_bundle(self.params(assignment=proposed))

    def test_horizontal_distance_is_metric_and_symmetric(self) -> None:
        a = occid.GlobalPosition(
            lat=45.0,
            lon=-73.0,
            alt=10.0,
            alt_frame=occid.AltitudeDatum.RELATIVE,
        )
        b = occid.GlobalPosition(
            lat=45.001,
            lon=-73.0,
            alt=10.0,
            alt_frame=occid.AltitudeDatum.RELATIVE,
        )
        ab = _distance_m(a, b)
        ba = _distance_m(b, a)
        self.assertAlmostEqual(ab, ba, places=6)
        self.assertGreater(ab, 110.0)
        self.assertLess(ab, 112.0)

    def test_relative_arrival_uses_relative_altitude_observation(self) -> None:
        location = occid.LocationState(
            position=occid.GlobalPosition(
                lat=47.397742,
                lon=8.545594,
                alt=510.0,
                alt_frame=occid.AltitudeDatum.SEA_LEVEL,
            ),
            altitude=occid.AltitudeState(
                absolute_m=510.0,
                absolute_datum=occid.AltitudeDatum.SEA_LEVEL,
                relative_m=18.5,
                relative_datum=occid.AltitudeDatum.RELATIVE,
            ),
        )
        horizontal_m, altitude_error_m = _arrival_metrics(
            location,
            self.task.destination,
        )
        self.assertAlmostEqual(horizontal_m, 0.0, places=6)
        self.assertAlmostEqual(altitude_error_m, 1.5, places=6)

    def test_arrival_rejects_missing_requested_altitude_datum(self) -> None:
        location = occid.LocationState(
            position=occid.GlobalPosition(
                lat=47.397742,
                lon=8.545594,
                alt=510.0,
                alt_frame=occid.AltitudeDatum.SEA_LEVEL,
            )
        )
        with self.assertRaisesRegex(RuntimeError, "no altitude observation"):
            _arrival_metrics(location, self.task.destination)


if __name__ == "__main__":
    unittest.main()
