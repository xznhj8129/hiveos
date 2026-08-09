from __future__ import annotations

import base64
import time
import unittest
import uuid

from lib.occid_bus import occid
from plugins.execution_ingress.execution_ingress import (
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
            authority="sigma.fixture",
            assigned_by=sid("sigma.fixture"),
            assigned_at=time.time(),
            status=occid.AssignmentStatus.ASSIGNED,
            constraints=[],
        )
        self.execution = occid.Execution(
            record=record(),
            execution_id=sid("execution-1"),
            assignment_id=self.assignment.assignment_id,
            executor_id=self.executor_id,
            phase=occid.ExecutionPhase.CREATED,
            external_job_refs=[],
        )

    def params(self, assignment=None, task=None) -> dict:
        return {
            "execution_b64": native_b64(self.execution),
            "assignment_b64": native_b64(assignment or self.assignment),
            "task_b64": native_b64(task or self.task),
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
        bad_assignment = self.assignment.model_copy(update={"plan_id": sid("plan-1")})
        with self.assertRaisesRegex(ValueError, "plan_b64"):
            decode_execution_bundle(self.params(assignment=bad_assignment))

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


if __name__ == "__main__":
    unittest.main()
