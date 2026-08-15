from __future__ import annotations

import unittest

from lib.occid_bus import occid
from plugins.execution_ingress.execution_ingress import (
    ExecutionIngress,
    _arrival_metrics,
    _distance_m,
    _location_identity,
    _location_position,
    validate_execution_bundle,
)


def sid(value: str) -> object:
    return occid.StringID(id_type=occid.IdentifierType.DB_ID, value=value)


def record(value: str) -> object:
    return occid.RecordMeta(
        record_id=sid(f"record.{value}"),
        revision=0,
        created_ts=1.0,
        updated_ts=1.0,
        origin_system="mpfc.tests",
        provenance=[],
    )


def build_bundle() -> tuple[object, object, object, object, object]:
    location = occid.MissionPoi(
        uid=sid("location.target"),
        name="Target",
        pos=occid.GlobalPosition(
            lat=45.5017,
            lon=-73.5673,
            alt=20.0,
            alt_frame=occid.AltitudeDatum.RELATIVE,
        ),
        origin="test",
    )
    task = occid.TaskManeuver(
        record=record("task"),
        task_id=sid("task.move"),
        instruction="Move to the designated target point and hold there.",
        target_refs=[],
        location_refs=[location.uid],
        objective_id=None,
        constraints=[],
        intent=occid.ManeuverIntent.MOVE,
    )
    plan = occid.Plan(
        record=record("plan"),
        plan_id=sid("plan.move"),
        name="move",
        objective_ids=[],
        task_ids=[task.task_id],
        actor_ids=[sid("uav1")],
        resource_ids=[],
        assignments=[],
        steps=[],
        routes=[],
        constraints=[],
        contingencies=[],
        approval_state=occid.PlanApprovalState.APPROVED,
    )
    assignment = occid.Assignment(
        record=record("assignment"),
        assignment_id=sid("assignment.move"),
        task_id=task.task_id,
        assignee_id=sid("uav1"),
        plan_id=plan.plan_id,
        authority_id=None,
        assigned_by=sid("control"),
        assigned_at=1.0,
        status=occid.AssignmentStatus.ASSIGNED,
        constraints=[],
    )
    execution = occid.Execution(
        record=record("execution"),
        execution_id=sid("execution.move"),
        assignment_id=assignment.assignment_id,
        executor_id=sid("mpfc:uav1"),
        phase=occid.ExecutionPhase.CREATED,
        external_job_refs=[sid("dispatch.move.1")],
    )
    return location, task, plan, assignment, execution


class ExecutionIngressTests(unittest.TestCase):
    def test_bundle_validation_preserves_execution_correlation(self) -> None:
        _, task, plan, assignment, execution = build_bundle()
        bundle = validate_execution_bundle(execution, assignment, task, plan)
        self.assertEqual(bundle.task.task_id, assignment.task_id)
        self.assertEqual(bundle.plan.plan_id, assignment.plan_id)
        self.assertEqual(bundle.execution.assignment_id, assignment.assignment_id)

    def test_bundle_validation_rejects_mismatched_assignment(self) -> None:
        _, task, plan, assignment, execution = build_bundle()
        bad_execution = execution.model_copy(update={"assignment_id": sid("assignment.other")})
        with self.assertRaisesRegex(ValueError, "Execution.assignment_id"):
            validate_execution_bundle(bad_execution, assignment, task, plan)

    def test_unapproved_plan_is_rejected_independently(self) -> None:
        _, task, plan, assignment, execution = build_bundle()
        draft = plan.model_copy(update={"approval_state": occid.PlanApprovalState.DRAFT})
        with self.assertRaisesRegex(ValueError, "not approved"):
            validate_execution_bundle(execution, assignment, task, draft)

    def test_move_task_resolves_global_position_through_location_ref(self) -> None:
        location, task, _, _, _ = build_bundle()
        ingress = ExecutionIngress.__new__(ExecutionIngress)
        ingress.records = {
            ("control", "location", "DB_ID:location.target"): location,
        }
        destination = ingress._resolve_move_destination("control", task)
        self.assertEqual(destination, location.pos)
        self.assertEqual(_location_identity(location), location.uid)
        self.assertEqual(_location_position(location), location.pos)

    def test_move_task_rejects_unresolved_location(self) -> None:
        _, task, _, _, _ = build_bundle()
        ingress = ExecutionIngress.__new__(ExecutionIngress)
        ingress.records = {}
        with self.assertRaisesRegex(ValueError, "unresolved"):
            ingress._resolve_move_destination("control", task)

    def test_unsupported_task_family_rejects_before_execution(self) -> None:
        _, task, _, _, _ = build_bundle()
        data = task.model_dump(exclude={"intent"})
        info_task = occid.TaskInformation(
            **data,
            intent=occid.InformationIntent.SEARCH,
        )
        ingress = ExecutionIngress.__new__(ExecutionIngress)
        ingress.records = {}
        with self.assertRaisesRegex(TypeError, "TaskManeuver/MOVE"):
            ingress._resolve_move_destination("control", info_task)

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

    def test_arrival_metrics_respect_relative_altitude_datum(self) -> None:
        location, _, _, _, _ = build_bundle()
        observed = occid.LocationState(
            position=occid.GlobalPosition(
                lat=location.pos.lat,
                lon=location.pos.lon,
                alt=500.0,
                alt_frame=occid.AltitudeDatum.SEA_LEVEL,
            ),
            altitude=occid.AltitudeState(
                absolute_m=500.0,
                absolute_datum=occid.AltitudeDatum.SEA_LEVEL,
                relative_m=19.5,
                relative_datum=occid.AltitudeDatum.RELATIVE,
            ),
        )
        horizontal_m, altitude_error_m = _arrival_metrics(observed, location.pos)
        self.assertAlmostEqual(horizontal_m, 0.0, places=5)
        self.assertAlmostEqual(altitude_error_m, 0.5, places=5)


if __name__ == "__main__":
    unittest.main()
