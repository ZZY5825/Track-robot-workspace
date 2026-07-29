from dataclasses import replace
from pathlib import Path

from track_robot_semantic_search.phase4a_advisor import (
    AdvisoryGoal,
    AdvisoryInput,
    AdvisoryTarget,
    build_advice,
)


def target(**overrides):
    value = AdvisoryTarget(
        query_text='green bottle',
        memory_epoch_id=11,
        global_object_id=42,
        localization_epoch_id=71,
        query_id=2026072801,
        query_version=1,
        x=1.60,
        y=-0.20,
        z=0.35,
        confidence=0.72,
        uncertainty=0.28,
    )
    return replace(value, **overrides)


def ready_input(**overrides):
    value = AdvisoryInput(
        planner_status='PASS',
        planner_reason='planned',
        planner_memory_epoch_id=11,
        planner_global_object_id=42,
        planner_localization_epoch_id=71,
        planner_query_id=2026072801,
        planner_query_version=1,
        target=target(),
        goal=AdvisoryGoal(x=0.81, y=-0.10),
        path=((0.0, 0.0), (0.4, -0.05), (0.81, -0.10)),
        standoff_distance=0.80,
    )
    return replace(value, **overrides)


def test_ready_advice_contains_position_and_approach():
    result = build_advice(ready_input())

    assert result.status == 'READY'
    assert 'READY target="green bottle"' in result.text
    assert 'position=front 1.60m,right 0.20m' in result.text
    assert 'range=1.61m' in result.text
    assert 'bearing=-7.1deg' in result.text
    assert 'approach=front-right' in result.text
    assert 'goal=(0.81,-0.10)m' in result.text
    assert 'standoff=0.80m' in result.text
    assert 'path=clear' in result.text
    assert 'ADVISORY_ONLY' in result.text
    assert result.path_length_m > 0.8
    assert '\n' not in result.text
    assert len(result.text) <= 512


def test_advice_uses_ros_base_link_direction_convention():
    left = build_advice(ready_input(
        target=target(y=0.20),
        goal=AdvisoryGoal(x=0.8, y=0.1)))
    behind = build_advice(ready_input(
        target=target(x=-1.0, y=-0.2),
        goal=AdvisoryGoal(x=-0.3, y=-0.1)))

    assert 'left 0.20m' in left.text
    assert 'approach=front-left' in left.text
    assert 'position=behind 1.00m,right 0.20m' in behind.text
    assert 'approach=behind-right' in behind.text


def test_planner_failure_replaces_ready_with_not_ready():
    result = build_advice(ready_input(
        planner_status='FAIL',
        planner_reason='blocked_path',
        goal=None,
        path=(),
    ))
    assert result.status == 'NOT_READY'
    assert result.text == 'NOT_READY reason=blocked_path ADVISORY_ONLY'


def test_missing_goal_or_path_fails_closed():
    assert build_advice(ready_input(goal=None)).reason == 'missing_goal'
    assert build_advice(ready_input(path=())).reason == 'missing_path'


def test_reference_mismatch_fails_closed():
    result = build_advice(ready_input(planner_global_object_id=99))
    assert result.status == 'NOT_READY'
    assert result.reason == 'reference_mismatch'


def test_missing_target_fails_closed():
    result = build_advice(ready_input(target=None))
    assert result.reason == 'missing_target'


def test_query_text_is_bounded_ascii():
    value = build_advice(ready_input(target=target(query_text='绿瓶')))
    assert value.reason == 'invalid_query_text'


def test_ros_adapter_logs_only_advice_state_transitions():
    source = (
        Path(__file__).resolve().parents[1]
        / 'track_robot_semantic_search'
        / 'phase4a_advisor_node.py').read_text(encoding='utf-8')

    assert 'self._last_logged_text' in source
    assert 'if result.text != self._last_logged_text:' in source
