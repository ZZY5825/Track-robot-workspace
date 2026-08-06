import json
import math
from dataclasses import dataclass
from pathlib import Path

from track_robot_semantic_search.active_search_evidence import (
    BoundedEvidenceBook,
    EvidenceConfig,
    ObjectEvidenceKey,
    ViewEvidence,
)
from track_robot_semantic_search.active_search_policy import (
    BoundedHeadingPolicy,
    SearchPolicyConfig,
    SearchState,
)


FIXTURE = Path(__file__).parent / 'data' / 'phase5a_search_replay.json'


@dataclass(frozen=True)
class ReplayResult:
    name: str
    terminal_status: str
    expected_status: str
    motion_intent_count: int
    expected_motion_intents: int
    maximum_linear_command: float
    selected_key: object
    expected_key: object


_EXPLICIT_TERMINALS = {
    'timeout': SearchState.TIMEOUT,
    'cancel': SearchState.CANCELLED,
    'sensor_unavailable': SearchState.SENSOR_UNAVAILABLE,
    'model_unavailable': SearchState.MODEL_UNAVAILABLE,
    'odometry_stale': SearchState.LOCALIZATION_UNAVAILABLE,
    'tf_failure': SearchState.LOCALIZATION_UNAVAILABLE,
    'safety_rejected': SearchState.SAFETY_REJECTED,
    'rc_takeover': SearchState.SAFETY_REJECTED,
    'estop': SearchState.SAFETY_REJECTED,
    'base_fault': SearchState.SAFETY_REJECTED,
    'nav2_spin_failed': SearchState.INTERNAL_FAULT,
    'query_changed': SearchState.CANCELLED,
    'memory_epoch_changed': SearchState.UNCERTAIN,
    'localization_epoch_changed': SearchState.UNCERTAIN,
    'handoff_key_changed': SearchState.UNCERTAIN,
    'repeated_identical_view': SearchState.SEARCH_SPACE_EXHAUSTED,
}


def _key(object_id, overrides=None):
    values = {
        'memory_epoch_id': 11,
        'global_object_id': int(object_id),
        'localization_epoch_id': 7,
        'query_id': 99,
        'query_version': 1,
    }
    values.update(overrides or {})
    return ObjectEvidenceKey(**values)


def _run_case(case):
    heading_policy = BoundedHeadingPolicy(SearchPolicyConfig.defaults())
    evidence = BoundedEvidenceBook(EvidenceConfig.defaults())
    evidence.bind_domain(11, 7, 99, 1)
    current_yaw = 0.0
    headings = []
    maximum_linear_command = 0.0

    for _ in range(int(case['rotations'])):
        decision = heading_policy.next_heading(
            SearchState.SELECTING_VIEW, 0.0, current_yaw)
        assert decision is not None
        assert abs(decision.rotation_delta_deg) <= 90.0
        assert decision.cumulative_rotation_deg <= 270.0
        # Phase 5A replay represents SearchMotionIntent, which has no linear
        # component; rotations are completed before the next observation.
        maximum_linear_command = max(maximum_linear_command, 0.0)
        heading_policy.mark_completed(decision)
        current_yaw = decision.target_yaw_rad
        headings.append(decision.relative_heading_deg)

    overrides = case.get('key_overrides')
    selected_ids = case.get('selected_ids', [])
    candidate_ids = case.get('candidate_ids', [])
    all_observations = [
        (object_id, True) for object_id in selected_ids
    ] + [
        (object_id, False) for object_id in candidate_ids
    ]
    for index, (object_id, selected) in enumerate(all_observations):
        heading = headings[index % len(headings)] if headings else 0.0
        accepted = evidence.add(
            ViewEvidence(
                key=_key(object_id, overrides),
                heading_deg=heading,
                horizontal_fov_deg=90.0,
                source_stamp_sec=100.0 + index,
                task_relevance=0.75 if selected else 0.40,
                uncertainty=0.10 if selected else 0.45,
                phase3_selected=selected,
            ),
            settled_after=99.0,
        )
        if overrides:
            assert accepted is False
        else:
            assert accepted is True

    scenario = case['scenario']
    if scenario in _EXPLICIT_TERMINALS:
        state = _EXPLICIT_TERMINALS[scenario]
        selected_key = None
    else:
        decision = evidence.evaluate(search_exhausted=bool(
            case.get('search_exhausted', False)))
        state = SearchState[decision.status.value]
        selected_key = decision.selected_key

    expected_key = (
        _key(case['expected_global_object_id'])
        if 'expected_global_object_id' in case else None
    )
    return ReplayResult(
        name=case['name'],
        terminal_status=state.value,
        expected_status=case['expected'],
        motion_intent_count=len(headings),
        expected_motion_intents=int(case['expected_motion_intents']),
        maximum_linear_command=maximum_linear_command,
        selected_key=selected_key,
        expected_key=expected_key,
    )


def _run_all_cases(path):
    fixture = json.loads(path.read_text(encoding='utf-8'))
    assert fixture['schema_version'] == 'phase5a_search_replay/1.0.0'
    return tuple(_run_case(case) for case in fixture['cases'])


def test_replay_cases_are_deterministic_and_fail_closed():
    first = _run_all_cases(FIXTURE)
    second = _run_all_cases(FIXTURE)

    assert first == second
    assert len(first) == 24
    for result in first:
        assert result.terminal_status == result.expected_status, result.name
        assert result.motion_intent_count == result.expected_motion_intents
        assert result.maximum_linear_command == 0.0
        if result.expected_status == SearchState.CONFIRMED.value:
            assert result.selected_key == result.expected_key


def test_replay_covers_required_success_failure_and_identity_boundaries():
    names = {result.name for result in _run_all_cases(FIXTURE)}

    assert names == {
        'visible_initially', 'found_after_one_rotation',
        'found_after_several_headings', 'ambiguous_then_resolved',
        'lost_static_target_reacquired_same_id', 'absent_complete_range',
        'persistent_ambiguity', 'contradictory_views',
        'repeated_identical_view', 'action_timeout', 'action_cancelled',
        'sensor_unavailable', 'model_unavailable', 'odometry_stale',
        'tf_failure', 'safety_rejected', 'rc_takeover', 'estop',
        'base_fault', 'nav2_spin_failed', 'query_changed',
        'memory_epoch_changed', 'localization_epoch_changed',
        'handoff_key_changed',
    }


def test_default_heading_sequence_stays_within_bounded_rotation_envelope():
    decisions = BoundedHeadingPolicy(
        SearchPolicyConfig.defaults()).complete_sequence(0.0)

    assert len(decisions) == 6
    assert max(abs(item.rotation_delta_deg) for item in decisions) <= 90.0
    assert all(item.rotation_delta_deg > 0.0 for item in decisions)
    assert decisions[-1].cumulative_rotation_deg <= 270.0
    assert math.isclose(decisions[-1].cumulative_rotation_deg, 270.0)
