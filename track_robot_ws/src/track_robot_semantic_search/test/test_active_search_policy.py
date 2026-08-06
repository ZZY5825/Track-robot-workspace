import math

import pytest

from track_robot_semantic_search.active_search_policy import (
    BoundedHeadingPolicy,
    SearchMode,
    SearchPolicyConfig,
    SearchState,
)


def test_default_policy_is_bounded_and_deterministic():
    policy = BoundedHeadingPolicy(SearchPolicyConfig.defaults())

    decisions = policy.complete_sequence(initial_yaw=0.0)

    assert [round(item.relative_heading_deg) for item in decisions] == [
        45, 90, 135, 180, 225, 270,
    ]
    assert [round(item.rotation_delta_deg) for item in decisions] == [
        45, 45, 45, 45, 45, 45,
    ]
    assert sum(abs(item.rotation_delta_deg) for item in decisions) == 270.0
    assert all(abs(item.rotation_delta_deg) <= 90.0 for item in decisions)
    assert all(item.rotation_delta_deg > 0.0 for item in decisions)
    assert sum(item.collect_evidence for item in decisions) == 6


def test_action_angle_limits_each_spin_without_truncating_the_sweep():
    config = SearchPolicyConfig.defaults(maximum_rotation_angle_deg=60.0)
    policy = BoundedHeadingPolicy(config)

    decisions = policy.complete_sequence(initial_yaw=0.0)

    assert [round(item.relative_heading_deg) for item in decisions] == [
        45, 90, 135, 180, 225, 270,
    ]
    assert all(abs(item.rotation_delta_deg) <= 60.0 for item in decisions)
    assert all(item.rotation_delta_deg > 0.0 for item in decisions)


def test_policy_normalizes_targets_and_uses_shortest_signed_delta():
    policy = BoundedHeadingPolicy(SearchPolicyConfig.defaults())

    decision = policy.next_heading(
        state=SearchState.SELECTING_VIEW,
        initial_yaw=math.radians(170.0),
        current_yaw=math.radians(170.0),
    )

    assert decision is not None
    assert math.isclose(decision.target_yaw_rad, math.radians(-145.0))
    assert math.isclose(decision.rotation_delta_deg, 45.0)


def test_terminal_states_cannot_generate_another_heading():
    terminal_states = (
        SearchState.CONFIRMED,
        SearchState.NOT_FOUND,
        SearchState.UNCERTAIN,
        SearchState.CANCELLED,
        SearchState.TIMEOUT,
        SearchState.SAFETY_REJECTED,
        SearchState.INTERNAL_FAULT,
    )

    for state in terminal_states:
        policy = BoundedHeadingPolicy(SearchPolicyConfig.defaults())
        assert policy.next_heading(state, 0.0, 0.0) is None


def test_completed_decision_cannot_be_counted_twice():
    policy = BoundedHeadingPolicy(SearchPolicyConfig.defaults())
    decision = policy.next_heading(SearchState.SELECTING_VIEW, 0.0, 0.0)
    assert decision is not None
    assert policy.pending_decision == decision
    policy.mark_completed(decision)
    assert policy.pending_decision is None

    with pytest.raises(ValueError, match='pending heading'):
        policy.mark_completed(decision)


def test_non_finite_angles_and_invalid_limits_are_rejected():
    with pytest.raises(ValueError, match='maximum_rotation_angle_deg'):
        SearchPolicyConfig.defaults(maximum_rotation_angle_deg=float('nan'))

    policy = BoundedHeadingPolicy(SearchPolicyConfig.defaults())
    with pytest.raises(ValueError, match='current_yaw'):
        policy.next_heading(SearchState.SELECTING_VIEW, 0.0, float('inf'))


def test_future_next_best_view_active_mode_is_fail_closed():
    assert SearchMode.parse('passive_only') is SearchMode.PASSIVE_ONLY
    assert SearchMode.parse('search_shadow') is SearchMode.SEARCH_SHADOW
    assert SearchMode.parse('rotation_supervised') is SearchMode.ROTATION_SUPERVISED
    assert not SearchMode.NEXT_BEST_VIEW_SHADOW.motion_enabled
    assert not SearchMode.NEXT_BEST_VIEW_ACTIVE.available_in_phase5a

    with pytest.raises(ValueError, match='search_mode'):
        SearchMode.parse('autonomous_exploration')
