import pytest

from track_robot_navigation.runtime_modes import (
    RuntimeMode,
    mode_spec,
    validate_mode_request,
)


def test_planning_only_has_no_execution_components():
    spec = mode_spec(RuntimeMode.PLANNING_ONLY)

    assert spec.planner
    assert not spec.controller
    assert not spec.bt_navigator
    assert not spec.safety_chain
    assert not spec.semantic_adapter


def test_manual_mode_uses_nav2_and_safety_chain_without_semantic_adapter():
    spec = mode_spec(RuntimeMode.MANUAL_NAV2_ACTIVE)

    assert spec.planner
    assert spec.controller
    assert spec.bt_navigator
    assert spec.recoveries
    assert spec.safety_chain
    assert not spec.semantic_adapter


def test_shadow_mode_plans_semantic_goal_without_execution():
    spec = mode_spec(RuntimeMode.SEMANTIC_SHADOW)

    assert spec.planner
    assert spec.semantic_adapter
    assert not spec.controller
    assert not spec.bt_navigator
    assert not spec.safety_chain


def test_semantic_active_requires_explicit_feature_gate():
    with pytest.raises(ValueError, match='enable_semantic_execution'):
        validate_mode_request(RuntimeMode.SEMANTIC_ACTIVE, False)

    validate_mode_request(RuntimeMode.SEMANTIC_ACTIVE, True)


def test_unknown_mode_is_rejected():
    with pytest.raises(ValueError, match='runtime_mode'):
        RuntimeMode.parse('AUTO')

