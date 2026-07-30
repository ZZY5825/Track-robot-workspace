from track_robot_navigation.semantic_goal_policy import (
    GoalAction,
    SemanticGoalPolicy,
    SemanticGoalSnapshot,
)


def snapshot(
        *,
        sequence=1,
        key=(11, 42),
        target_age=0.1,
        goal_age=0.1,
        diagnostics_age=0.1,
        odom_age=0.1,
        lifecycle_confirmed=True,
        position_valid=True,
        references_match=True,
        safety_armed=True,
        safety_permits_motion=True,
        safety_temporarily_blocked=False):
    return SemanticGoalSnapshot(
        memory_epoch_id=key[0],
        global_object_id=key[1],
        localization_epoch_id=7,
        query_id=101,
        query_version=2,
        snapshot_sequence=sequence,
        target_age_sec=target_age,
        goal_age_sec=goal_age,
        diagnostics_age_sec=diagnostics_age,
        odom_age_sec=odom_age,
        goal_frame_id='base_link',
        target_frame_id='base_link',
        lifecycle_confirmed=lifecycle_confirmed,
        position_valid=position_valid,
        references_match=references_match,
        safety_armed=safety_armed,
        safety_permits_motion=safety_permits_motion,
        safety_temporarily_blocked=safety_temporarily_blocked,
    )


def test_shadow_requires_repeated_same_global_object_then_only_plans():
    policy = SemanticGoalPolicy(
        runtime_mode='SEMANTIC_SHADOW',
        semantic_execution_enabled=False,
        confirmation_snapshots=2,
    )

    assert policy.evaluate(snapshot(sequence=1)).action is GoalAction.HOLD
    decision = policy.evaluate(snapshot(sequence=2))

    assert decision.action is GoalAction.COMPUTE_PATH
    assert decision.key == (11, 42)
    assert policy.evaluate(snapshot(sequence=3)).action is GoalAction.HOLD


def test_shadow_can_never_request_navigation():
    policy = SemanticGoalPolicy(
        runtime_mode='SEMANTIC_SHADOW',
        semantic_execution_enabled=True,
        confirmation_snapshots=1,
    )

    assert policy.evaluate(snapshot()).action is GoalAction.COMPUTE_PATH


def test_active_requires_feature_gate_and_clear_armed_safety_state():
    disabled = SemanticGoalPolicy(
        runtime_mode='SEMANTIC_ACTIVE',
        semantic_execution_enabled=False,
        confirmation_snapshots=1,
    )
    disarmed = SemanticGoalPolicy(
        runtime_mode='SEMANTIC_ACTIVE',
        semantic_execution_enabled=True,
        confirmation_snapshots=1,
    )
    blocked = SemanticGoalPolicy(
        runtime_mode='SEMANTIC_ACTIVE',
        semantic_execution_enabled=True,
        confirmation_snapshots=1,
    )
    enabled = SemanticGoalPolicy(
        runtime_mode='SEMANTIC_ACTIVE',
        semantic_execution_enabled=True,
        confirmation_snapshots=1,
    )

    assert disabled.evaluate(snapshot()).action is GoalAction.HOLD
    assert disarmed.evaluate(
        snapshot(safety_armed=False)).action is GoalAction.HOLD
    assert blocked.evaluate(
        snapshot(safety_permits_motion=False)).action is GoalAction.HOLD
    assert enabled.evaluate(snapshot()).action is GoalAction.NAVIGATE


def test_stale_or_mismatched_inputs_fail_closed():
    invalid_cases = (
        snapshot(target_age=1.1),
        snapshot(goal_age=0.6),
        snapshot(diagnostics_age=0.6),
        snapshot(odom_age=0.3),
        snapshot(position_valid=False),
        snapshot(lifecycle_confirmed=False),
        snapshot(references_match=False),
        snapshot(key=(0, 42)),
    )
    for item in invalid_cases:
        policy = SemanticGoalPolicy(
            runtime_mode='SEMANTIC_ACTIVE',
            semantic_execution_enabled=True,
            confirmation_snapshots=1,
        )
        assert policy.evaluate(item).action is GoalAction.HOLD


def test_target_change_or_loss_cancels_an_active_navigation_goal():
    policy = SemanticGoalPolicy(
        runtime_mode='SEMANTIC_ACTIVE',
        semantic_execution_enabled=True,
        confirmation_snapshots=1,
    )
    assert policy.evaluate(snapshot()).action is GoalAction.NAVIGATE

    changed = policy.evaluate(snapshot(sequence=2, key=(11, 43)))
    assert changed.action is GoalAction.CANCEL
    assert changed.reason == 'target_reference_changed'

    assert policy.evaluate(
        snapshot(sequence=3, key=(11, 43))).action is GoalAction.NAVIGATE
    stale = policy.evaluate(
        snapshot(sequence=3, key=(11, 43), target_age=1.1))
    assert stale.action is GoalAction.CANCEL
    assert stale.reason == 'target_stale'


def test_odometry_or_safety_loss_cancels_active_goal():
    policy = SemanticGoalPolicy(
        runtime_mode='SEMANTIC_ACTIVE',
        semantic_execution_enabled=True,
        confirmation_snapshots=1,
    )
    assert policy.evaluate(snapshot()).action is GoalAction.NAVIGATE
    assert policy.evaluate(
        snapshot(odom_age=0.3)).action is GoalAction.CANCEL

    policy = SemanticGoalPolicy(
        runtime_mode='SEMANTIC_ACTIVE',
        semantic_execution_enabled=True,
        confirmation_snapshots=1,
    )
    assert policy.evaluate(snapshot()).action is GoalAction.NAVIGATE
    assert policy.evaluate(
        snapshot(safety_permits_motion=False)).action is GoalAction.CANCEL


def test_transient_obstacle_holds_goal_and_clear_state_resumes_it():
    policy = SemanticGoalPolicy(
        runtime_mode='SEMANTIC_ACTIVE',
        semantic_execution_enabled=True,
        confirmation_snapshots=1,
    )
    assert policy.evaluate(snapshot()).action is GoalAction.NAVIGATE

    blocked = policy.evaluate(snapshot(
        sequence=2,
        safety_permits_motion=False,
        safety_temporarily_blocked=True,
    ))
    assert blocked.action is GoalAction.HOLD
    assert blocked.reason == 'safety_obstacle_blocked'

    clear_again = policy.evaluate(snapshot(sequence=3))
    assert clear_again.action is GoalAction.HOLD
    assert clear_again.reason == 'goal_already_dispatched'


def test_disarm_still_cancels_during_transient_obstacle_block():
    policy = SemanticGoalPolicy(
        runtime_mode='SEMANTIC_ACTIVE',
        semantic_execution_enabled=True,
        confirmation_snapshots=1,
    )
    assert policy.evaluate(snapshot()).action is GoalAction.NAVIGATE

    decision = policy.evaluate(snapshot(
        sequence=2,
        safety_armed=False,
        safety_permits_motion=False,
        safety_temporarily_blocked=True,
    ))
    assert decision.action is GoalAction.CANCEL
    assert decision.reason == 'safety_not_armed'


def test_failed_action_dispatch_can_be_retried_without_new_target():
    policy = SemanticGoalPolicy(
        runtime_mode='SEMANTIC_SHADOW',
        semantic_execution_enabled=False,
        confirmation_snapshots=1,
    )
    assert policy.evaluate(snapshot()).action is GoalAction.COMPUTE_PATH

    policy.mark_dispatch_failed()

    assert policy.evaluate(snapshot()).action is GoalAction.COMPUTE_PATH


def test_missing_correlated_input_cancels_then_allows_recovery():
    policy = SemanticGoalPolicy(
        runtime_mode='SEMANTIC_ACTIVE',
        semantic_execution_enabled=True,
        confirmation_snapshots=1,
    )
    assert policy.evaluate(snapshot()).action is GoalAction.NAVIGATE

    assert policy.invalidate(
        'waiting_for_correlated_inputs').action is GoalAction.CANCEL
    assert policy.invalidate(
        'waiting_for_correlated_inputs').action is GoalAction.HOLD
    assert policy.evaluate(snapshot()).action is GoalAction.NAVIGATE
