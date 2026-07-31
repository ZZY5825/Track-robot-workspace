import track_robot_navigation.semantic_navigation_supervisor_node as supervisor

from track_robot_navigation.semantic_navigation_supervisor_node import (
    _authorization_reference_is_current,
    _authorization_survives_interruption,
)


REFERENCE = (11, 22, 33, 44, 1)


def test_same_target_allows_a_lagging_rviz_snapshot_sequence():
    assert _authorization_reference_is_current(
        REFERENCE, 10582, REFERENCE, 10581)


def test_authorization_rejects_wrong_identity_future_or_zero_sequence():
    assert not _authorization_reference_is_current(
        REFERENCE, 10582, (11, 23, 33, 44, 1), 10581)
    assert not _authorization_reference_is_current(
        REFERENCE, 10582, REFERENCE, 10583)
    assert not _authorization_reference_is_current(
        REFERENCE, 10582, REFERENCE, 0)


def test_authorization_survives_transient_target_and_safety_interruptions():
    for reason in (
            'target_stale',
            'target_position_invalid',
            'target_reference_mismatch',
            'waiting_for_correlated_inputs',
            'planner_not_ready',
            'safety_motion_not_permitted'):
        assert _authorization_survives_interruption(reason)


def test_authorization_does_not_survive_explicit_or_identity_stop():
    for reason in (
            'operator_cancel',
            'target_reference_changed',
            'safety_hard_stop'):
        assert not _authorization_survives_interruption(reason)


def test_static_target_reacquires_changed_global_id_at_same_odom_position():
    matcher = getattr(supervisor, '_same_static_target_location', None)
    assert matcher is not None
    original = (11, 22, 33, 44, 1)
    changed_id = (11, 23, 33, 44, 1)

    assert matcher(original, changed_id, (2.30, -0.10), (2.43, -0.16), 0.45)


def test_static_target_reacquisition_rejects_far_or_different_query_target():
    matcher = getattr(supervisor, '_same_static_target_location', None)
    assert matcher is not None
    original = (11, 22, 33, 44, 1)

    assert not matcher(original, (11, 23, 33, 44, 1),
                       (2.30, -0.10), (3.0, -0.10), 0.45)
    assert not matcher(original, (11, 23, 33, 45, 1),
                       (2.30, -0.10), (2.31, -0.10), 0.45)


def test_target_dropout_grace_is_bounded_and_target_only():
    grace_type = getattr(supervisor, '_TransientTargetGrace', None)
    assert grace_type is not None
    grace = grace_type(1.0)

    assert grace.should_hold('waiting_for_correlated_inputs', 10.0)
    assert grace.should_hold('target_position_invalid', 10.8)
    assert not grace.should_hold('target_position_invalid', 11.01)
    grace.reset()
    assert not grace.should_hold('odometry_stale', 20.0)
