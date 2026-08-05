import math
from pathlib import Path

import pytest
import yaml

from track_robot_navigation.search_motion_adapter import (
    MotionIntentRequest,
    MotionLimits,
    SearchMotionPolicy,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_SOURCE = (
    PACKAGE_ROOT
    / 'track_robot_navigation'
    / 'search_motion_adapter_node.py'
)
CONFIG = PACKAGE_ROOT / 'config' / 'active_search_motion.yaml'


def _intent(
        query_id=44,
        angle=math.radians(45.0),
        speed=0.30,
        deadline=20.0,
        rotation_permitted=True,
        forward_permitted=False,
        stop=False):
    return MotionIntentRequest(
        query_id=query_id,
        signed_rotation_rad=angle,
        maximum_rotation_rad=math.radians(90.0),
        maximum_angular_speed_rad_s=speed,
        deadline_monotonic=deadline,
        rotation_permitted=rotation_permitted,
        forward_permitted=forward_permitted,
        stop=stop,
    )


def test_authorization_is_bound_once_to_one_pending_query():
    policy = SearchMotionPolicy(MotionLimits.defaults())
    assert policy.limits == MotionLimits.defaults()
    assert policy.accept_intent(_intent(), now_monotonic=10.0).accepted

    assert policy.authorize(query_id=44, now_monotonic=10.1).accepted
    assert policy.authorized_query_id == 44
    assert not policy.authorize(query_id=45, now_monotonic=10.2).accepted


def test_authorization_persists_across_headings_for_same_query():
    policy = SearchMotionPolicy(MotionLimits.defaults())
    policy.accept_intent(_intent(), now_monotonic=10.0)
    policy.authorize(44, now_monotonic=10.1)
    assert policy.begin_spin(44, now_monotonic=10.2).accepted
    policy.complete_spin(44)

    assert policy.accept_intent(
        _intent(angle=math.radians(-45.0)), now_monotonic=11.0).accepted
    assert policy.begin_spin(44, now_monotonic=11.1).accepted
    assert policy.authorized_query_id == 44


def test_forward_permission_is_always_rejected():
    policy = SearchMotionPolicy(MotionLimits.defaults())

    result = policy.accept_intent(
        _intent(forward_permitted=True), now_monotonic=10.0)

    assert not result.accepted
    assert result.reason == 'forward_motion_forbidden'


def test_shadow_intent_is_recorded_but_never_authorizable():
    policy = SearchMotionPolicy(MotionLimits.defaults())
    result = policy.accept_intent(
        _intent(rotation_permitted=False), now_monotonic=10.0)

    assert result.accepted
    assert result.reason == 'shadow_intent_recorded'
    assert not policy.authorize(44, now_monotonic=10.1).accepted


def test_angle_speed_and_deadline_limits_are_fail_closed():
    policy = SearchMotionPolicy(MotionLimits.defaults())

    assert policy.accept_intent(
        _intent(angle=math.radians(91.0)), 10.0
    ).reason == 'rotation_angle_limit_exceeded'
    assert policy.accept_intent(
        _intent(speed=0.31), 10.0
    ).reason == 'rotation_speed_limit_exceeded'
    assert policy.accept_intent(
        _intent(deadline=9.9), 10.0
    ).reason == 'intent_deadline_expired'


def test_safety_fault_clears_authorization_and_requests_stop():
    policy = SearchMotionPolicy(MotionLimits.defaults())
    policy.accept_intent(_intent(), 10.0)
    policy.authorize(44, 10.1)
    policy.begin_spin(44, 10.2)

    transition = policy.update_safety(
        healthy=False,
        reason='rc_override',
    )

    assert transition.cancel_spin
    assert transition.disarm
    assert transition.reason == 'rc_override'
    assert policy.authorized_query_id is None


def test_stop_intent_cancels_and_disarms_the_task():
    policy = SearchMotionPolicy(MotionLimits.defaults())
    policy.accept_intent(_intent(), 10.0)
    policy.authorize(44, 10.1)

    transition = policy.accept_intent(
        _intent(stop=True), now_monotonic=10.2)

    assert transition.accepted
    assert transition.cancel_spin
    assert transition.disarm
    assert policy.authorized_query_id is None


def test_adapter_source_uses_spin_but_has_no_velocity_or_pose_client():
    source = NODE_SOURCE.read_text()

    assert 'ActionClient' in source
    assert 'Spin' in source
    assert "'/spin'" in source
    assert 'NavigateToPose' not in source
    assert 'geometry_msgs.msg import Twist' not in source
    assert "'/cmd_vel'" not in source
    assert 'int(message.header.stamp.sec)' not in source
    assert 'self._spin_client.destroy()' in source
    assert 'except (KeyboardInterrupt, RuntimeError):' in source
    assert 'forward_motion_forbidden' in (
        PACKAGE_ROOT
        / 'track_robot_navigation'
        / 'search_motion_adapter.py'
    ).read_text()


def test_adapter_config_has_bounded_freshness_and_service_names():
    params = yaml.safe_load(CONFIG.read_text())[
        'search_motion_adapter']['ros__parameters']

    assert params['maximum_individual_rotation_deg'] == 90.0
    assert params['maximum_angular_speed_rad_s'] == 0.30
    assert params['odometry_timeout_sec'] <= 0.25
    assert params['safety_timeout_sec'] <= 0.25
    assert params['intent_timeout_sec'] <= 0.50
    assert params['authorize_service'] == (
        '/semantic_search/active_search/authorize_rotation')
    assert params['cancel_service'] == '/semantic_search/active_search/cancel'


def test_non_finite_motion_values_are_rejected():
    policy = SearchMotionPolicy(MotionLimits.defaults())

    with pytest.raises(ValueError, match='signed_rotation_rad'):
        policy.accept_intent(_intent(angle=float('nan')), 10.0)
