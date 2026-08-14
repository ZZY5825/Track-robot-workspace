import ast
from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / 'config'
ROOT = PACKAGE.parent
LAUNCHES = {
    'decision': (
        ROOT / 'track_robot_decision' / 'launch' /
        'outdoor_follow_decision.launch.py'),
    'controller': (
        ROOT / 'track_robot_control' / 'launch' /
        'target_follow_controller.launch.py'),
    'safety': (
        ROOT / 'track_robot_safety' / 'launch' / 'motion_safety.launch.py'),
}
LIMIT_FIELDS = {
    'follow_behavior_tree_node': (
        'confirmed_max_linear', 'confirmed_max_angular'),
    'target_follow_controller_node': ('max_linear_x', 'max_angular_z'),
    'local_trajectory_planner_node': ('max_linear_x', 'max_angular_z'),
    'motion_safety_supervisor_node': ('max_linear_x', 'max_angular_z'),
    'cmd_vel_gate': ('max_linear_x', 'max_angular_z'),
}


def load_profile(filename):
    with (CONFIG / filename).open(encoding='utf-8') as profile_file:
        return yaml.safe_load(profile_file)


def parameters(profile, node):
    return profile[node]['ros__parameters']


def test_supervised_profile_has_one_consistent_limit_pair():
    profile = load_profile('human_following_supervised_test.yaml')
    pairs = {
        tuple(parameters(profile, node)[key] for key in keys)
        for node, keys in LIMIT_FIELDS.items()
    }

    assert pairs == {(0.05, 0.15)}
    assert parameters(profile, 'follow_behavior_tree_node') == {
        'confirmed_max_linear': 0.05,
        'confirmed_max_angular': 0.15,
        'lidar_max_linear': 0.0,
        'lidar_max_angular': 0.15,
        'search_max_angular': 0.0,
    }
    assert parameters(profile, 'target_follow_controller_node') == {
        'follow_distance': 2.0,
        'max_linear_x': 0.05,
        'max_angular_z': 0.15,
        'linear_accel_limit': 0.05,
        'angular_accel_limit': 0.15,
        'allow_lidar_only_forward_motion': False,
    }
    assert parameters(profile, 'motion_safety_supervisor_node') == {
        'max_linear_x': 0.05,
        'max_angular_z': 0.15,
        'require_odom': True,
        'odom_timeout_sec': 0.25,
    }
    assert parameters(profile, 'human_following_supervisor_node') == {
        'runtime_mode': 'active',
        'motion_confirmed': False,
        'blocked_disarm_timeout_sec': 10.0,
        'uncertain_authorization_timeout_sec': 1.0,
    }


def test_shadow_profile_keeps_the_same_limits_and_cannot_authorize_motion():
    profile = load_profile('human_following_shadow.yaml')
    pairs = {
        tuple(parameters(profile, node)[key] for key in keys)
        for node, keys in LIMIT_FIELDS.items()
    }

    assert pairs == {(0.05, 0.15)}
    assert parameters(profile, 'follow_behavior_tree_node') == {
        'confirmed_max_linear': 0.05,
        'confirmed_max_angular': 0.15,
        'lidar_max_linear': 0.0,
        'lidar_max_angular': 0.15,
        'search_max_angular': 0.0,
    }
    assert parameters(profile, 'target_follow_controller_node') == {
        'follow_distance': 2.0,
        'max_linear_x': 0.05,
        'max_angular_z': 0.15,
        'linear_accel_limit': 0.05,
        'angular_accel_limit': 0.15,
        'allow_lidar_only_forward_motion': False,
    }
    assert parameters(profile, 'motion_safety_supervisor_node') == {
        'max_linear_x': 0.05,
        'max_angular_z': 0.15,
        'require_odom': True,
        'odom_timeout_sec': 0.25,
    }
    assert parameters(profile, 'human_following_supervisor_node') == {
        'runtime_mode': 'shadow',
        'motion_confirmed': False,
        'blocked_disarm_timeout_sec': 10.0,
        'uncertain_authorization_timeout_sec': 1.0,
    }


def _source(path):
    assert path.is_file(), 'required launch file is missing: {}'.format(path)
    return path.read_text(encoding='utf-8')


def _declared_arguments(tree):
    return {
        call.args[0].value
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == 'DeclareLaunchArgument'
        and call.args
        and isinstance(call.args[0], ast.Constant)
    }


def _opaque_functions(tree):
    return [
        call for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == 'OpaqueFunction'
    ]


def _parameter_helper_calls(tree):
    return [
        keyword.value
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == 'Node'
        for keyword in call.keywords
        if keyword.arg == 'parameters'
        and isinstance(keyword.value, ast.Call)
        and isinstance(keyword.value.func, ast.Name)
        and keyword.value.func.id == '_profile_parameters'
    ]


def test_leaf_launches_omit_empty_profiles_and_apply_nonempty_profiles_before_overrides():
    for launch in LAUNCHES.values():
        source = _source(launch)
        tree = ast.parse(source)

        assert 'profile_config' in _declared_arguments(tree)
        assert _opaque_functions(tree)
        assert '_profile_parameters' in source
        assert "LaunchConfiguration('profile_config').perform(context)" in source
        assert 'if profile_config:' in source
        assert 'parameters.append(profile_config)' in source
        assert source.index('parameters.append(profile_config)') < source.index(
            'parameters.append(overrides)')

        assert _parameter_helper_calls(tree)
