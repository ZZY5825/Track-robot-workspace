from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_supervisor_has_optional_odometry_freshness_gate():
    source = (
        PACKAGE_ROOT / 'src' / 'motion_safety_supervisor_node.cpp'
    ).read_text()

    assert 'nav_msgs/msg/odometry.hpp' in source
    assert '"odom_topic"' in source
    assert '"require_odom"' in source
    assert '"odom_timeout_sec"' in source
    assert '"odometry_stale"' in source
    assert 'odomCallback' in source


def test_phase4b_config_routes_nav2_through_supervisor():
    config_path = (
        PACKAGE_ROOT / 'config' / 'motion_safety_supervisor_nav2.yaml'
    )
    config = yaml.safe_load(config_path.read_text())
    params = config['motion_safety_supervisor_node']['ros__parameters']

    assert params['planned_cmd_topic'] == '/nav2/cmd_vel_raw'
    assert params['safe_cmd_topic'] == '/nav2/cmd_vel_safe'
    assert params['odom_topic'] == '/odom'
    assert params['require_odom'] is True
    assert params['require_planner_state'] is False
    assert params['allow_arm_without_command'] is True
    assert params['max_linear_x'] == 0.15
    assert params['max_angular_z'] == 0.50
    assert params['footprint_length'] == 0.88
    assert params['footprint_width'] == 0.80
    assert params['safety_inflation'] == 0.0
    assert params['bounded_rotation_collision_enabled'] is True
    assert params['angular_braking_deceleration'] == 0.80
    assert params['fixed_rotation_margin'] == 0.05
    stopping_distance = (
        params['max_linear_x'] ** 2 / (2.0 * params['braking_deceleration'])
        + params['max_linear_x'] * params['response_latency_sec']
        + params['fixed_stop_margin']
    )
    assert abs(stopping_distance - 0.5325) < 1e-9


def test_phase4b_obstacle_visualization_uses_physical_footprint_only():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'local_obstacle_map.yaml').read_text()
    )
    params = config['local_obstacle_map_node']['ros__parameters']

    assert params['footprint_length'] == 0.88
    assert params['footprint_width'] == 0.80
    assert params['safety_inflation'] == 0.0


def test_existing_supervisor_defaults_remain_unchanged():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'motion_safety_supervisor.yaml').read_text()
    )
    params = config['motion_safety_supervisor_node']['ros__parameters']

    assert params['planned_cmd_topic'] == '/follow/cmd_vel_avoiding'
    assert params['safe_cmd_topic'] == '/follow/cmd_vel_safe'
    assert params['require_planner_state'] is True
    assert 'require_odom' not in params
    assert 'allow_arm_without_command' not in params


def test_nav2_arm_bootstrap_keeps_runtime_command_freshness_gate():
    source = (
        PACKAGE_ROOT / 'src' / 'motion_safety_supervisor_node.cpp'
    ).read_text()

    assert '"allow_arm_without_command"' in source
    assert '!allow_arm_without_command_' in source
    assert 'decision.reason = "planned_command_stale"' in source
