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
    assert params['max_linear_x'] <= 0.15
    assert params['max_angular_z'] <= 0.35


def test_existing_supervisor_defaults_remain_unchanged():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'motion_safety_supervisor.yaml').read_text()
    )
    params = config['motion_safety_supervisor_node']['ros__parameters']

    assert params['planned_cmd_topic'] == '/follow/cmd_vel_avoiding'
    assert params['safe_cmd_topic'] == '/follow/cmd_vel_safe'
    assert params['require_planner_state'] is True
    assert 'require_odom' not in params

