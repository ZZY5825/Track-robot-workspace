from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_controller_output_is_remapped_away_from_final_cmd_vel():
    source = (
        PACKAGE_ROOT / 'launch' / 'phase4b_navigation.launch.py'
    ).read_text()

    assert "('cmd_vel', '/nav2/cmd_vel_raw')" in source
    assert source.count('remappings=NAV2_CMD_REMAPPINGS') == 2
    assert "output_topic': '/cmd_vel'" not in source


def test_launch_uses_only_nav2_servers_for_navigation():
    source = (
        PACKAGE_ROOT / 'launch' / 'phase4b_navigation.launch.py'
    ).read_text()

    assert "package='nav2_planner'" in source
    assert "package='nav2_controller'" in source
    assert "package='nav2_bt_navigator'" in source
    assert "package='nav2_recoveries'" in source
    assert "package='nav2_lifecycle_manager'" in source
    assert 'local_trajectory_planner_node' not in source


def test_active_modes_include_existing_safety_chain():
    source = (
        PACKAGE_ROOT / 'launch' / 'phase4b_navigation.launch.py'
    ).read_text()

    assert "package='track_robot_safety'" in source
    assert "executable='motion_safety_supervisor_node'" in source
    assert "package='track_robot_core'" in source
    assert "executable='cmd_vel_gate'" in source
    assert 'motion_safety_supervisor_nav2.yaml' in source
    assert 'cmd_vel_gate_nav2.yaml' in source


def test_supervised_behavior_tree_has_no_spin_or_backup():
    tree = (
        PACKAGE_ROOT
        / 'behavior_trees'
        / 'navigate_supervised.xml'
    ).read_text()

    assert '<ComputePathToPose' in tree
    assert '<FollowPath' in tree
    assert '<Wait ' in tree
    assert '<Spin ' not in tree
    assert '<BackUp ' not in tree
