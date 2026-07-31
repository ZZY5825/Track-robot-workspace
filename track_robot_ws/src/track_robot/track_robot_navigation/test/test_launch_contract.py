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
    assert "'start_obstacle_map'" in source
    assert 'if start_obstacle_map:' in source


def test_semantic_modes_start_supervisor_but_shadow_has_no_motion_servers():
    source = (
        PACKAGE_ROOT / 'launch' / 'phase4b_navigation.launch.py'
    ).read_text()

    assert "executable='semantic_navigation_supervisor'" in source
    assert "'runtime_mode': mode.value" in source
    assert "'true' if semantic_enabled else 'false'" in source
    assert 'requires the semantic supervisor delivered' not in source


def test_semantic_runtime_overrides_are_rewritten_into_node_yaml():
    source = (
        PACKAGE_ROOT / 'launch' / 'phase4b_navigation.launch.py'
    ).read_text()
    assert 'semantic_params = RewrittenYaml(' in source
    assert (
        "source_file=LaunchConfiguration('semantic_supervisor_config')"
        in source
    )
    assert "'runtime_mode': mode.value" in source
    assert "'true' if semantic_enabled else 'false'" in source
    assert 'parameters=[semantic_params]' in source


def test_semantic_execution_remains_disabled_by_default():
    source = (
        PACKAGE_ROOT / 'launch' / 'phase4b_navigation.launch.py'
    ).read_text()

    assert "'enable_semantic_execution'" in source
    assert "default_value='false'" in source


def test_operator_authorization_is_reference_bound_and_uses_safety_services():
    source = (
        PACKAGE_ROOT
        / 'track_robot_navigation'
        / 'semantic_navigation_supervisor_node.py'
    ).read_text()
    config = (
        PACKAGE_ROOT / 'config' / 'semantic_navigation.yaml'
    ).read_text()

    assert 'AuthorizeSemanticApproach' in source
    assert '_target_reference' in source
    assert '_authorization_reference_is_current(' in source
    assert 'operator_authorized=(' in source
    assert "Trigger, safety_arm_service" in source
    assert "Trigger, safety_disarm_service" in source
    assert '/semantic_navigation/authorize_approach' in config
    assert '/semantic_navigation/cancel_and_disarm' in config
    assert '/safety/arm' in config
    assert '/safety/disarm' in config
    assert 'create_publisher(Twist' not in source
    assert "'/cmd_vel'" not in source


def test_supervised_behavior_tree_uses_minimal_foxy_compatible_pipeline():
    tree = (
        PACKAGE_ROOT
        / 'behavior_trees'
        / 'navigate_supervised.xml'
    ).read_text()

    assert '<ComputePathToPose' in tree
    assert '<FollowPath' in tree
    assert '<RateController ' in tree
    assert '<PipelineSequence ' in tree
    assert '<RecoveryNode ' not in tree
    assert '<ClearEntireCostmap ' not in tree
    assert '<Wait ' not in tree
    assert '<Spin ' not in tree
    assert '<BackUp ' not in tree


def test_bt_navigator_receives_supervised_tree_as_an_explicit_parameter():
    source = (
        PACKAGE_ROOT / 'launch' / 'phase4b_navigation.launch.py'
    ).read_text()

    assert "parameters=[configured_params, {" in source
    assert (
        "'default_bt_xml_filename':\n"
        "                    LaunchConfiguration('default_bt_xml_filename')"
        in source
    )
