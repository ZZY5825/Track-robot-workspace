from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _params():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'nav2_phase4b.yaml').read_text()
    )
    return config


def test_navfn_astar_and_regulated_pure_pursuit_are_selected():
    config = _params()

    planner = config['planner_server']['ros__parameters']['GridBased']
    controller_params = config['controller_server']['ros__parameters']
    controller = controller_params['FollowPath']

    assert planner['plugin'] == 'nav2_navfn_planner/NavfnPlanner'
    assert planner['use_astar'] is True
    assert controller['plugin'].endswith('RegulatedPurePursuitController')
    assert controller['desired_linear_vel'] == 0.15
    assert (
        controller['max_linear_accel']
        >= controller['desired_linear_vel']
        * controller_params['controller_frequency']
    )
    assert controller['rotate_to_heading_angular_vel'] == 0.40
    assert controller['max_linear_decel'] == 0.25
    assert controller['max_angular_accel'] == 0.50
    assert controller_params['progress_checker']['movement_time_allowance'] == 30.0


def test_costmaps_are_short_range_rolling_odom_maps():
    config = _params()

    for name in ('global_costmap', 'local_costmap'):
        params = config[name][name]['ros__parameters']
        assert params['global_frame'] == 'odom'
        assert params['robot_base_frame'] == 'base_link'
        assert params['rolling_window'] is True
        assert params['resolution'] == 0.05
        assert params['footprint'] == (
            '[[-0.60,-0.50],[-0.60,0.50],'
            '[0.60,0.50],[0.60,-0.50]]'
        )
        assert isinstance(params['width'], int)
        assert isinstance(params['height'], int)
        assert params['width'] <= 12.0
        assert params['height'] <= 12.0
        assert params['inflation_layer']['inflation_radius'] == 0.105625


def test_costmaps_use_standard_lidar_layers():
    config = _params()
    local = config['local_costmap']['local_costmap']['ros__parameters']
    global_map = config['global_costmap']['global_costmap']['ros__parameters']

    assert local['voxel_layer']['plugin'] == 'nav2_costmap_2d::VoxelLayer'
    assert local['voxel_layer']['z_voxels'] <= 16
    assert local['voxel_layer']['observation_sources'] == (
        'raw_clear filtered_mark')
    assert local['voxel_layer']['raw_clear']['topic'] == '/rslidar_points'
    assert local['voxel_layer']['raw_clear']['clearing'] is True
    assert local['voxel_layer']['raw_clear']['marking'] is False
    assert local['voxel_layer']['filtered_mark']['topic'] == (
        '/safety/filtered_obstacle_points')
    assert local['voxel_layer']['filtered_mark']['clearing'] is True
    assert local['voxel_layer']['filtered_mark']['marking'] is True
    assert (
        global_map['obstacle_layer']['plugin']
        == 'nav2_costmap_2d::ObstacleLayer'
    )
    assert global_map['obstacle_layer']['observation_sources'] == (
        'raw_clear filtered_mark')
    assert global_map['obstacle_layer']['raw_clear']['topic'] == (
        '/rslidar_points')
    assert global_map['obstacle_layer']['raw_clear']['clearing'] is True
    assert global_map['obstacle_layer']['raw_clear']['marking'] is False
    assert global_map['obstacle_layer']['filtered_mark']['topic'] == (
        '/safety/filtered_obstacle_points')
    assert global_map['obstacle_layer']['filtered_mark']['clearing'] is True
    assert global_map['obstacle_layer']['filtered_mark']['marking'] is True
    for layer in (local['voxel_layer'], global_map['obstacle_layer']):
        for source_name in ('raw_clear', 'filtered_mark'):
            source = layer[source_name]
            assert source['observation_persistence'] == 0.0
            assert 0.0 < source['expected_update_rate'] <= 0.5
    assert 'static_layer' not in local['plugins']
    assert 'static_layer' not in global_map['plugins']


def test_recoveries_cannot_move_the_robot():
    config = _params()
    recoveries = config['recoveries_server']['ros__parameters']

    assert recoveries['recovery_plugins'] == ['wait']
    assert set(recoveries) >= {'wait'}
    assert 'spin' not in recoveries
    assert 'back_up' not in recoveries


def test_gate_is_the_only_final_cmd_vel_publisher():
    gate = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'cmd_vel_gate_nav2.yaml').read_text()
    )['cmd_vel_gate']['ros__parameters']

    assert gate['input_topic'] == '/nav2/cmd_vel_safe'
    assert gate['output_topic'] == '/cmd_vel'
    assert gate['max_linear_x'] == 0.15
    assert gate['max_angular_z'] == 0.50


def test_semantic_supervisor_defaults_to_shadow_and_fresh_inputs():
    params = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'semantic_navigation.yaml').read_text()
    )['semantic_navigation_supervisor']['ros__parameters']

    assert params['runtime_mode'] == 'SEMANTIC_SHADOW'
    assert params['semantic_execution_enabled'] is False
    assert params['navigation_frame'] == 'odom'
    assert params['confirmation_snapshots'] >= 2
    assert params['static_target_mode'] is True
    assert params['static_target_position_reacquisition_enabled'] is True
    assert 0.0 < params['static_target_reacquisition_radius_m'] <= 0.50
    assert 0.0 < params['target_dropout_grace_sec'] <= 1.0
    assert params['maximum_target_age_sec'] <= 4.0
    assert params['maximum_goal_age_sec'] <= 0.5
    assert params['maximum_diagnostics_age_sec'] <= 0.5
    assert params['maximum_odom_age_sec'] <= 0.25
