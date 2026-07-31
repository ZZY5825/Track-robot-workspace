import ast
from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / 'launch' / 'semantic_search_phase4a.launch.py'
MEMORY_CONFIG = (
    PACKAGE.parent / 'track_robot_semantic_memory'
    / 'config' / 'phase4a_test.yaml')
SEARCH_CONFIG = (
    PACKAGE.parents[1] / 'track_robot_semantic_search'
    / 'config' / 'semantic_search_phase4a.yaml')


def _source(path):
    assert path.is_file(), 'required Phase 4A file is missing: {}'.format(path)
    return path.read_text(encoding='utf-8')


def test_phase4a_launch_is_stationary_and_does_not_import_motion_interfaces():
    source = _source(LAUNCH)

    assert "'start_base': 'false'" in source
    assert "'start_imu': 'false'" in source
    assert "executable='local_obstacle_map_node'" in source
    assert 'semantic_search_phase4a_fixed_base' in source
    assert 'semantic_search_phase4a_selector' in source
    assert 'semantic_search_phase4a_advisor' in source
    assert 'semantic_search_phase4_planner' in source
    for forbidden in (
            'bunker_base',
            'cmd_vel',
            'geometry_msgs.msg.Twist',
            'local_trajectory_planner_node',
            'motion_safety_supervisor_node',
            'ActionClient',
            '/odom',
            '/imu'):
        assert forbidden not in source


def test_phase4a_launch_composes_real_phase1_phase2_and_rviz():
    source = _source(LAUNCH)

    for expected in (
            'semantic_search_sensors.launch.py',
            'semantic_search_yolo_world.launch.py',
            'semantic_search_spatial_observation',
            'semantic_memory_lidar_tracklets.launch.py',
            'semantic_memory_phase2.launch.py',
            'semantic_search_phase4.rviz',
            'phase4a_test.yaml',
            'semantic_search_phase4a.yaml'):
        assert expected in source
    assert "'start_camera': 'true'" in source
    assert "'start_lidar': 'true'" in source
    assert "'enable_test_camera_attachment': 'true'" in source
    assert "'allow_degraded_calibration': 'true'" in source
    assert "'start_visualizer': 'false'" in source
    assert "'camera_depth_mode': 'PERFORMANCE'" in source


def test_phase4a_exposes_optional_dino_and_keeps_tested_baseline_default():
    source = _source(LAUNCH)

    assert "'dino_enabled': LaunchConfiguration('dino_enabled')" in source
    assert "'dino_enabled', default_value='false'" in source


def test_phase4a_configs_share_the_fixed_base_contract_and_fail_closed():
    memory = yaml.safe_load(_source(MEMORY_CONFIG))['semantic_memory'][
        'ros__parameters']
    search = yaml.safe_load(_source(SEARCH_CONFIG))
    fixed = search['phase4a_fixed_base_session']['ros__parameters']
    selector = search['phase4a_target_selector']['ros__parameters']
    planner = search['phase4_approach_planner']['ros__parameters']
    advisor = search['phase4a_advisor']['ros__parameters']

    fixed_topic = '/semantic_search/phase4a/localization_state'
    target_topic = '/semantic_search/phase4a/selected_target'
    assert fixed['state_topic'] == fixed_topic
    assert memory['localization_topic'] == fixed_topic
    assert planner['localization_topic'] == fixed_topic
    assert fixed['frame_id'] == 'base_link'
    assert planner['selected_target_topic'] == target_topic
    assert selector['selected_target_topic'] == target_topic
    assert selector['depth_topic'] == (
        '/zed/zed_node/depth/depth_registered')
    enricher = search['semantic_depth_enricher']['ros__parameters']
    assert enricher['input_observations_topic'] == (
        '/semantic_memory/observations')
    assert enricher['output_observations_topic'] == (
        '/semantic_memory/spatial_observations')
    assert memory['observations_topic'] == (
        '/semantic_memory/spatial_observations')
    assert selector['spatial_objects_topic'] == (
        '/semantic_search/phase4a/spatial_objects')
    assert selector['maximum_depth_age_sec'] <= 0.5
    assert memory['best_candidate_threshold_calibrated'] is True
    assert memory['best_candidate_minimum_relevance'] == 0.26
    assert memory['publish_diagnostic_ranking'] is True
    assert memory['association_max_source_time_delta_sec'] == 0.20
    weights = [
        memory[f'association_weight_{name}']
        for name in (
            'position_consistency',
            'projected_centroid',
            'inside_fraction',
            'projected_iou',
            'visual_cosine',
            'extent_consistency',
            'point_count_consistency',
            'motion_continuity',
            'previous_association',
            'detector_confidence',
            'geometry_confidence',
            'sensor_confidence',
        )
    ]
    assert all(0.0 <= weight <= 1.0 for weight in weights)
    assert abs(sum(weights) - 1.0) < 1e-9
    assert 0.15 <= memory['association_match_threshold'] <= 0.20
    assert memory['association_ambiguity_margin'] >= 0.03
    assert memory['association_confirmation_frames'] >= 3
    assert selector['confirmation_snapshots'] >= 3
    assert selector['minimum_relevance'] == 0.26
    assert planner['minimum_target_relevance'] == 0.24
    # The tested Phase 4B workflow uses an explicit, bounded static-target
    # profile because semantic inference can legitimately take up to 3.51 s.
    # Dynamic/default profiles must keep the one-second policy; the static
    # profile is still fail-closed at four seconds.
    assert memory['static_target_profile'] is True
    assert 3.51 <= selector['maximum_age_sec'] <= 4.0
    assert 3.51 <= planner['maximum_target_age_sec'] <= 4.0
    assert selector['minimum_relevance'] == 0.26
    assert selector['retained_minimum_relevance'] == 0.24
    assert planner['planning_only'] is True
    assert planner['minimum_target_relevance'] == 0.24
    assert (
        planner['minimum_target_relevance']
        == selector['retained_minimum_relevance']
    )
    assert planner['unknown_is_obstacle'] is True
    assert planner['maximum_search_expansions'] > 0
    assert planner['enable_path_shortcutting'] is True
    assert advisor['advisory_only'] is True


def test_phase4a_launch_is_valid_python():
    ast.parse(_source(LAUNCH))
