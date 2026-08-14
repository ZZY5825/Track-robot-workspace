from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_stage2d_keeps_bounded_safe_off_default_configuration():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'semantic_memory.yaml').read_text())
    parameters = config['semantic_memory']['ros__parameters']

    assert parameters['enabled'] is True
    assert parameters['localization_topic'] == '/semantic_memory/localization_state'
    assert parameters['observations_topic'] == '/semantic_memory/observations'
    assert parameters['lidar_tracklets_topic'] == '/semantic_memory/lidar_tracklets'
    assert parameters['active_objects_topic'] == '/semantic_memory/active_objects'
    assert parameters['events_topic'] == '/semantic_memory/events'
    assert parameters['association_debug_topic'] == '/semantic_memory/association_debug'
    assert parameters['diagnostics_topic'] == '/semantic_memory/diagnostics'
    assert parameters['localization_queue_depth'] == 1
    assert parameters['observation_queue_depth'] == 1
    assert parameters['lidar_queue_depth'] == 1
    assert parameters['publish_active_objects'] is True
    assert parameters['publish_events'] is True
    assert parameters['publish_association_debug'] is True
    assert parameters['association_shadow_mode'] is True
    assert parameters['lidar_memory_updates_enabled'] is True
    assert parameters['camera_attachment_enabled'] is False
    assert parameters['camera_only_memory_enabled'] is False
    assert parameters['appearance_memory_enabled'] is True
    assert parameters['appearance_maximum_prototypes'] == 4
    assert parameters['reidentification_shadow_mode'] is True
    assert parameters['reidentification_mutation_enabled'] is False
    assert parameters['reidentification_calibration_status'] == 'uncalibrated'
    assert parameters['reidentification_confirmation_frames'] == 3
    assert parameters['association_calibration_status'] == 'calibrated'
    assert parameters['association_match_threshold'] == 0.6303095604656801
    assert parameters['association_ambiguity_margin'] == 0.058443876599150624
    assert parameters['association_confirmation_frames'] == 3
    assert 1 <= parameters['association_detach_after_misses'] <= 16
    assert parameters['association_cooldown_frames'] >= 1
    assert parameters['camera_info_topic'] == '/zed/zed_node/left/camera_info'
    assert parameters['camera_calibration_id'] == 'zed_left_rectified_v1'
    assert 1 <= parameters['max_association_debug_pairs_per_batch'] <= 1024
    assert 1 <= parameters['association_lidar_buffer_depth'] <= 512
    assert parameters['association_lidar_buffer_max_age_sec'] >= 1.0
    assert parameters['initial_memory_epoch_id'] == 0
    assert parameters['max_objects'] == 256
    assert parameters['max_history'] == 16
    assert 0.0 < parameters['tf_lookup_timeout_sec'] <= 0.1
    assert 0.0 < parameters['localization_state_timeout_sec'] <= 1.0
    assert parameters['tasks_topic'] == '/semantic_memory/tasks'
    assert parameters['task_queue_depth'] == 1
    assert parameters['best_candidate_topic'] == \
        '/semantic_memory/best_candidate'
    assert parameters['diagnostic_ranking_topic'] == \
        '/semantic_memory/diagnostic_ranking'
    assert parameters['publish_best_candidate'] is True
    assert parameters['publish_diagnostic_ranking'] is True
    assert 'task_appearance_weight' not in parameters
    assert parameters['task_grounding_weight'] == 0.70
    assert parameters['task_stability_weight'] == 0.10
    assert parameters['task_support_weight'] == 0.10
    assert parameters['task_semantic_weight'] == 0.10
    assert parameters['task_grounding_maximum_age_sec'] == 1.0
    assert parameters['best_candidate_threshold_calibrated'] is False
    assert parameters['best_candidate_minimum_relevance'] == 1.0

    visualizer = config['semantic_memory_visualizer']['ros__parameters']
    assert visualizer['enabled'] is True
    assert visualizer['active_objects_topic'] == '/semantic_memory/active_objects'
    assert visualizer['best_candidate_topic'] == (
        '/semantic_memory/best_candidate')
    assert visualizer['markers_topic'] == '/semantic_memory/markers'
    assert visualizer['max_objects'] == 256


def test_phase2_launch_starts_only_dedicated_memory_components():
    source = (
        PACKAGE_ROOT / 'launch' / 'semantic_memory_phase2.launch.py'
    ).read_text()

    assert "package='track_robot_semantic_memory'" in source
    assert "executable='semantic_memory_node'" in source
    assert "executable='semantic_memory_visualizer_node'" in source
    assert 'semantic_memory.yaml' in source
    assert 'human' not in source.lower()
    assert 'track_robot_bringup' not in source


def test_phase123_profile_requires_explicit_degraded_attachment_flags():
    profile = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'phase123_test.yaml').read_text())
    parameters = profile['semantic_memory']['ros__parameters']
    launch = (
        PACKAGE_ROOT / 'launch' / 'semantic_memory_phase2.launch.py'
    ).read_text()

    assert parameters['camera_only_memory_enabled'] is True
    assert parameters['camera_attachment_enabled'] is True
    assert parameters['association_shadow_mode'] is False
    assert parameters['enable_test_camera_attachment'] is True
    assert parameters['allow_degraded_calibration'] is True
    assert parameters['best_candidate_threshold_calibrated'] is False
    assert parameters['reidentification_mutation_enabled'] is False
    assert "DeclareLaunchArgument('enable_test_camera_attachment'" in launch
    assert "DeclareLaunchArgument('allow_degraded_calibration'" in launch
    assert "'enable_test_camera_attachment'" in launch
    assert "'allow_degraded_calibration'" in launch


def test_stage2d_node_has_runtime_attachment_and_source_time_tf_lookup():
    source = (
        PACKAGE_ROOT / 'src' / 'semantic_memory_node.cpp'
    ).read_text()
    calibration_source = (
        PACKAGE_ROOT / 'src' / 'association_calibration.cpp'
    ).read_text()

    assert 'create_publisher<track_robot_interfaces::msg::SemanticObjectArray>' in source
    assert 'create_publisher<track_robot_interfaces::msg::SemanticMemoryEvent>' in source
    assert 'create_publisher<track_robot_interfaces::msg::AssociationDebug>' in source
    assert 'SemanticObservationArray' in source
    assert 'CameraInfo' in source
    assert 'CameraLidarProjector' in source
    assert 'CrossModalAssociator' in source
    assert 'RuntimeAssociationCoordinator' in source
    assert 'next_runtime_association.process(' in source
    assert 'next_memory_core.supplement_visual(' in source
    assert 'next_memory_core.update_camera(' in source
    assert 'camera_only_memory_enabled_' in source
    assert 'next_reidentification.process(' in source
    assert 'next_memory_core.make_reidentification_frame(' in source
    assert 'next_memory_core.reidentify(' in source
    assert 'reidentification_calibration_report' in source
    assert 'association_calibration_report' in source
    assert 'camera_attachment_allowed' in calibration_source
    assert 'selected_parameters' in calibration_source
    assert 'camera_attachment_enabled_' in source
    assert 'camera attachment is deferred' not in source
    assert 'last_measurement_stamp' in source
    assert 'lookupTransform' in source
    assert 'association_lidar_batches_->nearest(' in source
    assert 'MemoryCore' in source
    assert 'latest_lidar_batch_->header.stamp' in source
    assert 'score_visual_pairs(' in source
    assert 'evaluation_stamp_ns)' in source
    assert 'last_lidar_source_epoch_id_)' in source


def test_phase4a_disables_direct_lidar_memory_updates():
    profile = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'phase4a_test.yaml').read_text())
    parameters = profile['semantic_memory']['ros__parameters']

    assert parameters['lidar_memory_updates_enabled'] is False


def test_node_applies_input_policy_at_subscription_and_direct_update_boundaries():
    source = (
        PACKAGE_ROOT / 'src' / 'semantic_memory_node.cpp'
    ).read_text()

    subscriptions = source[
        source.index('  void create_input_subscriptions()'):
        source.index('  void on_camera_info(')
    ]
    subscription_gate = 'if (input_policy_.requires_lidar_subscription())'
    subscription_create = 'lidar_subscription_ = create_subscription<'
    assert subscription_gate in subscriptions
    assert subscriptions.index(subscription_gate) < subscriptions.index(
        subscription_create)

    on_lidar = source[
        source.index('  void on_lidar('):
        source.index('  std::optional<VisualAssociationKey>')
    ]
    direct_update_gate = (
        'if (!input_policy_.allows_direct_lidar_memory_update())')
    assert direct_update_gate in on_lidar
    assert on_lidar.index(direct_update_gate) < on_lidar.index(
        'if (!memory_core_)')
    assert on_lidar.index(direct_update_gate) < on_lidar.index(
        'next_memory_core.update(')


def test_node_delegates_static_profile_authorization_to_input_policy():
    source = (
        PACKAGE_ROOT / 'src' / 'semantic_memory_node.cpp'
    ).read_text()
    static_guard = source[
        source.index('    if (static_target_profile_) {'):
        source.index('    task_relevance_config_.normalization_tolerance')
    ]

    assert 'input_policy_.validate_static_target_profile();' in static_guard
    assert "maximum_static_budget_ns = 4'000'000'000LL" in static_guard


def test_stage2e_runtime_fixture_covers_bounded_leave_and_reentry_cleanup():
    runtime = (PACKAGE_ROOT / 'test' / 'test_ros_runtime.py').read_text()

    assert 'test_stage2e_leave_and_reentry_preserves_one_global_identity' in runtime
    assert 'EVENT_REIDENTIFIED' in runtime
    assert 'reidentification_mutation_enabled' in runtime
    assert 'finally:' in runtime
    assert 'process.terminate()' in runtime


def test_stage2f_node_exposes_bounded_fail_closed_task_services():
    source = (
        PACKAGE_ROOT / 'src' / 'semantic_memory_node.cpp'
    ).read_text()

    assert 'SemanticTask' in source
    assert 'RuntimeTaskServiceCoordinator' in source
    assert 'GetSemanticObject' in source
    assert 'QuerySemanticObjects' in source
    assert 'MarkSemanticObjectInspected' in source
    assert 'ResetSemanticMemory' in source
    assert 'create_service<' in source
    assert 'best_candidate_publisher_' in source
    assert 'transient_local()' in source
    assert 'semantic_object_from_runtime_view' in source
    assert 'best_candidate_threshold_calibrated' in source
    assert 'diagnostic_ranking_publisher_' in source
    assert 'phase3_diagnostic_ranking/1.0.0' in source
    assert '"UNCALIBRATED"' in source
    assert 'YOLO_WORLD_GROUNDING' in source

    for name in (
            'semantic_memory.yaml',
            'phase2_association_baseline.yaml',
            'phase2_camera_attachment.yaml'):
        parameters = yaml.safe_load(
            (PACKAGE_ROOT / 'config' / name).read_text()
        )['semantic_memory']['ros__parameters']
        assert parameters['tasks_topic'] == '/semantic_memory/tasks'
        assert parameters['task_queue_depth'] == 1
        assert parameters['best_candidate_topic'] == \
            '/semantic_memory/best_candidate'
        assert parameters['diagnostic_ranking_topic'] == \
            '/semantic_memory/diagnostic_ranking'
        assert parameters['publish_best_candidate'] is True
        assert parameters['publish_diagnostic_ranking'] is True
        assert 'task_appearance_weight' not in parameters
        assert parameters['task_grounding_weight'] == 0.70
        assert parameters['task_stability_weight'] == 0.10
        assert parameters['task_support_weight'] == 0.10
        assert parameters['task_semantic_weight'] == 0.10
        assert parameters['task_grounding_maximum_age_sec'] == 1.0
        assert parameters['best_candidate_threshold_calibrated'] is False
        assert parameters['best_candidate_minimum_relevance'] == 1.0


def test_calibrated_baseline_remains_shadow_only_until_stage2d():
    baseline = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'phase2_association_baseline.yaml').read_text())
    parameters = baseline['semantic_memory']['ros__parameters']

    assert parameters['association_shadow_mode'] is True
    assert parameters['camera_attachment_enabled'] is False
    assert parameters['reidentification_shadow_mode'] is True
    assert parameters['reidentification_mutation_enabled'] is False
    assert parameters['association_calibration_status'] == 'calibrated'
    assert parameters['association_maximum_size_ratio'] == 40.0
    assert parameters['association_match_threshold'] > 0.0
    assert parameters['association_ambiguity_margin'] > 0.0
    weights = [
        value for key, value in parameters.items()
        if key.startswith('association_weight_')]
    assert abs(sum(weights) - 1.0) < 1.0e-12


def test_stage2d_calibrated_overlay_enables_non_shadow_attachment():
    overlay = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'phase2_camera_attachment.yaml').read_text())
    parameters = overlay['semantic_memory']['ros__parameters']

    assert parameters['association_shadow_mode'] is False
    assert parameters['camera_attachment_enabled'] is True
    assert parameters['reidentification_shadow_mode'] is True
    assert parameters['reidentification_mutation_enabled'] is False
    assert parameters['association_calibration_status'] == 'calibrated'
    assert parameters['association_match_threshold'] == 0.6303095604656801
    assert parameters['association_ambiguity_margin'] == 0.058443876599150624
    assert parameters['association_confirmation_frames'] == 3
    assert parameters['association_detach_after_misses'] == 2
    assert parameters['association_cooldown_frames'] == 2
    assert parameters['association_calibration_report'].startswith('reports/')


def test_stage2d_installs_and_resolves_the_calibration_report_from_package_share():
    cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text()
    node = (PACKAGE_ROOT / 'src' / 'semantic_memory_node.cpp').read_text()

    assert 'phase2_association_calibration_2026-07-16.json' in cmake
    assert 'DESTINATION share/${PROJECT_NAME}/reports' in cmake
    assert 'get_package_share_directory' in node


def test_visualizer_consumes_only_active_and_fail_closed_winner_snapshots():
    source = (
        PACKAGE_ROOT / 'src' / 'semantic_memory_visualizer_node.cpp'
    ).read_text()

    assert 'SemanticObjectArray' in source
    assert 'MarkerArray' in source
    assert 'best_candidate_topic' in source
    assert '/semantic_memory/best_candidate' in source
    assert 'SemanticLocalizationState' not in source
    assert 'SemanticLidarTrackletArray' not in source
