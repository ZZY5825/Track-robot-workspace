from pathlib import Path
import xml.etree.ElementTree as ElementTree


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_plugin_exports_the_semantic_search_panel():
    root = ElementTree.parse(
        str(PACKAGE_ROOT / 'plugin_description.xml')).getroot()
    exported = root.find('class')

    assert exported is not None
    assert exported.attrib['name'] == (
        'track_robot_semantic_search_rviz_plugins/SemanticSearchPanel')
    assert exported.attrib['type'] == (
        'track_robot_semantic_search_rviz_plugins::SemanticSearchPanel')
    assert exported.attrib['base_class_type'] == 'rviz_common::Panel'


def test_panel_uses_reference_bound_supervised_and_active_search_apis():
    source = (
        PACKAGE_ROOT / 'src' / 'semantic_search_panel.cpp'
    ).read_text(encoding='utf-8')
    header = (
        PACKAGE_ROOT / 'include'
        / 'track_robot_semantic_search_rviz_plugins'
        / 'semantic_search_panel.hpp'
    ).read_text(encoding='utf-8')

    for topic in (
            '/semantic_search/query',
            '/semantic_search/perception_diagnostics',
            '/semantic_search/regions',
            '/semantic_memory/active_objects',
            '/semantic_memory/best_candidate',
            '/semantic_memory/diagnostic_ranking',
            '/semantic_search/phase4a/selected_target',
            '/semantic_navigation/authorize_approach',
            '/semantic_navigation/cancel_and_disarm',
            '/semantic_navigation/diagnostics'):
        assert topic in source

    for interface in (
            '/semantic_search/search_for_object',
            '/semantic_search/active_search/cancel'):
        assert interface in source

    assert '/semantic_search/active_search/authorize_rotation' not in source

    assert 'SUPERVISED SEMANTIC CONTROL' in source
    assert 'Start Approach' in source
    assert 'Cancel & Disarm' in source
    assert 'Start Finding' in source
    assert 'Stop Finding' in source
    assert 'WAITING_FOR_AUTHORIZATION' not in source
    assert 'selected target is not ready' in source
    assert 'authorization candidate is not correlated' not in source
    assert 'AuthorizeSemanticApproach' in source
    assert 'SearchForObject' in source
    assert 'rclcpp_action' in source
    assert 'approach_request_active_' in source
    assert 'starting approach' in source
    assert 'if (request_active)' in source
    assert 'approach_request_active_ = false' in source
    assert 'safety_arm_pending' not in source
    assert 'approach enabled (supervised)' in source
    assert 'Diagnostic ranking' in source
    assert 'support=' in source
    assert 'query=' in source
    assert 'relevance=' in source
    assert 'Navigation recovery' in source
    assert 'semantic_navigation/supervisor' in source
    assert 'recovery_status_' in header
    assert 'diagnostic_msgs::msg::DiagnosticArray' in header
    assert '~SemanticSearchPanel() override;' in header
    assert 'CallbackLifetime' in header
    assert 'callback_lifetime_' in header
    assert 'session_mutex_' in header
    assert 'weak_lifetime.lock()' in source
    assert 'lifetime->alive = false' in source
    assert 'std::lock_guard<std::mutex> callback_lock(lifetime->mutex)' in source
    assert 'QTimer' not in header
    assert 'kCancellationDeadlineMs' not in source
    assert 'Retry Stop' not in source
    assert 'cancellation_timer_' not in source
    assert 'cancellation_deadline_elapsed' not in source
    assert 'async_cancel_goal' in source
    assert 'handle,' in source
    assert 'set_query_controls_enabled(false)' in source
    assert 'set_query_controls_enabled(true)' in source
    assert 'manual_query_allowed()' in source
    assert 'active search owns the current query' in source
    assert 'active_search_feedback_status' in source
    assert ('translation/approach starts only after Start Approach' in source)
    assert ('Start Finding may initiate bounded in-place rotation; '
            'RC override and E-stop remain authoritative.' in source)

    start_body = source.split(
        'void SemanticSearchPanel::start_approach()', 1)[1].split(
        'void SemanticSearchPanel::cancel_and_disarm()', 1)[0]
    assert 'selected_reference_.has_value()' in start_body
    assert 'best_reference_->same_identity' not in start_body
    for field in (
            'memory_epoch_id', 'global_object_id', 'localization_epoch_id',
            'query_id', 'query_version', 'snapshot_sequence'):
        assert 'request->{} = reference->{};'.format(field, field) in start_body

    result_body = source.split('options.result_callback =', 1)[1].split(
        'try {\n    search_client_->async_send_goal', 1)[0]
    assert 'finish_finding(' in result_body
    assert 'start_approach(' not in result_body

    finding_start_body = source.split(
        'void SemanticSearchPanel::start_finding()', 1)[1].split(
        'void SemanticSearchPanel::stop_finding()', 1)[0]
    assert 'finding_button_->setEnabled(false)' not in finding_start_body
    assert 'finding_button_->setEnabled(true)' in finding_start_body
    assert 'QuerySession::normalize_query' in finding_start_body
    assert finding_start_body.index('QuerySession::normalize_query') < (
        finding_start_body.index('finding_session_.begin()'))

    cancel_body = source.split(
        'void SemanticSearchPanel::stop_finding()', 1)[1].split(
        'void SemanticSearchPanel::render_finding_state(', 1)[0]
    assert 'try {\n    cancel_search_client_->async_send_request' in cancel_body
    assert 'active-search cancellation service send failed' in cancel_body
    assert 'weak_lifetime, generation' in cancel_body
    assert 'finish_finding(generation, tr("search stopped"))' not in cancel_body

    assert 'authorize_rotation_client_' not in header
    assert 'authorize_rotation(' not in header
    assert 'authorize_rotation_service_' not in header

    for forbidden in (
            'cmd_vel',
            'SearchMotionIntent',
            'geometry_msgs',
            'inspection'):
        assert forbidden not in source


def test_phase4b_rviz_hides_unselected_semantic_memory_boxes():
    rviz = (
        PACKAGE_ROOT.parent
        / 'track_robot_bringup'
        / 'rviz'
        / 'semantic_search_phase4b.rviz'
    ).read_text(encoding='utf-8')

    assert '/semantic_memory/markers' not in rviz
    assert '/semantic_search/phase4/markers' in rviz


def test_package_exports_pluginlib_metadata_and_installs_headers():
    cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
    package = ElementTree.parse(
        str(PACKAGE_ROOT / 'package.xml')).getroot()

    assert 'pluginlib_export_plugin_description_file' in cmake
    assert 'CMAKE_AUTOMOC ON' in cmake
    assert 'DESTINATION include' in cmake
    assert 'rclcpp_action' in cmake
    assert 'diagnostic_msgs' in cmake
    assert package.find("depend[.='rclcpp_action']") is not None
    assert package.find("depend[.='diagnostic_msgs']") is not None
    export = package.find('export')
    assert export is not None
    assert export.find('build_type').text == 'ament_cmake'
