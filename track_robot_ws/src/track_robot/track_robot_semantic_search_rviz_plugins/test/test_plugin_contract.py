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


def test_panel_uses_reference_bound_supervised_services_and_no_velocity_api():
    source = (
        PACKAGE_ROOT / 'src' / 'semantic_search_panel.cpp'
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
            '/semantic_navigation/cancel_and_disarm'):
        assert topic in source

    assert 'SUPERVISED SEMANTIC APPROACH' in source
    assert 'Start Approach' in source
    assert 'Cancel & Disarm' in source
    assert 'selected target is not ready' in source
    assert 'authorization candidate is not correlated' not in source
    assert 'AuthorizeSemanticApproach' in source
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

    start_body = source.split(
        'void SemanticSearchPanel::start_approach()', 1)[1].split(
        'void SemanticSearchPanel::cancel_and_disarm()', 1)[0]
    assert 'selected_reference_.has_value()' in start_body
    assert 'best_reference_->same_identity' not in start_body

    for forbidden in (
            'cmd_vel',
            'SearchMotionIntent',
            'geometry_msgs',
            'rclcpp_action',
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
    export = package.find('export')
    assert export is not None
    assert export.find('build_type').text == 'ament_cmake'
