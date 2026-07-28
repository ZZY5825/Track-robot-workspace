from pathlib import Path
import xml.etree.ElementTree as ElementTree


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_plugin_exports_the_passive_semantic_search_panel():
    root = ElementTree.parse(
        str(PACKAGE_ROOT / 'plugin_description.xml')).getroot()
    exported = root.find('class')

    assert exported is not None
    assert exported.attrib['name'] == (
        'track_robot_semantic_search_rviz_plugins/SemanticSearchPanel')
    assert exported.attrib['type'] == (
        'track_robot_semantic_search_rviz_plugins::SemanticSearchPanel')
    assert exported.attrib['base_class_type'] == 'rviz_common::Panel'


def test_panel_uses_only_approved_passive_topics_and_no_motion_api():
    source = (
        PACKAGE_ROOT / 'src' / 'semantic_search_panel.cpp'
    ).read_text(encoding='utf-8')

    for topic in (
            '/semantic_search/query',
            '/semantic_search/perception_diagnostics',
            '/semantic_search/regions',
            '/semantic_memory/active_objects',
            '/semantic_memory/best_candidate'):
        assert topic in source

    for forbidden in (
            'cmd_vel',
            'SearchMotionIntent',
            'geometry_msgs',
            'rclcpp_action',
            'reset',
            'inspection'):
        assert forbidden not in source


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
