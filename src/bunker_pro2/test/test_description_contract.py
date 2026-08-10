import ast
import copy
from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PIPER_DESCRIPTION_ROOT = PACKAGE_ROOT.parent / 'piper_description'

BUNKER_LINK_NAMES = {
    'robot_bottom',
    'base_link',
    'sensor_station_link',
    'camera_mount_link',
    'zed_camera_link',
    'lidar_link',
}
BUNKER_JOINT_NAMES = {
    'robot_bottom_to_base_link',
    'sensor_station_joint',
    'sensor_station_camera_mount_joint',
    'camera_mount_to_zed_camera_joint',
    'sensor_station_lidar_joint',
    'base_to_arm_base_joint',
}


def test_upstream_robot_assets_are_complete():
    urdf_path = PACKAGE_ROOT / 'urdf' / 'bunker_pro2.urdf'
    mesh_path = PACKAGE_ROOT / 'meshes' / 'base_link.STL'

    assert urdf_path.is_file()
    assert mesh_path.is_file()
    assert mesh_path.stat().st_size > 1024

    robot = ET.parse(str(urdf_path)).getroot()
    assert robot.tag == 'robot'
    assert robot.attrib['name'] == 'bunker_pro2'
    assert [link.attrib['name'] for link in robot.findall('link')][:6] == [
        'robot_bottom',
        'base_link',
        'sensor_station_link',
        'camera_mount_link',
        'zed_camera_link',
        'lidar_link',
    ]
    mesh = robot.find('./link/visual/geometry/mesh')
    assert mesh is not None
    assert mesh.attrib['filename'] == (
        'package://bunker_pro2/meshes/base_link.STL'
    )


def test_ros2_package_metadata_and_install_contract():
    package = ET.parse(str(PACKAGE_ROOT / 'package.xml')).getroot()
    assert package.findtext('name') == 'bunker_pro2'
    assert package.findtext('./export/build_type') == 'ament_cmake'

    cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
    for directory in ('launch', 'meshes', 'rviz', 'urdf'):
        assert directory in cmake
    assert 'ament_package()' in cmake


def test_package_uses_workspace_maintainer_identity():
    package = ET.parse(str(PACKAGE_ROOT / 'package.xml')).getroot()
    maintainer = package.find('maintainer')

    assert maintainer is not None
    assert maintainer.attrib['email'] == 'track-robot@example.com'


def test_cmake_disables_incompatible_pytest_plugin_autoload_during_discovery():
    cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')

    assert 'set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)' in cmake


def test_description_launch_starts_only_state_publisher():
    launch_path = PACKAGE_ROOT / 'launch' / 'description.launch.py'
    source = launch_path.read_text(encoding='utf-8')
    ast.parse(source)

    assert "get_package_share_directory('bunker_pro2')" in source
    assert "package='robot_state_publisher'" in source
    assert "executable='robot_state_publisher'" in source
    assert "'robot_description': robot_description" in source
    assert "package='rviz2'" not in source
    assert "package='tf2_ros'" not in source


def test_display_launch_composes_description_and_rviz():
    launch_path = PACKAGE_ROOT / 'launch' / 'display.launch.py'
    source = launch_path.read_text(encoding='utf-8')
    ast.parse(source)

    assert 'IncludeLaunchDescription' in source
    assert 'FindPackageShare' in source
    assert "'bunker_pro2'" in source
    assert "'description.launch.py'" in source
    assert "package='rviz2'" in source
    assert "executable='rviz2'" in source
    assert "'bunker_pro2.rviz'" in source


def test_launch_publishes_world_to_robot_bottom_transform():
    launch_path = PACKAGE_ROOT / 'launch' / 'display.launch.py'
    source = launch_path.read_text(encoding='utf-8')

    assert "package='tf2_ros'" in source
    assert "executable='static_transform_publisher'" in source
    assert "'world', 'robot_bottom'" in source

    package = ET.parse(str(PACKAGE_ROOT / 'package.xml')).getroot()
    assert 'tf2_ros' in {
        dependency.text for dependency in package.findall('exec_depend')
    }


def test_rviz_uses_published_world_frame_and_robot_description():
    config = (
        PACKAGE_ROOT / 'rviz' / 'bunker_pro2.rviz'
    ).read_text(encoding='utf-8')
    assert 'Fixed Frame: world' in config
    assert 'Target Frame: world' in config
    assert 'Class: rviz_default_plugins/RobotModel' in config
    assert 'Description Source: Topic' in config
    assert 'Description File: ""' in config
    assert 'Durability Policy: Transient Local' in config
    assert 'Value: /robot_description' in config
    assert 'Robot Description:' not in config


def test_sensor_station_visual_is_scaled_and_centered():
    mesh_path = PACKAGE_ROOT / 'meshes' / 'FullCase.STL'
    assert mesh_path.is_file()
    assert mesh_path.stat().st_size > 1024

    robot = ET.parse(str(PACKAGE_ROOT / 'urdf' / 'bunker_pro2.urdf')).getroot()
    link = robot.find("./link[@name='sensor_station_link']")
    assert link is not None
    origin = link.find('./visual/origin')
    assert origin.attrib == {
        'xyz': '-0.26875 0 -0.2335',
        'rpy': '0 0 0',
    }
    mesh = link.find('./visual/geometry/mesh')
    assert mesh.attrib['filename'] == (
        'package://bunker_pro2/meshes/FullCase.STL'
    )
    assert mesh.attrib['scale'] == '0.001 0.001 0.001'


def test_robot_bottom_is_empty_and_fixed_below_base_link():
    robot = ET.parse(str(PACKAGE_ROOT / 'urdf' / 'bunker_pro2.urdf')).getroot()
    robot_bottom = robot.find("./link[@name='robot_bottom']")
    assert robot_bottom is not None
    assert len(robot_bottom) == 0

    joint = robot.find("./joint[@name='robot_bottom_to_base_link']")
    assert joint is not None
    assert joint.attrib['type'] == 'fixed'
    assert joint.find('parent').attrib['link'] == 'robot_bottom'
    assert joint.find('child').attrib['link'] == 'base_link'
    assert joint.find('origin').attrib == {
        'xyz': '0 0 0.45',
        'rpy': '0 0 0',
    }


def test_sensor_station_is_fixed_to_top_rail_midpoint():
    robot = ET.parse(str(PACKAGE_ROOT / 'urdf' / 'bunker_pro2.urdf')).getroot()
    joint = robot.find("./joint[@name='sensor_station_joint']")
    assert joint is not None
    assert joint.attrib['type'] == 'fixed'
    assert joint.find('parent').attrib['link'] == 'base_link'
    assert joint.find('child').attrib['link'] == 'sensor_station_link'
    assert joint.find('origin').attrib == {
        'xyz': '-0.2125 0 0.016',
        'rpy': '1.57079632679 0 3.14159265359',
    }


def _joint(robot, name, parent, child, xyz, rpy):
    joint = robot.find("./joint[@name='{}']".format(name))
    assert joint is not None
    assert joint.attrib['type'] == 'fixed'
    assert joint.find('parent').attrib['link'] == parent
    assert joint.find('child').attrib['link'] == child
    assert joint.find('origin').attrib == {'xyz': xyz, 'rpy': rpy}


def _xml_signature(element):
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        (element.text or '').strip(),
        tuple(_xml_signature(child) for child in element),
    )


def _expected_integrated_piper_elements(tag):
    source = ET.parse(str(
        PIPER_DESCRIPTION_ROOT / 'urdf' / 'piper_description.xacro'
    )).getroot()
    expected = {}
    for source_element in source.findall(tag):
        name = source_element.attrib['name']
        if (tag == 'link' and name == 'world') or (
                tag == 'joint' and name == 'fixed_base_joint'):
            continue
        element = copy.deepcopy(source_element)
        if tag == 'link' and name == 'base_link':
            element.attrib['name'] = 'arm_base_link'
        for reference in element.findall('parent') + element.findall('child'):
            if reference.attrib['link'] == 'base_link':
                reference.attrib['link'] = 'arm_base_link'
        expected[element.attrib['name']] = element
    return expected


def test_piper_arm_is_a_front_rail_sibling_of_sensor_station():
    robot = ET.parse(str(PACKAGE_ROOT / 'urdf' / 'bunker_pro2.urdf')).getroot()
    links = [link.attrib['name'] for link in robot.findall('link')]
    assert links.count('base_link') == 1
    assert 'arm_base_link' in links
    _joint(
        robot, 'base_to_arm_base_joint', 'base_link', 'arm_base_link',
        '0.39 0 0.016', '0 0 0')


def test_combined_description_has_one_root():
    robot = ET.parse(str(PACKAGE_ROOT / 'urdf' / 'bunker_pro2.urdf')).getroot()
    children = {
        joint.find('child').attrib['link'] for joint in robot.findall('joint')
    }
    roots = {
        link.attrib['name'] for link in robot.findall('link')
    } - children
    assert roots == {'robot_bottom'}


def test_integrated_piper_subtree_matches_hash_locked_source():
    robot = ET.parse(str(PACKAGE_ROOT / 'urdf' / 'bunker_pro2.urdf')).getroot()
    integrated_links = {
        link.attrib['name']: link for link in robot.findall('link')
        if link.attrib['name'] not in BUNKER_LINK_NAMES
    }
    integrated_joints = {
        joint.attrib['name']: joint for joint in robot.findall('joint')
        if joint.attrib['name'] not in BUNKER_JOINT_NAMES
    }
    expected_links = _expected_integrated_piper_elements('link')
    expected_joints = _expected_integrated_piper_elements('joint')

    assert set(integrated_links) == set(expected_links)
    assert set(integrated_joints) == set(expected_joints)
    for name, expected in expected_links.items():
        assert _xml_signature(integrated_links[name]) == _xml_signature(expected)
    for name, expected in expected_joints.items():
        assert _xml_signature(integrated_joints[name]) == _xml_signature(expected)


def test_camera_mount_is_the_stereo_center_and_connects_vendor_root_directly():
    robot = ET.parse(str(PACKAGE_ROOT / 'urdf' / 'bunker_pro2.urdf')).getroot()
    links = {link.attrib['name'] for link in robot.findall('link')}
    assert {'camera_mount_link', 'zed_camera_link'} <= links
    assert 'camera_link' not in links

    _joint(
        robot, 'sensor_station_camera_mount_joint',
        'sensor_station_link', 'camera_mount_link',
        '-0.2212 0.318 0', '1.57079632679 0 3.14159265359')
    _joint(
        robot, 'camera_mount_to_zed_camera_joint',
        'camera_mount_link', 'zed_camera_link', '0 0 -0.015', '0 0 0')

    assert robot.find("./joint[@name='camera_mount_to_camera_joint']") is None
    assert robot.find("./joint[@name='camera_to_zed_camera_joint']") is None


def test_camera_reference_links_are_empty():
    robot = ET.parse(str(PACKAGE_ROOT / 'urdf' / 'bunker_pro2.urdf')).getroot()
    for name in ('camera_mount_link', 'zed_camera_link'):
        link = robot.find("./link[@name='{}']".format(name))
        assert link is not None
        assert len(link) == 0


def test_lidar_link_is_fixed_above_sensor_station():
    robot = ET.parse(str(PACKAGE_ROOT / 'urdf' / 'bunker_pro2.urdf')).getroot()
    lidar_link = robot.find("./link[@name='lidar_link']")
    assert lidar_link is not None
    assert len(lidar_link) == 0

    joint = robot.find("./joint[@name='sensor_station_lidar_joint']")
    assert joint is not None
    assert joint.attrib['type'] == 'fixed'
    assert joint.find('parent').attrib['link'] == 'sensor_station_link'
    assert joint.find('child').attrib['link'] == 'lidar_link'
    assert joint.find('origin').attrib == {
        'xyz': '0 0.4 0',
        'rpy': '1.57079632679 0 3.14159265359',
    }
