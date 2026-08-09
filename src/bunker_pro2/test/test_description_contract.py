import ast
from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_upstream_robot_assets_are_complete():
    urdf_path = PACKAGE_ROOT / 'urdf' / 'bunker_pro2.urdf'
    mesh_path = PACKAGE_ROOT / 'meshes' / 'base_link.STL'

    assert urdf_path.is_file()
    assert mesh_path.is_file()
    assert mesh_path.stat().st_size > 1024

    robot = ET.parse(str(urdf_path)).getroot()
    assert robot.tag == 'robot'
    assert robot.attrib['name'] == 'bunker_pro2'
    assert [link.attrib['name'] for link in robot.findall('link')] == [
        'base_link',
        'sensor_station_link',
        'camera_link',
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


def test_launch_starts_state_publisher_and_rviz():
    launch_path = PACKAGE_ROOT / 'launch' / 'display.launch.py'
    source = launch_path.read_text(encoding='utf-8')
    ast.parse(source)

    assert "get_package_share_directory('bunker_pro2')" in source
    assert "package='robot_state_publisher'" in source
    assert "executable='robot_state_publisher'" in source
    assert "'robot_description': robot_description" in source
    assert "package='rviz2'" in source
    assert "executable='rviz2'" in source
    assert "'bunker_pro2.rviz'" in source


def test_launch_publishes_world_to_base_link_transform():
    launch_path = PACKAGE_ROOT / 'launch' / 'display.launch.py'
    source = launch_path.read_text(encoding='utf-8')

    assert "package='tf2_ros'" in source
    assert "executable='static_transform_publisher'" in source
    assert "'world', 'base_link'" in source

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


def test_camera_link_is_fixed_to_sensor_station():
    robot = ET.parse(str(PACKAGE_ROOT / 'urdf' / 'bunker_pro2.urdf')).getroot()
    camera_link = robot.find("./link[@name='camera_link']")
    assert camera_link is not None
    assert len(camera_link) == 0

    joint = robot.find("./joint[@name='sensor_station_camera_joint']")
    assert joint is not None
    assert joint.attrib['type'] == 'fixed'
    assert joint.find('parent').attrib['link'] == 'sensor_station_link'
    assert joint.find('child').attrib['link'] == 'camera_link'
    assert joint.find('origin').attrib == {
        'xyz': '-0.2212 0.318 0',
        'rpy': '1.57079632679 0 3.14159265359',
    }


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
