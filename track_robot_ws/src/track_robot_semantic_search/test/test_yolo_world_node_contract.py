import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE = (
    PACKAGE_ROOT / 'track_robot_semantic_search' /
    'yolo_world_perception_node.py')


def test_yolo_world_node_has_only_passive_publishers():
    source = NODE.read_text(encoding='utf-8')
    tree = ast.parse(source)
    publisher_types = []
    for call in ast.walk(tree):
        if (
                isinstance(call, ast.Call) and
                isinstance(call.func, ast.Attribute) and
                call.func.attr == 'create_publisher' and call.args and
                isinstance(call.args[0], ast.Name)):
            publisher_types.append(call.args[0].id)

    assert sorted(publisher_types) == [
        'SemanticObservationArray',
        'SemanticRegionArray',
        'SemanticTask',
        'String',
    ]
    for forbidden in ('cmd_vel', 'SearchMotionIntent', 'Twist'):
        assert forbidden not in source


def test_yolo_world_worker_is_packaged_and_launch_is_default_off():
    setup = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')
    launch = (
        PACKAGE_ROOT / 'launch' /
        'semantic_search_yolo_world.launch.py').read_text(encoding='utf-8')

    assert 'semantic_search_yolo_world_perception' in setup
    assert 'yolo_world_perception_node:main' in setup
    assert "DeclareLaunchArgument('start_perception', default_value='false')" in (
        launch)


def test_yolo_world_launch_preloads_libgomp_before_python_imports():
    launch = (
        PACKAGE_ROOT / 'launch' /
        'semantic_search_yolo_world.launch.py').read_text(encoding='utf-8')

    assert "additional_env={" in launch
    assert "'LD_PRELOAD': '/lib/aarch64-linux-gnu/libgomp.so.1'" in launch


def test_yolo_world_node_declares_max_detections_only_once():
    source = NODE.read_text(encoding='utf-8')

    assert source.count("'max_detections'") == 1
    assert 'self._max_detections' in source


def test_yolo_world_image_subscription_uses_sensor_data_qos():
    source = NODE.read_text(encoding='utf-8')

    assert 'from rclpy.qos import qos_profile_sensor_data' in source
    assert 'self._image_callback,\n            qos_profile_sensor_data)' in source
