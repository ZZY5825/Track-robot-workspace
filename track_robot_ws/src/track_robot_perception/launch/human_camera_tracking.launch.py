from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    tracker_backend = LaunchConfiguration('tracker_backend')
    model_path = LaunchConfiguration('model_path')
    resize_width = LaunchConfiguration('resize_width')
    config_file = LaunchConfiguration('camera_config_file')

    tracker = Node(
        package='track_robot_perception',
        executable='human_image_tracker_node',
        name='human_image_tracker_node',
        output='screen',
        parameters=[config_file, {
            'image_topic': image_topic,
            'tracker_backend': tracker_backend,
            'model_path': model_path,
            'resize_width': resize_width,
            'detections_topic': '/human_tracking/detections',
            'annotated_image_topic': '/human_tracking/annotated_image',
            'debug_topic': '/human_tracking/tracker_debug',
        }],
    )

    gesture = Node(
        package='track_robot_perception',
        executable='gesture_trigger_node',
        name='gesture_trigger_node',
        output='screen',
        parameters=[config_file, {
            'image_topic': image_topic,
            'detections_topic': '/human_tracking/detections',
            'output_topic': '/human_tracking/gesture_state',
            'overlay_topic': '/human_tracking/gesture_overlay',
        }],
    )

    target_lock = Node(
        package='track_robot_perception',
        executable='camera_target_lock_node',
        name='camera_target_lock_node',
        output='screen',
        parameters=[config_file, {
            'image_topic': image_topic,
            'detections_topic': '/human_tracking/detections',
            'gesture_topic': '/human_tracking/gesture_state',
            'output_topic': '/human_tracking/camera_target',
            'overlay_topic': '/human_tracking/target_overlay',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument('tracker_backend', default_value='bytetrack'),
        DeclareLaunchArgument(
            'camera_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_perception'),
                'config',
                'human_tracking.yaml',
            ])),
        DeclareLaunchArgument(
            'model_path',
            default_value='/home/track-robot/track_robot_ws/models/human_tracking/yolov8n-pose.pt'),
        DeclareLaunchArgument('resize_width', default_value='960'),
        tracker,
        gesture,
        target_lock,
    ])
