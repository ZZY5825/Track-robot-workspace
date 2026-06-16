from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    node = Node(
        package='track_robot_perception',
        executable='zed_rfdetr_small_node',
        name='zed_rfdetr_small_node',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'output_image_topic': LaunchConfiguration('output_image_topic'),
            'output_text_topic': LaunchConfiguration('output_text_topic'),
            'score_threshold': LaunchConfiguration('score_threshold'),
            'device': LaunchConfiguration('device'),
            'weights_path': LaunchConfiguration('weights_path'),
            'run_every_n_frames': LaunchConfiguration('run_every_n_frames'),
            'max_detections': LaunchConfiguration('max_detections'),
            'publish_annotated_image':
                LaunchConfiguration('publish_annotated_image'),
            'publish_text': LaunchConfiguration('publish_text'),
            'debug_timing': LaunchConfiguration('debug_timing'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument(
            'output_image_topic',
            default_value='/rfdetr/annotated_image'),
        DeclareLaunchArgument(
            'output_text_topic',
            default_value='/rfdetr/detections_text'),
        DeclareLaunchArgument('score_threshold', default_value='0.5'),
        DeclareLaunchArgument('device', default_value='cuda'),
        DeclareLaunchArgument('weights_path', default_value=''),
        DeclareLaunchArgument('run_every_n_frames', default_value='3'),
        DeclareLaunchArgument('max_detections', default_value='30'),
        DeclareLaunchArgument(
            'publish_annotated_image', default_value='true'),
        DeclareLaunchArgument('publish_text', default_value='true'),
        DeclareLaunchArgument('debug_timing', default_value='true'),
        node,
    ])
