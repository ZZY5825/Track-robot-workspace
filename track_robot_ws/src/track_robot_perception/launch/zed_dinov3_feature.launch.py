from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    node = Node(
        package='track_robot_perception',
        executable='zed_dinov3_feature_node',
        name='zed_dinov3_feature_node',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'debug_image_topic': LaunchConfiguration('debug_image_topic'),
            'debug_text_topic': LaunchConfiguration('debug_text_topic'),
            'model_name': LaunchConfiguration('model_name'),
            'model_source': LaunchConfiguration('model_source'),
            'device': LaunchConfiguration('device'),
            'input_size': LaunchConfiguration('input_size'),
            'run_every_n_frames': LaunchConfiguration('run_every_n_frames'),
            'save_features': LaunchConfiguration('save_features'),
            'output_dir': LaunchConfiguration('output_dir'),
            'publish_heatmap': LaunchConfiguration('publish_heatmap'),
            'normalize_features': LaunchConfiguration('normalize_features'),
            'max_saved_frames': LaunchConfiguration('max_saved_frames'),
            'debug_timing': LaunchConfiguration('debug_timing'),
            'local_repo': LaunchConfiguration('local_repo'),
            'weights_path': LaunchConfiguration('weights_path'),
            'hf_model_id': LaunchConfiguration('hf_model_id'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic', default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument(
            'debug_image_topic', default_value='/dinov3/debug_image'),
        DeclareLaunchArgument(
            'debug_text_topic', default_value='/dinov3/feature_debug'),
        DeclareLaunchArgument('model_name', default_value='dinov3_vits16plus'),
        DeclareLaunchArgument('model_source', default_value='local_repo'),
        DeclareLaunchArgument('device', default_value='auto'),
        DeclareLaunchArgument('input_size', default_value='512'),
        DeclareLaunchArgument('run_every_n_frames', default_value='5'),
        DeclareLaunchArgument('save_features', default_value='false'),
        DeclareLaunchArgument(
            'output_dir',
            default_value='/home/track-robot/track_robot_ws/dinov3_feature_outputs'),
        DeclareLaunchArgument('publish_heatmap', default_value='true'),
        DeclareLaunchArgument('normalize_features', default_value='true'),
        DeclareLaunchArgument('max_saved_frames', default_value='100'),
        DeclareLaunchArgument('debug_timing', default_value='true'),
        DeclareLaunchArgument(
            'local_repo',
            default_value=(
                '/home/track-robot/track_robot_ws/src/track_robot_core/'
                'third_party/dinov3_py38')),
        DeclareLaunchArgument(
            'weights_path',
            default_value=(
                '/home/track-robot/track_robot_ws/models/'
                'dinov3_vits16plus_pretrain_lvd1689m.pth')),
        DeclareLaunchArgument(
            'hf_model_id',
            default_value='facebook/dinov3-vits16plus-pretrain-lvd1689m'),
        node,
    ])
