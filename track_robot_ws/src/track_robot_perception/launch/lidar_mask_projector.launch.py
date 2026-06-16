from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    projector = Node(
        package='track_robot_perception',
        executable='lidar_mask_projector_node',
        name='lidar_mask_projector_node',
        output='screen',
        parameters=[{
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'lidar_frame': LaunchConfiguration('lidar_frame'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'output_frame': LaunchConfiguration('output_frame'),
            'model_config': LaunchConfiguration('model_config'),
            'score_threshold': LaunchConfiguration('score_threshold'),
            'device': LaunchConfiguration('device'),
            'run_inference_every_n_images':
                LaunchConfiguration('run_inference_every_n_images'),
            'project_every_n_clouds': LaunchConfiguration('project_every_n_clouds'),
            'max_instances': LaunchConfiguration('max_instances'),
            'min_projection_distance': LaunchConfiguration('min_projection_distance'),
            'max_projection_distance': LaunchConfiguration('max_projection_distance'),
            'publish_unknown_points': LaunchConfiguration('publish_unknown_points'),
            'publish_only_labelled_points':
                LaunchConfiguration('publish_only_labelled_points'),
            'keep_intensity': LaunchConfiguration('keep_intensity'),
            'default_class_id': LaunchConfiguration('default_class_id'),
            'default_instance_id': LaunchConfiguration('default_instance_id'),
            'max_mask_age_sec': LaunchConfiguration('max_mask_age_sec'),
            'timestamp_mode': LaunchConfiguration('timestamp_mode'),
            'debug_timing': LaunchConfiguration('debug_timing'),
            'resize_width': LaunchConfiguration('resize_width'),
        }],
    )

    # Reuse the same camera-to-LiDAR prototype transform as the working RGB colorizer.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_camera_link_to_rslidar_mask_projector_tf',
        arguments=[
            LaunchConfiguration('static_tf_x'),
            LaunchConfiguration('static_tf_y'),
            LaunchConfiguration('static_tf_z'),
            LaunchConfiguration('static_tf_yaw'),
            LaunchConfiguration('static_tf_pitch'),
            LaunchConfiguration('static_tf_roll'),
            LaunchConfiguration('static_tf_parent_frame'),
            LaunchConfiguration('static_tf_child_frame'),
        ],
        condition=IfCondition(LaunchConfiguration('publish_static_tf')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar_topic', default_value='/rslidar_points'),
        DeclareLaunchArgument('image_topic', default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/zed/zed_node/left/camera_info'),
        DeclareLaunchArgument('output_topic', default_value='/lidar_semantic_points'),
        DeclareLaunchArgument('lidar_frame', default_value='rslidar'),
        DeclareLaunchArgument('camera_frame', default_value='zed_left_camera_optical_frame'),
        DeclareLaunchArgument('output_frame', default_value=''),
        DeclareLaunchArgument(
            'model_config',
            default_value='COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_1x.yaml'),
        DeclareLaunchArgument('score_threshold', default_value='0.5'),
        DeclareLaunchArgument('device', default_value='auto'),
        DeclareLaunchArgument('run_inference_every_n_images', default_value='1'),
        DeclareLaunchArgument('project_every_n_clouds', default_value='1'),
        DeclareLaunchArgument('max_instances', default_value='20'),
        DeclareLaunchArgument('min_projection_distance', default_value='0.3'),
        DeclareLaunchArgument('max_projection_distance', default_value='20.0'),
        DeclareLaunchArgument('publish_unknown_points', default_value='true'),
        DeclareLaunchArgument('publish_only_labelled_points', default_value='false'),
        DeclareLaunchArgument('keep_intensity', default_value='true'),
        DeclareLaunchArgument('default_class_id', default_value='-1'),
        DeclareLaunchArgument('default_instance_id', default_value='-1'),
        DeclareLaunchArgument('max_mask_age_sec', default_value='0.3'),
        DeclareLaunchArgument('timestamp_mode', default_value='auto'),
        DeclareLaunchArgument('debug_timing', default_value='true'),
        DeclareLaunchArgument('resize_width', default_value='0'),
        DeclareLaunchArgument('publish_static_tf', default_value='true'),
        DeclareLaunchArgument('static_tf_parent_frame', default_value='zed_camera_link'),
        DeclareLaunchArgument('static_tf_child_frame', default_value='rslidar'),
        DeclareLaunchArgument('static_tf_x', default_value='-0.05'),
        DeclareLaunchArgument('static_tf_y', default_value='0.0'),
        DeclareLaunchArgument('static_tf_z', default_value='0.20'),
        DeclareLaunchArgument('static_tf_yaw', default_value='1.08'),
        DeclareLaunchArgument('static_tf_pitch', default_value='-0.03'),
        DeclareLaunchArgument('static_tf_roll', default_value='0.0'),
        projector,
        static_tf,
    ])
