from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_topic = LaunchConfiguration('lidar_topic')
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    output_topic = LaunchConfiguration('output_topic')
    lidar_frame = LaunchConfiguration('lidar_frame')
    camera_frame = LaunchConfiguration('camera_frame')
    output_frame = LaunchConfiguration('output_frame')
    publish_static_tf = LaunchConfiguration('publish_static_tf')

    colorizer = Node(
        package='track_robot_perception',
        executable='lidar_camera_colorizer',
        name='lidar_camera_colorizer_node',
        output='screen',
        parameters=[{
            'lidar_topic': lidar_topic,
            'image_topic': image_topic,
            'camera_info_topic': camera_info_topic,
            'output_topic': output_topic,
            'lidar_frame': lidar_frame,
            'camera_frame': camera_frame,
            'output_frame': output_frame,
            'max_projection_distance': LaunchConfiguration('max_projection_distance'),
            'min_projection_distance': LaunchConfiguration('min_projection_distance'),
            'use_approximate_sync': LaunchConfiguration('use_approximate_sync'),
            'max_image_age_sec': LaunchConfiguration('max_image_age_sec'),
            'default_color_for_unprojected_points':
                LaunchConfiguration('default_color_for_unprojected_points'),
            'publish_uncolored_points': LaunchConfiguration('publish_uncolored_points'),
        }],
    )

    # Disabled by default because the sign/direction of the camera-LiDAR transform must be
    # verified on the robot. ROS2 Euler arguments are x y z yaw pitch roll parent child.
    # The defaults encode
    # only the rough mechanical assumption in camera_link-style coordinates: LiDAR 5 cm behind and
    # 20 cm above the camera, with no rotation.
    prototype_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_camera_link_to_rslidar_prototype_tf',
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
        condition=IfCondition(publish_static_tf),
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar_topic', default_value='/rslidar_points'),
        DeclareLaunchArgument('image_topic', default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument('camera_info_topic', default_value='/zed/zed_node/left/camera_info'),
        DeclareLaunchArgument('output_topic', default_value='/lidar_colored_points'),
        DeclareLaunchArgument('lidar_frame', default_value='rslidar'),
        DeclareLaunchArgument('camera_frame', default_value='zed_left_camera_optical_frame'),
        DeclareLaunchArgument('output_frame', default_value=''),
        DeclareLaunchArgument('max_projection_distance', default_value='80.0'),
        DeclareLaunchArgument('min_projection_distance', default_value='0.2'),
        DeclareLaunchArgument('use_approximate_sync', default_value='false'),
        DeclareLaunchArgument('max_image_age_sec', default_value='0.25'),
        DeclareLaunchArgument('default_color_for_unprojected_points', default_value='120,120,120'),
        DeclareLaunchArgument('publish_uncolored_points', default_value='true'),
        DeclareLaunchArgument('publish_static_tf', default_value='false'),
        DeclareLaunchArgument('static_tf_parent_frame', default_value='zed_camera_link'),
        DeclareLaunchArgument('static_tf_child_frame', default_value='rslidar'),
        DeclareLaunchArgument('static_tf_x', default_value='-0.05'),
        DeclareLaunchArgument('static_tf_y', default_value='0.0'),
        DeclareLaunchArgument('static_tf_z', default_value='0.20'),
        DeclareLaunchArgument('static_tf_yaw', default_value='1.08'),
        DeclareLaunchArgument('static_tf_pitch', default_value='-0.03'),
        DeclareLaunchArgument('static_tf_roll', default_value='0.0'),
        colorizer,
        prototype_static_tf,
    ])
