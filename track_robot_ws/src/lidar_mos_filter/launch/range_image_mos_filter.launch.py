from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="lidar_mos_filter",
                executable="range_image_mos_filter",
                name="range_image_mos_filter",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/rslidar_points",
                        "static_topic": "/rslidar_points_static",
                        "dynamic_topic": "/rslidar_points_dynamic_debug",
                        "odom_topic": "/odom",
                        "use_odom": True,
                        "filter_only_when_stationary": True,
                        "min_range": 0.8,
                        "max_range": 30.0,
                        "foreground_margin": 0.45,
                        "background_match_tolerance": 0.25,
                        "background_update_alpha": 0.05,
                        "min_background_observations": 4,
                        "stationary_translation_threshold": 0.03,
                        "stationary_yaw_threshold_deg": 1.0,
                        "publish_dynamic_debug": True,
                    }
                ],
            )
        ]
    )
