from setuptools import setup

package_name = 'track_robot_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml',
            'README.md',
            'requirements-human-tracking.txt',
        ]),
        ('share/' + package_name + '/docs', [
            'docs/fast_lio_rshelios.md',
            'docs/phidget_imu_time_sync.md',
            'docs/imu_lio_debug.md',
            'docs/point_lio_rshelios.md',
            'docs/point_lio_ros2_port_assessment.md',
            'docs/pretrained_lidar_feasibility.md',
            'docs/human_tracking_dependencies.md',
            'docs/human_tracking_progress.md',
            'docs/human_tracking_reinforcement.md',
            'docs/lidar_phase4_methods.md',
        ]),
        ('share/' + package_name + '/config', [
            'config/fast_lio_rshelios.yaml',
            'config/human_tracking.yaml',
            'config/imu_lio_adapter.yaml',
            'config/imu_lio_debug.yaml',
            'config/phidget_imu.yaml',
            'config/point_lio_rshelios_lidar_only.yaml',
            'config/point_lio_rshelios.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/fast_lio_rshelios.launch.py',
            'launch/fast_lio_rshelios_mapping.launch.py',
            'launch/human_camera_tracking.launch.py',
            'launch/human_tracking_simplified.launch.py',
            'launch/human_tracking_rosbag_replay.launch.py',
            'launch/human_tracking_validation.launch.py',
            'launch/camera_lidar_tracklet_tracking.launch.py',
            'launch/lidar_camera_colorizer.launch.py',
            'launch/lidar_cluster_baseline.launch.py',
            'launch/lidar_ground_segment.launch.py',
            'launch/lidar_human_segment.launch.py',
            'launch/lidar_mask_projector.launch.py',
            'launch/phidget_imu.launch.py',
            'launch/imu_lio_debug.launch.py',
            'launch/point_lio_rshelios.launch.py',
            'launch/zed_dinov3_feature.launch.py',
            'launch/zed_mask_rcnn.launch.py',
            'launch/zed_pose_rcnn.launch.py',
            'launch/zed_rfdetr_small.launch.py',
        ]),
        ('share/' + package_name + '/rviz', [
            'rviz/human_tracking.rviz',
            'rviz/point_lio.rviz',
        ]),
        ('share/' + package_name + '/scripts', [
            'scripts/prepare_lidar_for_pretrained_model.py',
            'scripts/run_pretrained_lidar_inference.py',
            'scripts/test_dinov3_on_image.py',
            'scripts/visualize_lidar_prediction.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='track-robot',
    maintainer_email='track-robot@example.com',
    description='LiDAR-camera colorization tools for the Track Robot.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'lidar_camera_colorizer = track_robot_perception.lidar_camera_colorizer:main',
            'lidar_cluster_baseline_node = '
            'track_robot_perception.lidar_cluster_baseline_node:main',
            'lidar_ground_segment_node = '
            'track_robot_perception.lidar_ground_segment_node:main',
            'lidar_human_segment_node = '
            'track_robot_perception.lidar_human_segment_node:main',
            'export_lidar_frames_node = '
            'track_robot_perception.export_lidar_frames_node:main',
            'lidar_mask_projector_node = '
            'track_robot_perception.lidar_mask_projector_node:main',
            'imu_collect_static_sample = '
            'track_robot_perception.imu_collect_static_sample:main',
            'imu_six_face_calibrate = '
            'track_robot_perception.imu_six_face_calibrate:main',
            'phidget_spatial_imu_node = '
            'track_robot_perception.phidget_spatial_imu_node:main',
            'imu_lio_adapter_node = '
            'track_robot_perception.imu_lio_adapter_node:main',
            'imu_lio_debug_node = '
            'track_robot_perception.imu_lio_debug_node:main',
            'analyze_lio_bag.py = track_robot_perception.analyze_lio_bag:main',
            'point_lio_offset_sweep.py = '
            'track_robot_perception.point_lio_offset_sweep:main',
            'rslidar_point_lio_adapter_node = '
            'track_robot_perception.rslidar_point_lio_adapter_node:main',
            'imu_static_check = track_robot_perception.imu_static_check:main',
            'zed_mask_rcnn_node = track_robot_perception.zed_mask_rcnn_node:main',
            'human_image_tracker_node = '
            'track_robot_perception.human_image_tracker_node:main',
            'gesture_trigger_node = '
            'track_robot_perception.gesture_trigger_node:main',
            'camera_target_lock_node = '
            'track_robot_perception.camera_target_lock_node:main',
            'human_tracking_pipeline_diagnostic = '
            'track_robot_perception.human_tracking_pipeline_diagnostic:main',
            'human_tracking_regression_monitor = '
            'track_robot_perception.human_tracking_regression_monitor:main',
            'human_tracking_compare_runs = '
            'track_robot_perception.human_tracking_compare_runs:main',
            'zed_dinov3_feature_node = '
            'track_robot_perception.zed_dinov3_feature_node:main',
            'zed_rfdetr_small_node = '
            'track_robot_perception.zed_rfdetr_small_node:main',
        ],
    },
)
