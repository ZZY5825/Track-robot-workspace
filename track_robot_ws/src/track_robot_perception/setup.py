from setuptools import setup

package_name = 'track_robot_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/docs', [
            'docs/fast_lio_rshelios.md',
            'docs/phidget_imu_time_sync.md',
            'docs/pretrained_lidar_feasibility.md',
        ]),
        ('share/' + package_name + '/config', [
            'config/fast_lio_rshelios.yaml',
            'config/phidget_imu.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/fast_lio_rshelios.launch.py',
            'launch/lidar_camera_colorizer.launch.py',
            'launch/lidar_cluster_baseline.launch.py',
            'launch/lidar_ground_segment.launch.py',
            'launch/lidar_human_segment.launch.py',
            'launch/lidar_mask_projector.launch.py',
            'launch/phidget_imu.launch.py',
            'launch/zed_dinov3_feature.launch.py',
            'launch/zed_mask_rcnn.launch.py',
            'launch/zed_pose_rcnn.launch.py',
            'launch/zed_rfdetr_small.launch.py',
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
    tests_require=['pytest'],
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
            'phidget_spatial_imu_node = '
            'track_robot_perception.phidget_spatial_imu_node:main',
            'zed_mask_rcnn_node = track_robot_perception.zed_mask_rcnn_node:main',
            'zed_dinov3_feature_node = '
            'track_robot_perception.zed_dinov3_feature_node:main',
            'zed_rfdetr_small_node = '
            'track_robot_perception.zed_rfdetr_small_node:main',
        ],
    },
)
