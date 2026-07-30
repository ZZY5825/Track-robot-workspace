from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_dynamic_obstacles_use_current_filtered_marks_and_raw_clear_rays():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'nav2_phase4b.yaml').read_text())
    layers = (
        config['local_costmap']['local_costmap']['ros__parameters'][
            'voxel_layer'],
        config['global_costmap']['global_costmap']['ros__parameters'][
            'obstacle_layer'],
    )

    for layer in layers:
        raw = layer['raw_clear']
        filtered = layer['filtered_mark']
        assert raw['data_type'] == 'PointCloud2'
        assert raw['raytrace_max_range'] == 8.0
        assert raw['obstacle_max_range'] == 8.0
        assert filtered['data_type'] == 'PointCloud2'
        assert filtered['obstacle_max_range'] == 8.0
        assert raw['observation_persistence'] == 0.0
        assert filtered['observation_persistence'] == 0.0
