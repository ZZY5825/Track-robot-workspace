from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_semantic_profile_targets_the_launched_node_name():
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'semantic_memory_lidar_tracklets.launch.py'
    ).read_text(encoding='utf-8')
    profile = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'semantic_memory_lidar_tracklets.yaml')
        .read_text(encoding='utf-8')
    )

    expected_node_name = 'semantic_memory_lidar_tracklet_manager'
    assert "name='{}'".format(expected_node_name) in launch_source
    assert set(profile) == {expected_node_name}

    parameters = profile[expected_node_name]['ros__parameters']
    assert parameters['semantic_output_topic'] == '/semantic_memory/lidar_tracklets'
    assert parameters['max_range'] == 15.0
    assert parameters['generic_tracklet_min_points'] == 4


if __name__ == '__main__':
    test_semantic_profile_targets_the_launched_node_name()
