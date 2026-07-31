from pathlib import Path

import yaml


SOURCE_ROOT = Path(__file__).resolve().parents[3]
ROBOT_ROOT = SOURCE_ROOT / 'track_robot'


def _parameters(path, node):
    source = yaml.safe_load(path.read_text(encoding='utf-8'))
    return source[node]['ros__parameters']


def test_static_target_profile_covers_measured_inference_gap_end_to_end():
    measured_maximum_gap_sec = 3.51
    memory = _parameters(
        ROBOT_ROOT / 'track_robot_semantic_memory/config/phase4a_test.yaml',
        'semantic_memory',
    )
    phase4a = yaml.safe_load((
        SOURCE_ROOT /
        'track_robot_semantic_search/config/semantic_search_phase4a.yaml'
    ).read_text(encoding='utf-8'))
    selector = phase4a['phase4a_target_selector']['ros__parameters']
    planner = phase4a['phase4_approach_planner']['ros__parameters']
    navigation = _parameters(
        ROBOT_ROOT / 'track_robot_navigation/config/semantic_navigation.yaml',
        'semantic_navigation_supervisor',
    )

    assert memory['static_target_profile'] is True
    assert memory['task_grounding_maximum_age_sec'] > measured_maximum_gap_sec
    assert memory['static_stale_after_sec'] > measured_maximum_gap_sec
    assert memory['static_lost_after_sec'] > memory['static_stale_after_sec']
    assert selector['maximum_age_sec'] > measured_maximum_gap_sec
    assert planner['maximum_target_age_sec'] > measured_maximum_gap_sec
    assert navigation['static_target_mode'] is True
    assert navigation['maximum_target_age_sec'] > measured_maximum_gap_sec
    assert navigation['maximum_target_age_sec'] <= 4.0
