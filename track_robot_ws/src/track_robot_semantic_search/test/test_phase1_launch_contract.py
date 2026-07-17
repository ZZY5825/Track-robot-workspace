import ast
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE_ROOT / 'launch' / 'semantic_search_phase1.launch.py'
CONFIG = PACKAGE_ROOT / 'config' / 'semantic_search_phase1.yaml'


def _constant_keyword(call, name):
    for keyword in call.keywords:
        if keyword.arg == name and isinstance(keyword.value, ast.Constant):
            return keyword.value.value
    return None


def test_phase1_launch_is_opt_in_and_contains_only_perception_worker():
    source = LAUNCH.read_text(encoding='utf-8')
    tree = ast.parse(source)
    nodes = [
        item for item in ast.walk(tree)
        if isinstance(item, ast.Call) and
        isinstance(item.func, ast.Name) and item.func.id == 'Node'
    ]

    assert len(nodes) == 1
    assert _constant_keyword(nodes[0], 'package') == 'track_robot_semantic_search'
    assert _constant_keyword(nodes[0], 'executable') == 'semantic_search_perception'
    condition = next(
        keyword.value for keyword in nodes[0].keywords
        if keyword.arg == 'condition')
    assert isinstance(condition, ast.Call)
    assert isinstance(condition.func, ast.Name)
    assert condition.func.id == 'IfCondition'

    arguments = {
        call.args[0].value: _constant_keyword(call, 'default_value')
        for call in ast.walk(tree)
        if isinstance(call, ast.Call) and
        isinstance(call.func, ast.Name) and
        call.func.id == 'DeclareLaunchArgument' and
        call.args and isinstance(call.args[0], ast.Constant)
    }
    assert arguments['start_perception'] == 'false'
    assert arguments['use_sim_time'] == 'false'
    assert arguments['adapter_implementation'] == 'openai_clip'
    assert arguments['model_name'] == 'ViT-B/32'


def test_phase1_config_is_bounded_and_namespaced():
    config = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    parameters = config['semantic_search_perception']['ros__parameters']

    assert parameters['target_rate_hz'] == 5.0
    assert parameters['grid_size'] == 2
    assert 1 <= parameters['max_regions'] <= 16
    assert parameters['adapter_implementation'] == 'openai_clip'
    assert parameters['model_name'] == 'ViT-B/32'
    assert parameters['query_topic'].startswith('/semantic_search/')
    assert parameters['regions_topic'] == '/semantic_search/regions'
    assert parameters['observations_topic'] == '/semantic_memory/observations'
    assert parameters['task_topic'] == '/semantic_memory/tasks'
    assert parameters['proposal_topic'] == '/semantic_memory/visual_proposals'
    assert 1 <= parameters['max_visual_candidates'] <= 64
    assert parameters['producer_epoch_seed'] >= 0
    assert parameters['diagnostics_topic'].startswith('/semantic_search/')


def test_phase1_launch_and_config_contain_no_motion_path():
    source = LAUNCH.read_text(encoding='utf-8') + CONFIG.read_text(encoding='utf-8')
    for forbidden in (
            'cmd_vel', 'SearchMotionIntent', 'FollowDecision',
            'controller', 'planner', 'motion_bridge', 'action_server'):
        assert forbidden not in source


def test_readme_documents_fail_closed_external_model_runtime():
    readme = (PACKAGE_ROOT / 'README.md').read_text(encoding='utf-8')

    assert 'Phase 1' in readme
    assert 'external runtime' in readme
    assert 'unavailable' in readme
    assert 'one visual backbone' in readme
