import ast
from pathlib import Path

import yaml


LAUNCH = (
    Path(__file__).resolve().parents[1] /
    'launch' /
    'semantic_search_phase0.launch.py'
)
CONFIG = (
    Path(__file__).resolve().parents[1] /
    'config' /
    'semantic_search_phase0.yaml'
)
LAUNCH_ARGUMENTS = {
    'use_sim_time',
    'start_evaluator',
    'config_file',
    'manifest_path',
    'output_path',
    'tegrastats_path',
    'duration_sec',
    'run_id',
    'replay_rate',
    'software_revision',
    'freshness_time_base',
    'timing_policy',
}
EVALUATOR_RUNTIME_PARAMETERS = {
    'config_path',
    'duration_sec',
    'manifest_path',
    'output_path',
    'replay_rate',
    'run_id',
    'software_revision',
    'tegrastats_path',
    'freshness_time_base',
    'timing_policy',
}
DYNAMIC_TIMING_PARAMETERS = {'freshness_time_base', 'timing_policy'}


def string_keyword(call, name):
    for keyword in call.keywords:
        if keyword.arg == name and isinstance(keyword.value, ast.Constant):
            return keyword.value.value
    return None


def keyword_value(call, name):
    for keyword in call.keywords:
        if keyword.arg == name:
            return keyword.value
    return None


def inline_parameters(call):
    parameters = keyword_value(call, 'parameters')
    assert isinstance(parameters, ast.List)
    dictionaries = [
        item for item in parameters.elts if isinstance(item, ast.Dict)
    ]
    assert len(dictionaries) == 1
    return {
        key.value: value
        for key, value in zip(
            dictionaries[0].keys, dictionaries[0].values)
        if isinstance(key, ast.Constant)
    }


def test_phase0_launch_contains_only_diagnostic_and_evaluator_nodes():
    source = LAUNCH.read_text(encoding='utf-8')
    tree = ast.parse(source)
    node_calls = [
        item for item in ast.walk(tree)
        if isinstance(item, ast.Call) and
        isinstance(item.func, ast.Name) and item.func.id == 'Node'
    ]
    assert len(node_calls) == 2
    executables = {
        string_keyword(call, 'executable') for call in node_calls
    }
    assert executables == {
        'semantic_search_localization_health',
        'semantic_search_evaluator',
    }
    for forbidden in (
            'cmd_vel', 'SearchMotionIntent', 'FollowDecision',
            'controller', 'planner', 'motion_bridge'):
        assert forbidden not in source


def test_evaluator_is_conditionally_started():
    source = LAUNCH.read_text(encoding='utf-8')
    tree = ast.parse(source)
    launch_argument_calls = [
        item for item in ast.walk(tree)
        if isinstance(item, ast.Call) and
        isinstance(item.func, ast.Name) and
        item.func.id == 'DeclareLaunchArgument'
    ]
    launch_arguments = {
        call.args[0].value for call in launch_argument_calls
        if call.args and isinstance(call.args[0], ast.Constant)
    }
    assert len(launch_argument_calls) == 12
    assert launch_arguments == LAUNCH_ARGUMENTS
    launch_argument_by_name = {
        call.args[0].value: call for call in launch_argument_calls
    }
    assert string_keyword(
        launch_argument_by_name['freshness_time_base'],
        'default_value') == 'source_clock'
    assert string_keyword(
        launch_argument_by_name['timing_policy'],
        'default_value') == 'online_source_time'

    node_calls = {
        string_keyword(item, 'executable'): item
        for item in ast.walk(tree)
        if isinstance(item, ast.Call) and
        isinstance(item.func, ast.Name) and item.func.id == 'Node'
    }
    localization_condition = keyword_value(
        node_calls['semantic_search_localization_health'], 'condition')
    assert localization_condition is None

    evaluator_condition = keyword_value(
        node_calls['semantic_search_evaluator'], 'condition')
    assert isinstance(evaluator_condition, ast.Call)
    assert isinstance(evaluator_condition.func, ast.Name)
    assert evaluator_condition.func.id == 'IfCondition'
    assert len(evaluator_condition.args) == 1
    assert isinstance(evaluator_condition.args[0], ast.Name)
    assert evaluator_condition.args[0].id == 'start_evaluator'

    localization_parameters = inline_parameters(
        node_calls['semantic_search_localization_health'])
    evaluator_parameters = inline_parameters(
        node_calls['semantic_search_evaluator'])
    assert isinstance(
        localization_parameters['freshness_time_base'], ast.Name)
    assert localization_parameters['freshness_time_base'].id == (
        'freshness_time_base')
    assert isinstance(
        evaluator_parameters['freshness_time_base'], ast.Name)
    assert evaluator_parameters['freshness_time_base'].id == (
        'freshness_time_base')
    assert isinstance(evaluator_parameters['timing_policy'], ast.Name)
    assert evaluator_parameters['timing_policy'].id == 'timing_policy'


def test_evaluator_yaml_does_not_override_launch_runtime_parameters():
    config = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    evaluator_parameters = config['semantic_search_evaluator']['ros__parameters']
    assert EVALUATOR_RUNTIME_PARAMETERS.isdisjoint(evaluator_parameters)
    localization_parameters = config[
        'semantic_search_localization_health']['ros__parameters']
    assert DYNAMIC_TIMING_PARAMETERS.isdisjoint(localization_parameters)
    assert localization_parameters['state_topic'] == (
        '/semantic_memory/localization_state')
