import json

from track_robot_semantic_search.phase04_validation import (
    build_phase4_contract_report,
    main,
)


EXPECTED_SCENARIOS = {
    'success',
    'no_target',
    'ambiguous_target',
    'target_lost',
    'invalid_position',
    'blocked_path',
    'stale_map',
    'localization_reset',
}


def test_phase4_contract_report_covers_success_and_required_failures():
    report = build_phase4_contract_report()

    assert report['schema_version'] == 'phase0_4_validation/1.0.0'
    assert report['phase4']['status'] == 'PASS'
    assert set(report['phase4']['scenarios']) == EXPECTED_SCENARIOS
    assert all(
        item['test_status'] == 'PASS'
        for item in report['phase4']['scenarios'].values())
    success = report['phase4']['scenarios']['success']
    assert success['planning_status'] == 'PASS'
    assert success['global_object_id'] == 42
    assert success['memory_epoch_id'] == 11
    assert success['localization_epoch_id'] == 7
    assert success['query_id'] == 1234
    assert success['query_version'] == 2
    assert success['candidate_count'] >= 8
    assert success['path_pose_count'] > 0
    assert report['safety']['planning_only'] is True
    assert report['safety']['motion_interfaces'] == []


def test_phase4_contract_cli_writes_strict_json(tmp_path):
    output = tmp_path / 'phase4_contract.json'

    assert main(['--output', str(output)]) == 0

    loaded = json.loads(output.read_text(encoding='utf-8'))
    assert loaded['phase4']['status'] == 'PASS'
    assert loaded['phase4']['scenarios']['blocked_path']['observed_reason'] == (
        'blocked_path')
