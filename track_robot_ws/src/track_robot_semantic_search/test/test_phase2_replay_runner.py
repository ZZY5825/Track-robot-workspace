import json
from pathlib import Path

from track_robot_semantic_search.phase2_replay import run_replay_twice


def executable(tmp_path, body):
    path = tmp_path / 'replay'
    path.write_text('#!/bin/sh\nset -eu\n' + body, encoding='utf-8')
    path.chmod(0o755)
    return path


def test_runner_requires_byte_equivalence_and_writes_evidence(tmp_path):
    replay = executable(tmp_path, 'cp "$1" "$2"\n')
    source = tmp_path / 'input.json'
    output = tmp_path / 'output.json'
    report = tmp_path / 'report.json'
    source.write_text(json.dumps({
        'schema_version': '1.0.0', 'frames': [],
    }, sort_keys=True), encoding='utf-8')

    exit_code = run_replay_twice(replay, source, output, report)

    assert exit_code == 0
    evidence = json.loads(report.read_text(encoding='utf-8'))
    assert evidence['deterministic_replay_passed'] is True
    assert evidence['first_output_sha256'] == evidence['second_output_sha256']
    assert len(evidence['input_sha256']) == 64
    assert output.read_bytes() == source.read_bytes()


def test_runner_fails_closed_when_outputs_differ(tmp_path):
    replay = executable(
        tmp_path,
        'printf \'{"schema_version":"1.0.0","output":"%s"}\\n\' "$2" > "$2"\n')
    source = tmp_path / 'input.json'
    output = tmp_path / 'output.json'
    report = tmp_path / 'report.json'
    source.write_text('{"schema_version":"1.0.0","frames":[]}',
                      encoding='utf-8')

    exit_code = run_replay_twice(replay, source, output, report)

    assert exit_code == 2
    evidence = json.loads(report.read_text(encoding='utf-8'))
    assert evidence['deterministic_replay_passed'] is False
    assert not output.exists()


def test_runner_rejects_non_json_output(tmp_path):
    replay = executable(tmp_path, 'printf not-json > "$2"\n')
    source = tmp_path / 'input.json'
    source.write_text('{"schema_version":"1.0.0","frames":[]}',
                      encoding='utf-8')

    exit_code = run_replay_twice(
        replay, source, tmp_path / 'output.json', tmp_path / 'report.json')

    assert exit_code == 2
    report = json.loads((tmp_path / 'report.json').read_text())
    assert report['reason'] == 'output_json_invalid'
