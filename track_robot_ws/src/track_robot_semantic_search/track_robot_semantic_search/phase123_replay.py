"""Run the bounded Phase 1→2→3 contract replay twice."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

from .manifest import write_json_atomic


_SCHEMA = 'phase123_yolo_world/1.0.0'


def _sha256(payload):
    return hashlib.sha256(payload).hexdigest()


def _write_bytes_atomic(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_bytes(payload)
    os.replace(str(temporary), str(path))


def _valid_output(payload):
    if payload.get('schema_version') != _SCHEMA:
        return False
    if payload.get('calibration_state') != 'UNCALIBRATED':
        return False
    if payload.get('best_candidate') != []:
        return False
    ranking = payload.get('diagnostic_ranking')
    return (
        isinstance(ranking, list) and bool(ranking)
        and ranking[0].get('evidence_mode') == 'YOLO_WORLD_GROUNDING'
    )


def run_phase123_replay_twice(
        executable, input_path, output_path, report_path):
    """Require byte-equivalent C++ results and a fail-closed winner."""

    executable = Path(executable)
    input_path = Path(input_path)
    if not executable.is_file() or not os.access(str(executable), os.X_OK):
        raise ValueError('semantic memory replay executable is unavailable')
    try:
        input_payload = input_path.read_bytes()
        parsed_input = json.loads(input_payload.decode('utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError(
            'Phase 1-3 replay input is invalid: {}'.format(error)) from error
    if parsed_input.get('schema_version') != _SCHEMA:
        raise ValueError('Phase 1-3 replay schema version is unsupported')

    with tempfile.TemporaryDirectory(prefix='phase123-replay-') as directory:
        first_path = Path(directory) / 'first.json'
        second_path = Path(directory) / 'second.json'
        executions = [
            subprocess.run(
                [str(executable), str(input_path), str(destination)],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
                timeout=120)
            for destination in (first_path, second_path)
        ]
        first = first_path.read_bytes() if first_path.is_file() else b''
        second = second_path.read_bytes() if second_path.is_file() else b''
        same = (
            all(item.returncode == 0 for item in executions)
            and bool(first) and first == second
        )
        parsed = None
        if same:
            try:
                parsed = json.loads(first.decode('utf-8'))
                same = _valid_output(parsed)
            except (UnicodeError, json.JSONDecodeError):
                same = False
        report = {
            'schema_version': _SCHEMA,
            'deterministic_replay_passed': bool(same),
            'production_best_candidate_empty': bool(
                parsed is not None and parsed.get('best_candidate') == []),
            'input_sha256': _sha256(input_payload),
            'first_output_sha256': _sha256(first) if first else None,
            'second_output_sha256': _sha256(second) if second else None,
            'reason': '' if same else 'replay_contract_failed',
        }
        write_json_atomic(report_path, report)
        if not same:
            return 2
        _write_bytes_atomic(output_path, first)
        return 0


def parser():
    root = argparse.ArgumentParser(
        description='Run Phase 1-3 YOLO-World contract replay twice.')
    root.add_argument('--executable', type=Path, required=True)
    root.add_argument('--input', type=Path, required=True)
    root.add_argument('--output', type=Path, required=True)
    root.add_argument('--report', type=Path, required=True)
    return root


def main(argv=None):
    arguments = parser().parse_args(argv)
    try:
        code = run_phase123_replay_twice(
            arguments.executable, arguments.input,
            arguments.output, arguments.report)
    except (ValueError, OSError, subprocess.SubprocessError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
    raise SystemExit(code)


if __name__ == '__main__':
    main()
