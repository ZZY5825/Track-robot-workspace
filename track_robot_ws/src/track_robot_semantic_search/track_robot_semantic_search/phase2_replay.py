import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

from .manifest import write_json_atomic


def _sha256(payload):
    return hashlib.sha256(payload).hexdigest()


def _write_bytes_atomic(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_bytes(payload)
    os.replace(str(temporary), str(path))


def run_replay_twice(executable, input_path, output_path, report_path):
    executable = Path(executable)
    input_path = Path(input_path)
    if not executable.is_file() or not os.access(str(executable), os.X_OK):
        raise ValueError('semantic memory replay executable is unavailable')
    try:
        input_payload = input_path.read_bytes()
        parsed_input = json.loads(input_payload.decode('utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError('normalized replay input is invalid: {}'.format(error)) \
            from error
    if parsed_input.get('schema_version') != '1.0.0':
        raise ValueError('normalized replay schema version is unsupported')

    with tempfile.TemporaryDirectory(prefix='phase2-replay-') as directory:
        first_path = Path(directory) / 'first.json'
        second_path = Path(directory) / 'second.json'
        executions = []
        for destination in (first_path, second_path):
            completed = subprocess.run(
                [str(executable), str(input_path), str(destination)],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
                timeout=120)
            executions.append(completed)
        if any(item.returncode != 0 for item in executions):
            report = {
                'schema_version': '1.0.0',
                'deterministic_replay_passed': False,
                'input_sha256': _sha256(input_payload),
                'first_output_sha256': None,
                'second_output_sha256': None,
                'reason': 'replay_execution_failed',
            }
            write_json_atomic(report_path, report)
            return 2
        first = first_path.read_bytes()
        second = second_path.read_bytes()
        first_hash = _sha256(first)
        second_hash = _sha256(second)
        same = first == second
        reason = '' if same else 'byte_equivalence_failed'
        if same:
            try:
                parsed = json.loads(first.decode('utf-8'))
                if parsed.get('schema_version') != '1.0.0':
                    raise ValueError('unsupported output schema')
            except (UnicodeError, json.JSONDecodeError, ValueError):
                same = False
                reason = 'output_json_invalid'
        report = {
            'schema_version': '1.0.0',
            'deterministic_replay_passed': same,
            'input_sha256': _sha256(input_payload),
            'first_output_sha256': first_hash,
            'second_output_sha256': second_hash,
            'reason': reason,
        }
        write_json_atomic(report_path, report)
        if not same:
            return 2
        _write_bytes_atomic(output_path, first)
        return 0


def parser():
    root = argparse.ArgumentParser(
        description='Run normalized Phase 2 C++ replay twice and compare bytes.')
    root.add_argument('--executable', type=Path, required=True)
    root.add_argument('--input', type=Path, required=True)
    root.add_argument('--output', type=Path, required=True)
    root.add_argument('--report', type=Path, required=True)
    return root


def main(argv=None):
    arguments = parser().parse_args(argv)
    try:
        code = run_replay_twice(
            arguments.executable, arguments.input,
            arguments.output, arguments.report)
    except (ValueError, OSError, subprocess.SubprocessError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
    raise SystemExit(code)


if __name__ == '__main__':
    main()
