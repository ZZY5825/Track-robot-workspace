import argparse
import json
from pathlib import Path
import sys

from .manifest import write_json_atomic
from .task_threshold_calibration import calibrate_task_threshold


def _load_jsonl(path):
    records = []
    try:
        with Path(path).open('r', encoding='utf-8') as stream:
            for line_number, line in enumerate(stream, start=1):
                if not line.strip():
                    continue
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError as error:
                    raise ValueError(
                        'invalid JSONL at line {}: {}'.format(
                            line_number, error)) from error
    except OSError as error:
        raise ValueError('could not load {}: {}'.format(path, error)) from error
    return records


def run(samples_path, output_path, dataset_id):
    report = calibrate_task_threshold(dataset_id, _load_jsonl(samples_path))
    write_json_atomic(output_path, report)
    return 0 if report['status'] == 'calibrated' else 2


def parser():
    root = argparse.ArgumentParser(
        description='Freeze a fail-closed Phase 2 task relevance threshold.')
    root.add_argument('--samples', type=Path, required=True)
    root.add_argument('--output', type=Path, required=True)
    root.add_argument('--dataset-id', required=True)
    return root


def main(argv=None):
    arguments = parser().parse_args(argv)
    try:
        code = run(arguments.samples, arguments.output, arguments.dataset_id)
    except (ValueError, OSError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
    raise SystemExit(code)


if __name__ == '__main__':
    main()
