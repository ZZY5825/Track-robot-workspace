import argparse
import json
from pathlib import Path
import sys

from .manifest import sha256_file, write_json_atomic
from .phase2_evaluation import build_phase2_report


def _load_json(path, default=None):
    if path is None:
        return default
    try:
        with Path(path).open('r', encoding='utf-8') as stream:
            return json.load(stream)
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError('could not load {}: {}'.format(path, error)) from error


def _load_jsonl(path):
    if path is None:
        return []
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


def run(
        manifest_path, annotations_path, predictions_path, runtime_path,
        resources_path, output_path, deterministic_replay_passed,
        human_tracking_regression_passed, software_revision,
        task_threshold_calibration_path=None):
    manifest = _load_json(manifest_path, {})
    phase2 = manifest.get('phase2', {})
    report = build_phase2_report(
        dataset_id=manifest.get('dataset_id', ''),
        manifest_sha256=sha256_file(manifest_path),
        annotations=_load_jsonl(annotations_path),
        predictions=_load_jsonl(predictions_path),
        runtime=_load_json(runtime_path),
        resources=_load_json(resources_path),
        covered_scenarios=phase2.get('scenario_ids', []),
        deterministic_replay_passed=deterministic_replay_passed,
        human_tracking_regression_passed=human_tracking_regression_passed,
        software_revision=software_revision,
        task_threshold_calibration=_load_json(
            task_threshold_calibration_path))
    write_json_atomic(output_path, report)
    return 0 if report['passed'] else 2


def parser():
    root = argparse.ArgumentParser(
        description='Build a fail-closed Phase 2 semantic-memory report.')
    root.add_argument('--manifest', type=Path, required=True)
    root.add_argument('--annotations', type=Path)
    root.add_argument('--predictions', type=Path)
    root.add_argument('--runtime', type=Path)
    root.add_argument('--resources', type=Path)
    root.add_argument('--task-threshold-calibration', type=Path)
    root.add_argument('--output', type=Path, required=True)
    root.add_argument('--deterministic-replay-passed', action='store_true')
    root.add_argument('--human-tracking-regression-passed', action='store_true')
    root.add_argument('--software-revision', required=True)
    return root


def main(argv=None):
    arguments = parser().parse_args(argv)
    try:
        code = run(
            arguments.manifest, arguments.annotations, arguments.predictions,
            arguments.runtime, arguments.resources, arguments.output,
            arguments.deterministic_replay_passed,
            arguments.human_tracking_regression_passed,
            arguments.software_revision,
            arguments.task_threshold_calibration)
    except (ValueError, OSError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
    raise SystemExit(code)


if __name__ == '__main__':
    main()
