import argparse
import json
from pathlib import Path
import sys

from .manifest import sha256_file, write_json_atomic
from .phase1_baselines import BASELINES, build_baseline_report


def _load_json(path: Path):
    try:
        with Path(path).open('r', encoding='utf-8') as stream:
            return json.load(stream)
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError('could not load {}: {}'.format(path, exc)) from exc


def _load_jsonl(path: Path):
    if path is None or not Path(path).is_file():
        return []
    records = []
    with Path(path).open('r', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise ValueError(
                    'invalid JSONL at line {}: {}'.format(
                        line_number, exc)) from exc
    return records


def run(
        manifest_path: Path,
        observations_path: Path,
        output_dir: Path,
        software_revision: str,
        model_evidence_path: Path) -> int:
    manifest = _load_json(manifest_path)
    dataset_id = manifest.get('dataset_id')
    capabilities = manifest.get('capabilities')
    records = _load_jsonl(observations_path)
    model_evidence = (
        _load_json(model_evidence_path) if model_evidence_path else None)
    reports = []
    for baseline_id, _ in BASELINES:
        selected_model_evidence = model_evidence
        if isinstance(model_evidence, dict) and any(
                key in dict(BASELINES) for key in model_evidence):
            selected_model_evidence = model_evidence.get(baseline_id)
        report = build_baseline_report(
            baseline_id=baseline_id,
            dataset_id=dataset_id,
            manifest_sha256=sha256_file(manifest_path),
            manifest_capabilities=capabilities,
            records=records,
            software_revision=software_revision,
            model_evidence=selected_model_evidence)
        write_json_atomic(Path(output_dir) / '{}.json'.format(baseline_id), report)
        reports.append(report)
    return 2 if any(
        report['status'] in ('unavailable', 'failed') for report in reports) else 0


def parser():
    root = argparse.ArgumentParser(
        description='Write comparable semantic-search Phase 1 baseline reports.')
    root.add_argument('--manifest', type=Path, required=True)
    root.add_argument('--observations', type=Path)
    root.add_argument('--output-dir', type=Path, required=True)
    root.add_argument('--software-revision', required=True)
    root.add_argument('--model-evidence', type=Path)
    return root


def main(argv=None):
    arguments = parser().parse_args(argv)
    try:
        exit_code = run(
            manifest_path=arguments.manifest,
            observations_path=arguments.observations,
            output_dir=arguments.output_dir,
            software_revision=arguments.software_revision,
            model_evidence_path=arguments.model_evidence)
    except (ValueError, OSError) as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1)
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
