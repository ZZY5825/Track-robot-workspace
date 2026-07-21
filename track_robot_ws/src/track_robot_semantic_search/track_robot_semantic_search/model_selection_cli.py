import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

from .manifest import write_json_atomic
from .model_selection import select_candidate, validate_benchmark


def run(input_path: Path, output_path: Path) -> int:
    try:
        with Path(input_path).open('r', encoding='utf-8') as stream:
            payload = json.load(stream)
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError('could not load benchmark: {}'.format(exc)) from exc
    validate_benchmark(payload)
    selection = select_candidate(payload['candidates'])
    output = dict(payload)
    output['selection'] = asdict(selection)
    write_json_atomic(Path(output_path), output)
    return 0 if selection.status == 'selected' else 2


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(
        description='Select a Phase 1 text/image model from measured gates.')
    root.add_argument('--input', type=Path, required=True)
    root.add_argument('--output', type=Path, required=True)
    return root


def main(argv=None) -> None:
    arguments = parser().parse_args(argv)
    try:
        exit_code = run(arguments.input, arguments.output)
    except (ValueError, OSError) as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1)
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
