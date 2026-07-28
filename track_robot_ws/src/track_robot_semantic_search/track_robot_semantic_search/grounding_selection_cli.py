import argparse
import json
from pathlib import Path
import sys

from .grounding_selection import select_grounding_candidate
from .manifest import write_json_atomic


_MAX_ERROR_LENGTH = 512


def _bounded_error(error: BaseException) -> str:
    message = ' '.join(str(error).splitlines()).strip()
    if not message:
        message = error.__class__.__name__
    return message[:_MAX_ERROR_LENGTH]


def _load_report(path: Path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def _selection_payload(selection):
    return {
        'status': selection.status,
        'selected_candidate_id': selection.selected_candidate_id,
        'rejected': {
            candidate_id: list(reasons)
            for candidate_id, reasons in selection.rejected.items()
        },
        'ranking': list(selection.ranking),
    }


def run(report_paths, output_path: Path) -> int:
    try:
        reports = [_load_report(Path(path)) for path in report_paths]
        selection = select_grounding_candidate(reports)
        payload = _selection_payload(selection)
        json.dumps(payload, allow_nan=False)
        write_json_atomic(Path(output_path), payload)
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as error:
        print(_bounded_error(error), file=sys.stderr)
        return 2
    return 0 if selection.status == 'selected' else 2


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(
        description='Select a semantic grounding candidate from reports.')
    root.add_argument('--report', type=Path, action='append', required=True)
    root.add_argument('--output', type=Path, required=True)
    return root


def main(argv=None) -> None:
    arguments = parser().parse_args(argv)
    raise SystemExit(run(arguments.report, arguments.output))


if __name__ == '__main__':
    main()
