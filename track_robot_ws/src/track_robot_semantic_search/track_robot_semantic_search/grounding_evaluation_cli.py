import argparse
import json
from pathlib import Path
import sys

from .grounding_dataset import load_grounding_dataset
from .grounding_evaluation import evaluate_grounding_candidate
from .grounding_predictions import load_grounding_predictions
from .manifest import write_json_atomic


_MAX_ERROR_LENGTH = 512


def _bounded_error(error: BaseException) -> str:
    message = ' '.join(str(error).splitlines()).strip()
    if not message:
        message = error.__class__.__name__
    return message[:_MAX_ERROR_LENGTH]


def run(dataset_path: Path, predictions_path: Path, output_path: Path) -> int:
    try:
        dataset = load_grounding_dataset(Path(dataset_path))
        prediction_set = load_grounding_predictions(Path(predictions_path))
        report = evaluate_grounding_candidate(dataset, prediction_set)
        json.dumps(report, allow_nan=False)
        write_json_atomic(Path(output_path), report)
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as error:
        print(_bounded_error(error), file=sys.stderr)
        return 2
    return 0


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(
        description='Evaluate one semantic grounding candidate.')
    root.add_argument('--dataset', type=Path, required=True)
    root.add_argument('--predictions', type=Path, required=True)
    root.add_argument('--output', type=Path, required=True)
    return root


def main(argv=None) -> None:
    arguments = parser().parse_args(argv)
    raise SystemExit(run(
        arguments.dataset,
        arguments.predictions,
        arguments.output,
    ))


if __name__ == '__main__':
    main()
