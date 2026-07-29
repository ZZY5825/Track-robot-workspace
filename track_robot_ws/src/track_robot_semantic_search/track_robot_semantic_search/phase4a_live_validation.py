"""Bounded live validator for the stationary Phase 0-4A test chain."""

import argparse
import math
from pathlib import Path

from .phase04_live_validation import (
    _write,
    build_live_report,
    collect_live,
)


PHASE4A_LOCALIZATION_TOPIC = (
    '/semantic_search/phase4a/localization_state')
PHASE4A_SELECTED_TARGET_TOPIC = (
    '/semantic_search/phase4a/selected_target')
PHASE4A_SPATIAL_OBJECTS_TOPIC = (
    '/semantic_search/phase4a/spatial_objects')


def parser():
    root = argparse.ArgumentParser(
        description=(
            'Collect one stationary Phase 0-4A live validation report'))
    root.add_argument('--query', required=True)
    root.add_argument('--query-id', required=True, type=int)
    root.add_argument('--query-version', type=int, default=1)
    root.add_argument('--duration-sec', type=float, default=25.0)
    root.add_argument('--output', required=True)
    root.add_argument(
        '--localization-topic',
        default=PHASE4A_LOCALIZATION_TOPIC,
        help=argparse.SUPPRESS)
    root.add_argument(
        '--active-objects-topic',
        default=PHASE4A_SPATIAL_OBJECTS_TOPIC,
        help=argparse.SUPPRESS)
    root.add_argument(
        '--selected-target-topic',
        default=PHASE4A_SELECTED_TARGET_TOPIC,
        help=argparse.SUPPRESS)
    return root


def main(argv=None):
    arguments = parser().parse_args(argv)
    if (
            arguments.query_id <= 0
            or arguments.query_version <= 0
            or not arguments.query.strip()
            or not arguments.query.isascii()
            or not math.isfinite(arguments.duration_sec)
            or arguments.duration_sec <= 0.0):
        return 4
    evidence = collect_live(
        arguments.duration_sec,
        arguments.query_id,
        arguments.query_version,
        localization_topic=arguments.localization_topic,
        active_objects_topic=arguments.active_objects_topic,
        selected_target_topic=arguments.selected_target_topic,
        collect_phase4a=True)
    report = build_live_report(
        evidence,
        arguments.query,
        arguments.duration_sec,
        require_advisory=True)
    _write(Path(arguments.output).expanduser(), report)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
