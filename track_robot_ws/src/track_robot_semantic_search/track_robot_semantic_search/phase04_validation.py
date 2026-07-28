"""Deterministic Phase 4 planning-only acceptance scenarios."""

import argparse
from dataclasses import replace
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import tempfile
import time

from .approach_planning import (
    GridMap,
    Phase4Planner,
    PlannerConfig,
    PlanningContext,
    TargetCandidate,
)


NOW_NS = 10_000_000_000


def _grid(stamp_ns=None):
    width = 80
    height = 80
    return GridMap(
        frame_id='base_link',
        stamp_ns=(
            NOW_NS - 50_000_000 if stamp_ns is None else stamp_ns),
        resolution=0.1,
        width=width,
        height=height,
        origin_x=-4.0,
        origin_y=-4.0,
        data=tuple([0] * (width * height)),
    )


def _blocked_grid():
    grid = _grid()
    data = list(grid.data)
    wall_x = 50
    for cell_y in range(grid.height):
        data[cell_y * grid.width + wall_x] = 100
    return replace(grid, data=tuple(data))


def _target(**overrides):
    values = {
        'memory_epoch_id': 11,
        'global_object_id': 42,
        'localization_epoch_id': 7,
        'query_id': 1234,
        'query_version': 2,
        'position_frame_id': 'base_link',
        'position_valid': True,
        'x': 2.0,
        'y': 0.0,
        'z': 0.4,
        'lifecycle_state': 'confirmed',
        'task_relevance': 0.82,
        'uncertainty': 0.18,
        'last_seen_ns': NOW_NS - 100_000_000,
    }
    values.update(overrides)
    return TargetCandidate(**values)


def _context(candidates=None, grid=None, **overrides):
    values = {
        'now_ns': NOW_NS,
        'localization_epoch_id': 7,
        'localization_healthy': True,
        'robot_x': 0.0,
        'robot_y': 0.0,
        'target_candidates': tuple(
            [_target()] if candidates is None else candidates),
        'grid': _grid() if grid is None else grid,
    }
    values.update(overrides)
    return PlanningContext(**values)


def _scenario_inputs():
    return {
        'success': (_context(), 'PASS', 'planned'),
        'no_target': (_context(candidates=()), 'FAIL', 'no_target'),
        'ambiguous_target': (
            _context(candidates=(
                _target(task_relevance=0.82),
                _target(
                    global_object_id=43,
                    task_relevance=0.78,
                    x=1.5,
                    y=0.5),
            )),
            'FAIL',
            'ambiguous_target',
        ),
        'target_lost': (
            _context(candidates=(_target(lifecycle_state='lost'),)),
            'FAIL',
            'target_lost',
        ),
        'invalid_position': (
            _context(candidates=(_target(position_valid=False),)),
            'FAIL',
            'invalid_position',
        ),
        'blocked_path': (
            _context(grid=_blocked_grid()),
            'FAIL',
            'blocked_path',
        ),
        'stale_map': (
            _context(grid=_grid(NOW_NS - 2_000_000_000)),
            'FAIL',
            'stale_map',
        ),
        'localization_reset': (
            _context(
                localization_epoch_id=8,
                candidates=(_target(localization_epoch_id=7),)),
            'FAIL',
            'localization_reset',
        ),
    }


def build_phase4_contract_report():
    planner = Phase4Planner(PlannerConfig())
    scenarios = {}
    all_passed = True
    for name, (context, expected_status, expected_reason) in (
            _scenario_inputs().items()):
        started = time.perf_counter_ns()
        result = planner.plan(context)
        latency_ms = (time.perf_counter_ns() - started) / 1_000_000.0
        passed = (
            result.status == expected_status
            and result.reason == expected_reason)
        all_passed = all_passed and passed
        evidence = {
            'test_status': 'PASS' if passed else 'FAIL',
            'expected_status': expected_status,
            'expected_reason': expected_reason,
            'planning_status': result.status,
            'observed_reason': result.reason,
            'latency_ms': latency_ms,
            'candidate_count': len(result.approach_candidates),
            'path_pose_count': len(result.path),
        }
        if result.target is not None:
            evidence.update({
                'memory_epoch_id': result.target.memory_epoch_id,
                'global_object_id': result.target.global_object_id,
                'localization_epoch_id': result.target.localization_epoch_id,
                'query_id': result.target.query_id,
                'query_version': result.target.query_version,
                'position_frame_id': result.target.position_frame_id,
                'task_relevance': result.target.task_relevance,
                'uncertainty': result.target.uncertainty,
            })
        scenarios[name] = evidence
    return {
        'schema_version': 'phase0_4_validation/1.0.0',
        'generated_at': datetime.now(timezone.utc).replace(
            microsecond=0).isoformat().replace('+00:00', 'Z'),
        'phase4': {
            'status': 'PASS' if all_passed else 'FAIL',
            'mode': 'DETERMINISTIC_CONTRACT',
            'scenarios': scenarios,
        },
        'safety': {
            'planning_only': True,
            'motion_interfaces': [],
        },
    }


def _write_json(path, payload):
    encoded = json.dumps(
        payload, allow_nan=False, indent=2, sort_keys=True) + '\n'
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=path.name + '.', suffix='.tmp', dir=str(path.parent))
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, str(path))
    finally:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass


def _parser():
    parser = argparse.ArgumentParser(
        description='Run deterministic Phase 4 planning-only acceptance cases')
    parser.add_argument('--output', required=True)
    return parser


def main(argv=None):
    args = _parser().parse_args(argv)
    report = build_phase4_contract_report()
    _write_json(Path(args.output).expanduser(), report)
    return 0 if report['phase4']['status'] == 'PASS' else 4


if __name__ == '__main__':
    raise SystemExit(main())
