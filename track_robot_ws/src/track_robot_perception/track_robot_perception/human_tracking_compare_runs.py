#!/usr/bin/env python3

import argparse
import json
import sys


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('reports', nargs='+')
    parsed = parser.parse_args(args)
    reports = []
    for path in parsed.reports:
        with open(path, 'r', encoding='utf-8') as report_file:
            reports.append((path, json.load(report_file)))

    baseline = reports[0][1]
    failures = []
    for path, report in reports:
        if report.get('logical_target_ids') != baseline.get('logical_target_ids'):
            failures.append(f'{path}: logical target ID sequence differs')
        if report.get('selected_tracklet_sequence') != baseline.get('selected_tracklet_sequence'):
            failures.append(f'{path}: selected tracklet sequence differs')
        if report.get('unconfirmed_tracklet_switch_count', 0) != 0:
            failures.append(f'{path}: contains unconfirmed physical-tracklet switches')
        if not report.get('rate_target_met', False):
            failures.append(f'{path}: 15 Hz rate target was not met')
        if not report.get('sync_target_met', False):
            failures.append(f'{path}: 80 ms synchronization target was not met')

    exact_sequence_match = len({
        report.get('sensor_sequence_sha256') for _, report in reports}) == 1
    result = {
        'reports': [path for path, _ in reports],
        'exact_sensor_sequence_match': exact_sequence_match,
        'failures': failures,
        'passed': not failures,
    }
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if not failures else 1


if __name__ == '__main__':
    sys.exit(main())
