#!/usr/bin/env python3
"""Render five publication figures from normalized human-following replay records."""

import argparse
from collections import Counter
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-human-following-statistics')

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
import numpy as np

from human_following_statistics import (
    FUNNEL_LABELS,
    aggregate_association,
    aggregate_funnel,
    align_repeatability,
    percentile_summary,
    summarize_run,
)


COLORS = {
    'red': '#CC3311', 'orange': '#EE7733', 'blue': '#4477AA',
    'green': '#2A7F76', 'purple': '#6F4C9B', 'gray': '#9AA1A6',
    'ink': '#202428', 'muted': '#5B6268', 'line': '#C8CDD1',
    'camera_lidar': '#2A7F76', 'camera_only': '#6F4C9B',
    'lidar_only': '#4477AA', 'prediction': '#D89B2B', 'none': '#C8CDD1',
}
BAG_COLORS = ['#4477AA', '#2A7F76', '#EE7733', '#6F4C9B']
RUN_COLORS = ['#4477AA', '#2A7F76', '#EE7733', '#CC6677', '#6F4C9B']


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data-dir', required=True)
    parser.add_argument('--output-dir', required=True)
    return parser.parse_args()


def short_bag(record):
    return record['bag'].replace('human_tracking_lidar_20260706_', '')


def configure_style():
    plt.rcParams.update({
        'font.family': 'DejaVu Sans', 'font.size': 9.3,
        'axes.titlesize': 11.2, 'axes.labelsize': 9.3,
        'legend.fontsize': 7.8,
    })


def save_figure(fig, output_dir, stem, metadata):
    output = Path(output_dir) / f'{stem}.png'
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=190, bbox_inches='tight', facecolor='white')
    fig.savefig(output.with_suffix('.pdf'), bbox_inches='tight', facecolor='white')
    plt.close(fig)
    (Path(output_dir) / f'{stem}.json').write_text(
        json.dumps(metadata, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def style_axis(axis):
    axis.grid(True, color='#E0E3E5', linewidth=0.55, zorder=0)
    axis.set_facecolor('#FAFAF9')


def draw_footer(fig, blocks):
    footer = fig.add_axes([0.04, 0.015, 0.92, 0.09])
    footer.axis('off')
    footer.axhline(0.98, color=COLORS['line'], linewidth=0.8)
    width = 1.0 / len(blocks)
    for index, (heading, value) in enumerate(blocks):
        x = 0.01 + index * width
        footer.text(x, 0.63, heading, transform=footer.transAxes,
                    fontsize=7.8, weight='bold', color=COLORS['muted'])
        footer.text(x, 0.16, value, transform=footer.transAxes,
                    fontsize=8.6, weight='bold', color=COLORS['ink'])


def add_boxplot(axis, values_by_bag, labels, colors, ylabel, title, reference=None,
                reference_label=None):
    clean = [values if values else [np.nan] for values in values_by_bag]
    boxes = axis.boxplot(
        clean, patch_artist=True, widths=0.58, showfliers=False,
        medianprops={'color': COLORS['ink'], 'linewidth': 1.4})
    for patch, color in zip(boxes['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.75)
    rng = np.random.default_rng(7)
    for index, (values, color) in enumerate(zip(values_by_bag, colors), start=1):
        if values:
            x = rng.normal(index, 0.045, len(values))
            axis.scatter(x, values, s=9, color=color, alpha=0.38,
                         linewidths=0, zorder=3)
    if reference is not None:
        axis.axhline(reference, color=COLORS['purple'], linestyle='--', linewidth=1.4,
                     label=reference_label)
        axis.legend(loc='upper right', frameon=True)
    axis.set_xticks(range(1, len(labels) + 1), labels)
    axis.set_ylabel(ylabel)
    axis.set_title(title, loc='left')
    style_axis(axis)


def render_association_statistics(records, output_dir):
    aggregate = aggregate_association(records)
    labels = [short_bag(record) for record in records]
    keys = [
        ('anchor_distance_m', 'anchor distance [m]', '(a) Camera-anchor spatial consistency',
         1.0, '1.0 m hard gate'),
        ('range_difference_m', 'range difference [m]', '(b) Radial consistency',
         0.75, '0.75 m hard gate'),
        ('projection_center_error_px', 'centre error [px]', '(c) Image-plane consistency',
         220.0, '220 px score scale'),
        ('association_score', 'published score', '(d) Selected association score',
         0.65, 'minimum score 0.65'),
    ]
    fig = plt.figure(figsize=(17.2, 8.5), facecolor='white')
    grid = fig.add_gridspec(2, 3, left=0.055, right=0.98, top=0.95, bottom=0.16,
                           hspace=0.34, wspace=0.25)
    for index, (key, ylabel, title, ref, ref_label) in enumerate(keys):
        axis = fig.add_subplot(grid[index // 2, index % 2])
        values_by_bag = [
            [float(row[key]) for row in record.get('associations', [])
             if isinstance(row.get(key), (int, float)) and np.isfinite(float(row[key]))]
            for record in records]
        add_boxplot(axis, values_by_bag, labels, BAG_COLORS, ylabel, title, ref, ref_label)
        if key in ('anchor_distance_m', 'range_difference_m', 'projection_center_error_px'):
            maximum = max([max(values) for values in values_by_bag if values] + [ref])
            axis.set_ylim(bottom=0, top=max(ref * 1.08, maximum * 1.12))
        else:
            axis.set_ylim(0.55, 1.0)

    cdf_axis = fig.add_subplot(grid[0, 2])
    scores = sorted(aggregate['association_score'])
    if scores:
        cdf_axis.step(scores, np.arange(1, len(scores) + 1) / len(scores), where='post',
                      color=COLORS['green'], linewidth=2.0)
    cdf_axis.axvline(0.65, color=COLORS['purple'], linestyle='--', linewidth=1.4)
    cdf_axis.set_xlim(0.60, 1.0)
    cdf_axis.set_ylim(0, 1.02)
    cdf_axis.set_xlabel('association score')
    cdf_axis.set_ylabel('empirical CDF')
    cdf_axis.set_title('(e) Aggregate score distribution', loc='left')
    style_axis(cdf_axis)

    availability = fig.add_subplot(grid[1, 2])
    availability.axis('off')
    availability.set_title('(f) Evidence coverage', loc='left')
    hypothesis_counts = Counter(
        int(row.get('hypothesis_count', 0))
        for record in records for row in record.get('associations', []))
    lines = [
        ('synchronized associations', str(aggregate['sample_count'])),
        ('finite selected scores', str(len(aggregate['association_score']))),
        ('bags represented', str(len(records))),
        ('top-two margins', str(len(aggregate['top_two_margin']))),
    ]
    for index, (label, value) in enumerate(lines):
        y = 0.84 - index * 0.15
        availability.text(0.05, y, label, fontsize=8.4, color=COLORS['muted'])
        availability.text(0.88, y, value, ha='right', fontsize=11, weight='bold',
                          color=COLORS['ink'])
    availability.text(
        0.05, 0.24,
        'hypothesis counts: ' + ', '.join(
            f'{key}: {value}' for key, value in sorted(hypothesis_counts.items())),
        fontsize=8.2, color=COLORS['ink'])
    availability.text(
        0.05, 0.04,
        'Only one valid hypothesis was published per synchronized selection;\n'
        'therefore a top-two margin is not reported for these recordings.',
        fontsize=8.2, color=COLORS['purple'], weight='bold', va='bottom')

    draw_footer(fig, [
        ('EVIDENCE', 'four recorded bags · run 1 only'),
        ('INTERPRETATION', 'internal camera–LiDAR consistency;\nnot ground-truth error'),
        ('SCORE THRESHOLD', 'configured\nmin_association_score = 0.65'),
        ('SYNC', 'state/tracklet/anchor ≤120 ms;\ncamera ≤200 ms'),
    ])
    metadata = {
        'source': 'four_recorded_rosbag_replays_run1',
        'bags': labels,
        'aggregation': aggregate,
        'hypothesis_count_distribution': dict(sorted(hypothesis_counts.items())),
        'gates': {
            'anchor_xy_m': 1.0, 'anchor_range_m': 0.75,
            'projection_center_score_scale_px': 220.0, 'min_association_score': 0.65,
        },
        'ground_truth_available': False,
        'interpretation': 'internal_consistency_only',
    }
    save_figure(fig, output_dir, 'human-following-association-statistics-v1', metadata)


def primary_episode(record):
    summary = summarize_run(record)
    target_id = summary['primary_target_id']
    locked = [
        row for row in record['states']
        if row['target_id'] == target_id and row['lock_state'] == 'TARGET_LOCKED']
    if not locked:
        return [], [], summary
    start = min(row['stamp'] for row in locked)
    episode_states = [
        row for row in record['states']
        if row['target_id'] == target_id and row['stamp'] >= start]
    end = max(row['stamp'] for row in episode_states)
    debug = [row for row in record['debug_samples'] if start <= row['stamp'] <= end]
    return episode_states, debug, summary


def render_filter_consistency(record, output_dir):
    states, debug, summary = primary_episode(record)
    start = min(row['stamp'] for row in states)
    times = np.asarray([row['stamp'] - start for row in debug], dtype=float)
    nis = np.asarray([max(float(row['kalman_nis_xy']), 1e-4) for row in debug], dtype=float)
    sources = [row['measurement_source'] for row in debug]
    accepted = [bool(row['measurement_accepted']) for row in debug]

    fig = plt.figure(figsize=(17.2, 8.5), facecolor='white')
    grid = fig.add_gridspec(2, 2, left=0.065, right=0.98, top=0.95, bottom=0.16,
                           hspace=0.31, wspace=0.22)
    nis_axis = fig.add_subplot(grid[0, 0])
    source_styles = {
        'camera_guided_anchor': (COLORS['orange'], 'camera-guided anchor'),
        'selected_tracklet_lidar_only': (COLORS['blue'], 'selected LiDAR tracklet'),
        'prediction': (COLORS['gray'], 'prediction'),
        'none': (COLORS['gray'], 'no measurement'),
    }
    for source, (color, label) in source_styles.items():
        mask = np.asarray([value == source for value in sources])
        if np.any(mask):
            nis_axis.scatter(times[mask], nis[mask], s=28, color=color, alpha=0.72,
                             edgecolors='white', linewidths=0.4, label=label)
    nis_axis.axhline(25.0, color=COLORS['orange'], linestyle='--', linewidth=1.3,
                     label='camera NIS gate 25.0')
    nis_axis.axhline(9.21, color=COLORS['blue'], linestyle=':', linewidth=1.5,
                     label='tracklet NIS gate 9.21')
    nis_axis.set_yscale('log')
    nis_axis.set_xlabel('time since primary target lock [s]')
    nis_axis.set_ylabel('Kalman NIS (xy) [log scale]')
    nis_axis.set_title('(a) Innovation consistency and configured gates', loc='left')
    nis_axis.legend(loc='upper right', frameon=True, ncol=2)
    style_axis(nis_axis)

    decision_axis = fig.add_subplot(grid[0, 1])
    decisions = Counter()
    for row in debug:
        if row['measurement_accepted']:
            decisions['accepted'] += 1
        elif row['measurement_source'] in ('none', 'prediction') and row['rejection_reason'] == 'none':
            decisions['no measurement / prediction'] += 1
        else:
            decisions[row['rejection_reason']] += 1
    decision_labels = list(decisions)
    decision_values = [decisions[label] for label in decision_labels]
    colors = [COLORS['green'] if label == 'accepted' else (
        COLORS['gray'] if label == 'no measurement / prediction' else COLORS['red'])
        for label in decision_labels]
    y = np.arange(len(decision_labels))
    decision_axis.barh(y, decision_values, color=colors, height=0.62)
    decision_axis.set_yticks(y, [label.replace('_', ' ') for label in decision_labels])
    decision_axis.invert_yaxis()
    decision_axis.set_xlabel('debug updates')
    decision_axis.set_title('(b) Measurement decisions', loc='left')
    for index, value in enumerate(decision_values):
        decision_axis.text(value + 0.4, index, str(value), va='center', fontsize=8.3,
                           weight='bold')
    style_axis(decision_axis)

    covariance_axis = fig.add_subplot(grid[1, 0])
    state_time = np.asarray([row['stamp'] - start for row in states], dtype=float)
    covariance = np.asarray([row['covariance_trace_xy'] for row in states], dtype=float)
    covariance_axis.plot(state_time, covariance, color=COLORS['green'], linewidth=1.8)
    covariance_axis.fill_between(state_time, 0, covariance, color=COLORS['green'], alpha=0.16)
    covariance_axis.set_xlabel('time since primary target lock [s]')
    covariance_axis.set_ylabel(r'$\mathrm{tr}(P_{xy})$ [m$^2$]')
    covariance_axis.set_title('(c) Planar state uncertainty', loc='left')
    style_axis(covariance_axis)

    imm_axis = fig.add_subplot(grid[1, 1])
    imm_rows = [row for row in debug if isinstance(row.get('imm_probabilities'), list)]
    imm_time = np.asarray([row['stamp'] - start for row in imm_rows], dtype=float)
    probabilities = np.asarray([row['imm_probabilities'] for row in imm_rows], dtype=float)
    if len(probabilities):
        imm_axis.stackplot(
            imm_time, probabilities[:, 0], probabilities[:, 1], probabilities[:, 2],
            colors=[COLORS['blue'], COLORS['green'], '#CC6677'], alpha=0.85,
            labels=['low dynamics', 'nominal', 'maneuvering'])
    imm_axis.set_ylim(0, 1)
    imm_axis.set_xlabel('time since primary target lock [s]')
    imm_axis.set_ylabel('model probability')
    imm_axis.set_title('(d) Three-model IMM posterior', loc='left')
    imm_axis.legend(loc='upper right', frameon=True)
    style_axis(imm_axis)

    accepted_nis = [
        row['kalman_nis_xy'] for row in debug
        if row['measurement_accepted'] and row['measurement_source'] not in ('none', 'prediction')]
    draw_footer(fig, [
        ('REPLAY', f"{short_bag(record)} · primary logical target {summary['primary_target_id']}"),
        ('MEASUREMENTS', f"{sum(accepted)} accepted of {len(debug)} debug updates"),
        ('ACCEPTED NIS', f"median {np.median(accepted_nis):.3f} · max {np.max(accepted_nis):.3f}"),
        ('LIMITATION', 'NIS uses filter innovation; no external trajectory ground truth'),
    ])
    metadata = {
        'source': 'recorded_rosbag_replay',
        'bag': record['bag'], 'run_index': record['run_index'],
        'primary_target_id': summary['primary_target_id'],
        'debug_update_count': len(debug),
        'measurement_decisions': dict(decisions),
        'accepted_nis_summary': percentile_summary(accepted_nis),
        'covariance_trace_xy_summary': percentile_summary(covariance.tolist()),
        'nis_gates': {'camera_anchor': 25.0, 'selected_tracklet': 9.21},
        'imm_sample_count': len(imm_rows),
        'ground_truth_available': False,
    }
    save_figure(fig, output_dir, 'human-following-filter-consistency-v1', metadata)


def render_four_bag_benchmark(records, output_dir):
    summaries = [summarize_run(record) for record in records]
    labels = [short_bag(record) for record in records]
    x = np.arange(len(records))
    width = 0.24
    fig = plt.figure(figsize=(17.2, 6.8), facecolor='white')
    grid = fig.add_gridspec(1, 3, left=0.06, right=0.98, top=0.91, bottom=0.21,
                           wspace=0.25)

    duration_axis = fig.add_subplot(grid[0, 0])
    duration_axis.bar(x - width, [r['recorded_duration_sec'] for r in records], width,
                      color='#B5BCC1', label='recorded duration')
    duration_axis.bar(x, [s['locked_duration_sec'] for s in summaries], width,
                      color=COLORS['green'], label='camera-locked duration')
    duration_axis.bar(x + width, [s['confirmed_association_duration_sec'] for s in summaries],
                      width, color=COLORS['orange'], label='confirmed association')
    duration_axis.set_xticks(x, labels)
    duration_axis.set_ylabel('duration [s]')
    duration_axis.set_title('(a) Episode timing by recorded bag', loc='left')
    duration_axis.legend(loc='upper right', frameon=True)
    style_axis(duration_axis)

    source_axis = fig.add_subplot(grid[0, 1])
    sources = ['CAMERA_LIDAR', 'CAMERA_ONLY', 'LIDAR_ONLY', 'PREDICTION_ONLY', 'NONE']
    source_labels = ['camera + LiDAR', 'camera only', 'LiDAR only', 'prediction only', 'none']
    source_colors = [COLORS['camera_lidar'], COLORS['camera_only'], COLORS['lidar_only'],
                     COLORS['prediction'], COLORS['none']]
    bottom = np.zeros(len(records))
    for source, label, color in zip(sources, source_labels, source_colors):
        values = np.asarray([s['source_fractions'].get(source, 0.0) for s in summaries])
        source_axis.bar(x, values, bottom=bottom, color=color, width=0.72, label=label)
        bottom += values
    source_axis.set_xticks(x, labels)
    source_axis.set_ylim(0, 1.0)
    source_axis.set_ylabel('fraction of primary target episode')
    source_axis.set_title('(b) Measurement-source occupancy', loc='left')
    source_axis.legend(loc='upper right', frameon=True, fontsize=7.2)
    style_axis(source_axis)

    table_axis = fig.add_subplot(grid[0, 2])
    table_axis.axis('off')
    table_axis.set_title('(c) Identity continuity and release evidence', loc='left')
    columns = ['bag', 'lock', 'first lock', 'switches', 'release', 'assoc. n']
    cell_text = []
    for label, summary in zip(labels, summaries):
        cell_text.append([
            label, 'PASS' if summary['locked'] else 'FAIL',
            f"{summary['first_lock_sec']:.2f} s" if summary['first_lock_sec'] is not None else '—',
            str(summary['selected_tracklet_switches']),
            'observed' if summary['safe_release'] else 'not observed',
            str(summary['association_samples']),
        ])
    table = table_axis.table(
        cellText=cell_text, colLabels=columns, cellLoc='center', colLoc='center',
        loc='center', colWidths=[0.17, 0.12, 0.17, 0.14, 0.20, 0.14])
    table.auto_set_font_size(False)
    table.set_fontsize(7.7)
    table.scale(1.05, 2.0)
    for (row, col), cell in table.get_celld().items():
        cell.set_edgecolor('white')
        if row == 0:
            cell.set_facecolor('#DDE1E4')
            cell.set_text_props(weight='bold', color=COLORS['ink'])
        else:
            cell.set_facecolor('#F5F6F6' if row % 2 else '#EAECEE')
            if col == 1:
                cell.set_text_props(color='#1B7837', weight='bold')
            if col == 4 and cell.get_text().get_text() == 'not observed':
                cell.set_text_props(color='#B36B00', weight='bold')

    draw_footer(fig, [
        ('PLAYBACK', 'four bags · independent fresh launch · rate 0.5×'),
        ('LOCK SUCCESS', f"{sum(s['locked'] for s in summaries)}/{len(summaries)} bags"),
        ('SAFE RELEASE', f"{sum(s['safe_release'] for s in summaries)}/{len(summaries)} observed in recording window"),
        ('INTERPRETATION', 'not-observed release is not asserted as unsafe behavior'),
    ])
    metadata = {
        'source': 'four_recorded_rosbag_replays_run1',
        'playback_rate': 0.5,
        'bags': summaries,
        'safe_release_definition': 'NO_TARGET state observed after the primary target lock episode',
        'note': 'A release not observed before the bag/capture window ends is reported as not observed, not as a failure.',
    }
    save_figure(fig, output_dir, 'human-following-four-bag-benchmark-v1', metadata)


def render_repeatability(records, output_dir):
    aligned = align_repeatability(records)
    summaries = [summarize_run(record) for record in records]
    fig = plt.figure(figsize=(17.2, 8.5), facecolor='white')
    grid = fig.add_gridspec(2, 2, left=0.065, right=0.98, top=0.95, bottom=0.16,
                           hspace=0.30, wspace=0.23)

    trajectory_axis = fig.add_subplot(grid[0, 0])
    for index, run in enumerate(aligned):
        trajectory_axis.plot(run['y_m'], run['x_m'], color=RUN_COLORS[index], linewidth=1.5,
                             alpha=0.86, label=f"run {run['run_index']}")
        trajectory_axis.scatter([run['y_m'][0]], [run['x_m'][0]], color=RUN_COLORS[index],
                                s=24, edgecolor='white', linewidth=0.5)
    trajectory_axis.set_xlabel('lateral position y [m]')
    trajectory_axis.set_ylabel('forward position x [m]')
    trajectory_axis.set_aspect('equal', adjustable='datalim')
    trajectory_axis.set_title('(a) Aligned physical target trajectories', loc='left')
    trajectory_axis.legend(loc='best', frameon=True, ncol=2)
    style_axis(trajectory_axis)

    common_end = min(max(run['time_sec']) for run in aligned)
    grid_time = np.linspace(0.0, common_end, 160)
    x_stack = np.asarray([np.interp(grid_time, run['time_sec'], run['x_m']) for run in aligned])
    y_stack = np.asarray([np.interp(grid_time, run['time_sec'], run['y_m']) for run in aligned])
    range_stack = np.asarray([np.interp(grid_time, run['time_sec'], run['range_m']) for run in aligned])
    median_x, median_y = np.median(x_stack, axis=0), np.median(y_stack, axis=0)
    rms_deviation = np.sqrt(np.mean(
        (x_stack - median_x) ** 2 + (y_stack - median_y) ** 2, axis=1))

    range_axis = fig.add_subplot(grid[0, 1])
    for index, run in enumerate(aligned):
        range_axis.plot(run['time_sec'], run['range_m'], color=RUN_COLORS[index],
                        linewidth=1.0, alpha=0.42)
    range_mean = np.mean(range_stack, axis=0)
    range_std = np.std(range_stack, axis=0)
    range_axis.plot(grid_time, range_mean, color=COLORS['ink'], linewidth=2.0,
                    label='five-run mean')
    range_axis.fill_between(grid_time, range_mean - range_std, range_mean + range_std,
                            color=COLORS['green'], alpha=0.20, label='±1 standard deviation')
    range_axis.set_xlabel('time since primary target lock [s]')
    range_axis.set_ylabel('target range [m]')
    range_axis.set_title('(b) Range repeatability', loc='left')
    range_axis.legend(loc='best', frameon=True)
    style_axis(range_axis)

    metric_axis = fig.add_subplot(grid[1, 0])
    x = np.arange(len(records))
    width = 0.34
    metric_axis.bar(x - width / 2, [s['first_lock_sec'] for s in summaries], width,
                    color=COLORS['blue'], label='first lock')
    metric_axis.bar(x + width / 2,
                    [s['confirmed_association_duration_sec'] for s in summaries], width,
                    color=COLORS['orange'], label='confirmed association')
    metric_axis.set_xticks(x, [f"run {r['run_index']}" for r in records])
    metric_axis.set_ylabel('time [s]')
    metric_axis.set_title('(c) Run-level timing stability', loc='left')
    metric_axis.legend(loc='upper left', frameon=True)
    style_axis(metric_axis)

    identity_axis = fig.add_subplot(grid[1, 1])
    identity_axis.axis('off')
    identity_axis.set_title('(d) Run-local identity and trajectory deviation', loc='left')
    columns = ['run', 'tracklet IDs', 'switches', 'safe release', 'RMS dev.']
    cell_text = []
    for run, summary, deviation in zip(aligned, summaries, rms_deviation):
        cell_text.append([
            str(run['run_index']), ', '.join(f"T{value}" for value in run['tracklet_ids']),
            str(summary['selected_tracklet_switches']),
            'yes' if summary['safe_release'] else 'no',
            f'{deviation * 100:.1f} cm',
        ])
    table = identity_axis.table(
        cellText=cell_text, colLabels=columns, cellLoc='center', colLoc='center',
        loc='center', colWidths=[0.10, 0.25, 0.14, 0.20, 0.17])
    table.auto_set_font_size(False)
    table.set_fontsize(8.0)
    table.scale(1.0, 1.75)
    for (row, col), cell in table.get_celld().items():
        cell.set_edgecolor('white')
        cell.set_facecolor('#DDE1E4' if row == 0 else ('#F5F6F6' if row % 2 else '#EAECEE'))
        if row == 0:
            cell.set_text_props(weight='bold')
    identity_axis.text(
        0.04, 0.08,
        'Tracklet numbers are allocator-local and are not expected to match across launches.',
        transform=identity_axis.transAxes, fontsize=8.2, color=COLORS['purple'], weight='bold')

    first_lock_values = [s['first_lock_sec'] for s in summaries]
    association_values = [s['confirmed_association_duration_sec'] for s in summaries]
    draw_footer(fig, [
        ('REPLAY', '145900 · five fresh launches · rate 0.5×'),
        ('SUCCESS', f"{sum(s['locked'] for s in summaries)}/5 lock · {sum(s['safe_release'] for s in summaries)}/5 release"),
        ('FIRST LOCK', f"mean {np.mean(first_lock_values):.2f} s · std {np.std(first_lock_values):.2f} s"),
        ('TRAJECTORY', f"mean RMS deviation {np.mean(rms_deviation) * 100:.1f} cm"),
    ])
    metadata = {
        'source': 'five_independent_replays',
        'bag': records[0]['bag'], 'playback_rate': 0.5,
        'run_summaries': summaries,
        'trajectory_rms_deviation_m': rms_deviation.tolist(),
        'mean_trajectory_rms_deviation_m': float(np.mean(rms_deviation)),
        'first_lock_mean_sec': float(np.mean(first_lock_values)),
        'first_lock_std_sec': float(np.std(first_lock_values)),
        'confirmed_association_mean_sec': float(np.mean(association_values)),
        'confirmed_association_std_sec': float(np.std(association_values)),
        'tracklet_id_scope': 'run_local',
        'ground_truth_available': False,
    }
    save_figure(fig, output_dir, 'human-following-replay-repeatability-v1', metadata)


def render_funnel(records, output_dir):
    funnel = aggregate_funnel(records, score_threshold=0.65)
    counts = np.asarray(funnel['counts'], dtype=float)
    labels = [label.replace('\n', ' ') for label in FUNNEL_LABELS]
    conversions = np.ones_like(counts)
    conversions[1:] = np.divide(
        counts[1:], counts[:-1], out=np.zeros_like(counts[1:]), where=counts[:-1] > 0)
    fig = plt.figure(figsize=(15.8, 7.0), facecolor='white')
    grid = fig.add_gridspec(1, 2, left=0.07, right=0.97, top=0.90, bottom=0.22,
                           width_ratios=(1.35, 0.65), wspace=0.20)
    funnel_axis = fig.add_subplot(grid[0, 0])
    y = np.arange(len(counts))
    normalized = counts / counts[0] if counts[0] else counts
    colors = ['#A6CEE3', '#80B1D3', '#66C2A5', '#41AE76', '#FDD87D', '#F4A261', '#CC3311']
    funnel_axis.barh(y, normalized, color=colors, height=0.68)
    funnel_axis.set_yticks(y, labels)
    funnel_axis.invert_yaxis()
    funnel_axis.set_xlim(0, 1.08)
    funnel_axis.set_xlabel('retained fraction of confirmed candidate evaluations')
    funnel_axis.set_title('(a) Episode-level association evidence funnel', loc='left')
    for index, (fraction, count) in enumerate(zip(normalized, counts.astype(int))):
        funnel_axis.text(min(fraction + 0.018, 0.94), index,
                         f'{count}  ({fraction * 100:.1f}%)', va='center',
                         fontsize=8.4, weight='bold')
    style_axis(funnel_axis)

    conversion_axis = fig.add_subplot(grid[0, 1])
    transition_x = np.arange(1, len(conversions))
    transition_labels = [
        'cand.→XY', 'XY→range', 'range→proj.',
        'proj.→pub.', 'pub.→score', 'score→select',
    ]
    conversion_axis.plot(transition_x, conversions[1:] * 100,
                         marker='o', color=COLORS['purple'], linewidth=2.0)
    conversion_axis.set_xticks(transition_x, transition_labels, rotation=18, ha='right')
    conversion_axis.set_ylim(0, 105)
    conversion_axis.set_ylabel('stage-to-stage retention [%]')
    conversion_axis.set_title('(b) Conditional retention', loc='left')
    for index, value in enumerate(conversions[1:] * 100, start=1):
        conversion_axis.text(index, value + 3, f'{value:.1f}%', ha='center', fontsize=8.1)
    style_axis(conversion_axis)

    updates = [row for record in records for row in record.get('funnel_updates', [])]
    upstream = {
        'debug_updates': len(updates),
        'raw_candidate_clusters_total': int(sum(row.get('raw_candidate_clusters', 0) for row in updates)),
        'active_tracklet_observations_total': int(sum(row.get('active_tracklets', 0) for row in updates)),
    }
    draw_footer(fig, [
        ('EVIDENCE', f"four bags · {upstream['debug_updates']} synchronized debug updates"),
        ('UPSTREAM CONTEXT', f"{upstream['raw_candidate_clusters_total']} raw clusters · {upstream['active_tracklet_observations_total']} active-track observations"),
        ('FUNNEL START', f"{int(counts[0])} confirmed candidate evaluations"),
        ('FINAL SELECTION', f"{int(counts[-1])} updates passed score 0.65 and matched selected ID"),
    ])
    metadata = {
        'source': 'four_recorded_rosbag_replays_run1',
        'funnel': funnel,
        'stage_to_stage_retention': conversions.tolist(),
        'upstream_context': upstream,
        'note': 'The monotonic funnel begins at confirmed candidate evaluations. Raw clusters and active tracks are context totals because persistent tracks are not a one-to-one stage of each debug update.',
    }
    save_figure(fig, output_dir, 'human-following-association-funnel-v1', metadata)


def main():
    args = parse_args()
    configure_style()
    files = sorted(Path(args.data_dir).glob('*.json'))
    records = [json.loads(path.read_text(encoding='utf-8')) for path in files]
    if not records:
        raise SystemExit('No normalized replay JSON records found')
    benchmark_by_bag = {}
    for record in records:
        if int(record.get('run_index', 1)) == 1:
            benchmark_by_bag[record['bag']] = record
    benchmark = [benchmark_by_bag[key] for key in sorted(benchmark_by_bag)]
    repeatability = sorted(
        [record for record in records if record['bag'].endswith('145900')],
        key=lambda row: int(row['run_index']))
    if len(benchmark) != 4:
        raise SystemExit(f'Expected four benchmark bags, found {len(benchmark)}')
    if len(repeatability) != 5:
        raise SystemExit(f'Expected five repeatability runs, found {len(repeatability)}')

    render_association_statistics(benchmark, args.output_dir)
    render_filter_consistency(repeatability[0], args.output_dir)
    render_four_bag_benchmark(benchmark, args.output_dir)
    render_repeatability(repeatability, args.output_dir)
    render_funnel(benchmark, args.output_dir)
    print(json.dumps({
        'benchmark_bags': len(benchmark), 'repeatability_runs': len(repeatability),
        'figures': 5,
    }))


if __name__ == '__main__':
    main()
