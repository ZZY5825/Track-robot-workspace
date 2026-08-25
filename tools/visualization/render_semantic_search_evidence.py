#!/usr/bin/env python3
"""Render a publication-style semantic-search evidence summary."""

import argparse
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-semantic-search-evidence')

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
import numpy as np


COLORS = {
    'phase0': '#8DA0CB', 'phase1': '#66C2A5', 'phase2': '#A6D854',
    'phase3': '#FFD92F', 'phase4': '#FC8D62', 'advisory': '#E78AC3',
    'ink': '#202428', 'muted': '#5B6268', 'line': '#C8CDD1',
}


def parse_args():
    repo = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--metadata-output', required=True)
    parser.add_argument('--overlay', default=str(
        repo / 'track_robot_ws/artifacts/semantic-search/'
        'phase1-mws-green-bottle-2026-07-27-rerun/phase1_overlay.png'))
    parser.add_argument('--phase1-report', default=str(
        repo / 'track_robot_ws/artifacts/semantic-search/'
        'phase1-mws-green-bottle-2026-07-27-rerun/report.json'))
    parser.add_argument('--phase4-report', default=str(
        repo / 'track_robot_ws/artifacts/semantic-search/'
        'phase4a-green-bottle-2026-07-28/phase4a_validation.json'))
    return parser.parse_args()


def draw_box(axis, x, y, width, height, title, detail, color):
    patch = FancyBboxPatch(
        (x, y), width, height, boxstyle='round,pad=0.012,rounding_size=0.015',
        facecolor=color, edgecolor='white', linewidth=1.2)
    axis.add_patch(patch)
    axis.text(x + width / 2, y + height * 0.62, title, ha='center', va='center',
              fontsize=9.1, weight='bold', color=COLORS['ink'])
    axis.text(x + width / 2, y + height * 0.28, detail, ha='center', va='center',
              fontsize=7.8, color=COLORS['ink'])


def render(args):
    phase1 = json.loads(Path(args.phase1_report).read_text(encoding='utf-8'))
    validation = json.loads(Path(args.phase4_report).read_text(encoding='utf-8'))
    image = plt.imread(args.overlay)
    phases = validation['phases']

    metrics = [
        ('visual obs.', phases['phase1']['evidence']['observation_count'], COLORS['phase1']),
        ('LiDAR tracklets', phases['phase2']['evidence']['lidar_tracklet_messages'], COLORS['phase2']),
        ('valid poses', phases['phase2']['evidence']['valid_position_samples'], '#7FC97F'),
        ('target selections', phases['phase3']['evidence']['selected_target_count_total'], COLORS['phase3']),
        ('planned paths', phases['phase4']['evidence']['planned_messages'], COLORS['phase4']),
        ('ready advice', phases['phase4a_advisory']['evidence']['ready_advice_messages'], COLORS['advisory']),
    ]

    plt.rcParams.update({
        'font.family': 'DejaVu Sans', 'font.size': 9.4,
        'axes.titlesize': 11.3, 'axes.labelsize': 9.4,
    })
    fig = plt.figure(figsize=(17.2, 6.4), facecolor='white')
    grid = fig.add_gridspec(2, 3, height_ratios=(8.2, 1.15), hspace=0.18, wspace=0.36)
    qualitative = fig.add_subplot(grid[0, 0])
    chain = fig.add_subplot(grid[0, 1])
    counts = fig.add_subplot(grid[0, 2])
    footer = fig.add_subplot(grid[1, :])

    qualitative.imshow(image)
    qualitative.axis('off')
    qualitative.set_title('(a) Query-conditioned object proposal', loc='left')
    qualitative.text(
        0.02, 0.04,
        f"query: \"{phase1['query']['text']}\"  ·  max score {phase1['metrics']['score']['maximum']:.3f}",
        transform=qualitative.transAxes, fontsize=8.6, color='white', weight='bold',
        bbox={'boxstyle': 'round,pad=0.35', 'facecolor': '#202428CC', 'edgecolor': 'none'})

    chain.set_xlim(0, 1)
    chain.set_ylim(0, 1)
    chain.axis('off')
    chain.set_title('(b) Cross-phase evidence chain', loc='left')
    box_specs = [
        (0.03, 0.69, 'Phase 0', 'localization\n251 messages', COLORS['phase0']),
        (0.36, 0.69, 'Phase 1', 'semantic regions\n19 observations', COLORS['phase1']),
        (0.69, 0.69, 'Phase 2', 'bounded memory\nglobal ID 17', COLORS['phase2']),
        (0.20, 0.29, 'Phase 3', 'target selection\n34 samples', COLORS['phase3']),
        (0.53, 0.29, 'Phase 4', 'approach planning\n17 paths', COLORS['phase4']),
        (0.76, 0.035, 'Safety', 'advisory only\n0 cmd_vel pubs', COLORS['advisory']),
    ]
    for x, y, title, detail, color in box_specs:
        draw_box(chain, x, y, 0.27 if title != 'Safety' else 0.22,
                 0.19, title, detail, color)
    arrows = [
        ((0.30, 0.785), (0.36, 0.785)), ((0.63, 0.785), (0.69, 0.785)),
        ((0.82, 0.68), (0.48, 0.49)), ((0.47, 0.29), (0.53, 0.385)),
        ((0.80, 0.29), (0.85, 0.225)),
    ]
    for start, end in arrows:
        chain.annotate('', xy=end, xytext=start,
                       arrowprops={'arrowstyle': '-|>', 'color': '#697176', 'lw': 1.5})
    chain.text(0.03, 0.06, 'cross-phase contract: PASS', color='#1B7837',
               fontsize=9.2, weight='bold')

    labels = [row[0] for row in metrics]
    values = [row[1] for row in metrics]
    colors = [row[2] for row in metrics]
    y = np.arange(len(metrics))
    counts.barh(y, values, color=colors, height=0.62)
    counts.set_yticks(y, labels)
    counts.invert_yaxis()
    counts.set_xscale('log')
    counts.set_xlabel('observed sample/message count [log scale]')
    counts.grid(True, axis='x', which='both', color='#E0E3E5', linewidth=0.55)
    counts.set_facecolor('#FAFAF9')
    counts.set_title('(c) Recorded end-to-end activity', loc='left')
    for index, value in enumerate(values):
        counts.text(value * 1.08, index, str(value), va='center', fontsize=8.4,
                    weight='bold', color=COLORS['ink'])
    counts.set_xlim(8, max(values) * 1.7)

    footer.axis('off')
    footer.axhline(0.98, color=COLORS['line'], linewidth=0.8)
    blocks = [
        ('QUALITATIVE RUN', '2026-07-27 · Phase 1 PASS · 40/40 nonempty'),
        ('SYSTEM VALIDATION', '2026-07-28 · Phase 0–4A PASS'),
        ('PLANNER TIMING', f"p50 {phases['phase4']['latency_ms']['planner_p50']:.1f} ms · p95 {phases['phase4']['latency_ms']['planner_p95']:.1f} ms"),
        ('SAFETY MODE', 'planning only · advisory only · no motion output'),
    ]
    for index, (heading, value) in enumerate(blocks):
        x = 0.01 + index * 0.25
        footer.text(x, 0.66, heading, transform=footer.transAxes, fontsize=8,
                    weight='bold', color=COLORS['muted'])
        footer.text(x, 0.22, value, transform=footer.transAxes, fontsize=8.7,
                    weight='bold', color=COLORS['ink'])

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=190, bbox_inches='tight', facecolor='white')
    fig.savefig(output.with_suffix('.pdf'), bbox_inches='tight', facecolor='white')
    plt.close(fig)

    metadata = {
        'sources': {
            'qualitative_overlay': str(Path(args.overlay)),
            'phase1_report': str(Path(args.phase1_report)),
            'phase0_4a_validation': str(Path(args.phase4_report)),
        },
        'qualitative_query': phase1['query'],
        'qualitative_metrics': phase1['metrics'],
        'cross_phase_consistency': validation['cross_phase_consistency'],
        'activity_counts': {label: value for label, value, _ in metrics},
        'planner_latency_ms': phases['phase4']['latency_ms'],
        'safety': validation['safety'],
        'note': 'Panels (a) and (b–c) summarize two separately dated green-bottle validation runs.',
    }
    Path(args.metadata_output).write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')


def main():
    args = parse_args()
    render(args)
    print(json.dumps({'output': args.output}))


if __name__ == '__main__':
    main()
