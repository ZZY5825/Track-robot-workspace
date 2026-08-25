#!/usr/bin/env python3
"""Render planned human-following evaluation protocols without inventing results."""

import argparse
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-human-evaluation-protocol')

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle
import numpy as np


COLORS = {
    'robot': '#30363B', 'target': '#CC3311', 'distractor': '#4477AA',
    'occluder': '#8C8C8C', 'safe': '#2A7F76', 'pending': '#B36B00',
    'grid': '#D9DDE0', 'ink': '#202428', 'muted': '#5B6268',
}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--metadata-output', required=True)
    return parser.parse_args()


def robot(axis, xy, heading=0.0):
    marker = np.asarray([[0.23, 0], [-0.17, 0.15], [-0.17, -0.15]])
    rotation = np.asarray([[np.cos(heading), -np.sin(heading)],
                           [np.sin(heading), np.cos(heading)]])
    marker = marker @ rotation.T + np.asarray(xy)
    axis.fill(marker[:, 0], marker[:, 1], color=COLORS['robot'], zorder=5)


def person(axis, xy, color, label):
    axis.add_patch(Circle(xy, 0.13, facecolor=color, edgecolor='white', linewidth=1.0, zorder=6))
    axis.text(xy[0], xy[1] - 0.28, label, ha='center', fontsize=7.6,
              weight='bold', color=color)


def setup(axis, title):
    axis.set_xlim(0, 6)
    axis.set_ylim(0, 4)
    axis.set_aspect('equal', adjustable='box')
    axis.set_xticks([])
    axis.set_yticks([])
    axis.set_facecolor('#FAFAF9')
    axis.grid(True, color=COLORS['grid'], linewidth=0.5)
    axis.set_title(title, loc='left')


def render(args):
    plt.rcParams.update({
        'font.family': 'DejaVu Sans', 'font.size': 9.4,
        'axes.titlesize': 11.2,
    })
    fig = plt.figure(figsize=(17.2, 6.25), facecolor='white')
    grid = fig.add_gridspec(2, 3, height_ratios=(7.9, 1.25), hspace=0.18, wspace=0.16)
    occlusion = fig.add_subplot(grid[0, 0])
    crossing = fig.add_subplot(grid[0, 1])
    control = fig.add_subplot(grid[0, 2])
    footer = fig.add_subplot(grid[1, :])

    setup(occlusion, '(a) Occlusion and identity recovery')
    robot(occlusion, (0.8, 2.0))
    occlusion.add_patch(Rectangle((2.6, 0.65), 0.55, 2.7,
                                  facecolor=COLORS['occluder'], alpha=0.82))
    occlusion.plot([1.4, 2.4], [2.0, 2.0], color=COLORS['target'], lw=2.2)
    occlusion.plot([3.35, 5.2], [2.0, 2.0], color=COLORS['target'], lw=2.2)
    occlusion.plot([2.4, 3.35], [2.0, 2.0], color=COLORS['target'], lw=2.2,
                   linestyle='--', alpha=0.5)
    occlusion.annotate('', xy=(5.25, 2.0), xytext=(1.4, 2.0),
                       arrowprops={'arrowstyle': '-|>', 'color': COLORS['target'], 'lw': 1.6})
    person(occlusion, (1.6, 2.0), COLORS['target'], 'target')
    person(occlusion, (4.8, 2.0), COLORS['target'], 'reacquired')
    occlusion.text(2.88, 3.55, 'opaque occluder', ha='center', fontsize=8,
                   color=COLORS['muted'], weight='bold')
    occlusion.text(0.25, 0.18, 'Measure: identity preserved · reacquisition time · covariance growth',
                   fontsize=7.8, color=COLORS['ink'])

    setup(crossing, '(b) Multi-person crossing and ID stability')
    robot(crossing, (0.7, 0.7), heading=0.55)
    crossing.plot([1.2, 5.0], [1.0, 3.2], color=COLORS['target'], lw=2.2)
    crossing.plot([1.2, 5.0], [3.2, 1.0], color=COLORS['distractor'], lw=2.2)
    crossing.annotate('', xy=(5.05, 3.23), xytext=(4.6, 2.97),
                      arrowprops={'arrowstyle': '-|>', 'color': COLORS['target'], 'lw': 1.5})
    crossing.annotate('', xy=(5.05, 0.97), xytext=(4.6, 1.23),
                      arrowprops={'arrowstyle': '-|>', 'color': COLORS['distractor'], 'lw': 1.5})
    person(crossing, (1.35, 1.08), COLORS['target'], 'target')
    person(crossing, (1.35, 3.12), COLORS['distractor'], 'distractor')
    crossing.add_patch(Circle((3.1, 2.1), 0.43, fill=False,
                              edgecolor=COLORS['pending'], linestyle='--', linewidth=1.5))
    crossing.text(3.1, 2.68, 'ambiguity zone', ha='center', fontsize=8,
                  color=COLORS['pending'], weight='bold')
    crossing.text(0.25, 0.18, 'Measure: ID switches · wrong-person lock · score margin · recovery',
                  fontsize=7.8, color=COLORS['ink'])

    setup(control, '(c) Closed-loop following and safe stop')
    robot(control, (0.9, 2.0))
    person(control, (4.8, 2.0), COLORS['target'], 'target')
    control.annotate('', xy=(4.55, 2.0), xytext=(1.25, 2.0),
                     arrowprops={'arrowstyle': '<->', 'color': COLORS['safe'], 'lw': 1.8})
    control.text(2.9, 2.17, 'measured range d(t)', ha='center', fontsize=8.2,
                 color=COLORS['safe'], weight='bold')
    control.axvspan(3.9, 5.7, ymin=0.18, ymax=0.34, color='#FDE0DD', alpha=0.9)
    control.text(4.8, 1.05, 'stop / comfort zone', ha='center', fontsize=8,
                 color=COLORS['target'], weight='bold')
    control.plot([1.2, 2.1, 3.0, 3.8], [1.65, 1.65, 1.72, 1.82],
                 color=COLORS['safe'], lw=2.0)
    control.annotate('', xy=(3.9, 1.85), xytext=(3.45, 1.78),
                     arrowprops={'arrowstyle': '-|>', 'color': COLORS['safe'], 'lw': 1.5})
    control.text(0.25, 0.18, 'Measure: range RMSE · command latency · min clearance · stop distance',
                 fontsize=7.8, color=COLORS['ink'])

    footer.axis('off')
    footer.axhline(0.98, color='#C8CDD1', linewidth=0.8)
    footer.text(0.01, 0.62, 'PLANNED EXPERIMENTS', transform=footer.transAxes,
                fontsize=8, weight='bold', color=COLORS['pending'])
    footer.text(0.01, 0.18,
                'Dedicated recordings are not present in the repository; these panels define capture geometry and metrics, not measured results.',
                transform=footer.transAxes, fontsize=9.1, weight='bold', color=COLORS['ink'])
    footer.text(0.76, 0.62, 'REQUIRED DATA', transform=footer.transAxes,
                fontsize=8, weight='bold', color=COLORS['muted'])
    footer.text(0.76, 0.18, 'camera · LiDAR · fused target · session · odometry · cmd_vel',
                transform=footer.transAxes, fontsize=8.8, weight='bold', color=COLORS['ink'])

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=190, bbox_inches='tight', facecolor='white')
    fig.savefig(output.with_suffix('.pdf'), bbox_inches='tight', facecolor='white')
    plt.close(fig)

    metadata = {
        'source': 'evaluation_protocol_only',
        'measured_results': False,
        'status': 'DATA_COLLECTION_PENDING',
        'protocols': {
            'occlusion_recovery': ['identity_preservation', 'reacquisition_time', 'covariance_growth'],
            'multi_person_crossing': ['id_switches', 'wrong_person_lock', 'score_margin', 'recovery'],
            'closed_loop_following': ['range_rmse', 'command_latency', 'minimum_clearance', 'stop_distance'],
        },
        'required_topics': ['camera', 'lidar', 'fused_target', 'session', 'odometry', 'cmd_vel'],
    }
    Path(args.metadata_output).write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')


def main():
    args = parse_args()
    render(args)
    print(json.dumps({'output': args.output, 'status': 'DATA_COLLECTION_PENDING'}))


if __name__ == '__main__':
    main()
