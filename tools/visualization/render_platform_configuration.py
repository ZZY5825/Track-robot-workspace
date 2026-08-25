#!/usr/bin/env python3
"""Render an annotated publication figure of the integrated robot model."""

import argparse
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-platform-configuration')

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch


ACCENT = '#2A7F76'
INK = '#202428'
MUTED = '#5B6268'


def parse_args():
    repo = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--metadata-output', required=True)
    parser.add_argument('--image', default=str(repo / 'docs/assets/readme/track-robot-hero.png'))
    return parser.parse_args()


def render(args):
    image = plt.imread(args.image)
    callouts = [
        (1, 225, 300, 'PiPER arm', '6-DoF mobile manipulation'),
        (2, 160, 190, 'RealSense L515', 'wrist-mounted depth camera'),
        (3, 343, 210, 'ZED stereo camera', 'RGB, pose and depth observations'),
        (4, 402, 166, 'RS-Helios LiDAR', '32-beam geometry and obstacle sensing'),
        (5, 430, 278, 'Sensor station', 'protected sensing / compute enclosure'),
        (6, 440, 460, 'Bunker Pro 2', 'tracked mobile base'),
    ]

    plt.rcParams.update({
        'font.family': 'DejaVu Sans', 'font.size': 9.6,
        'axes.titlesize': 11.5,
    })
    fig = plt.figure(figsize=(15.8, 7.5), facecolor='white')
    grid = fig.add_gridspec(1, 2, width_ratios=(1.45, 1.0), wspace=0.05)
    model = fig.add_subplot(grid[0, 0])
    legend = fig.add_subplot(grid[0, 1])

    model.imshow(image)
    model.axis('off')
    model.set_title('(a) Integrated mobile-manipulation digital model', loc='left')
    for number, x, y, _, _ in callouts:
        model.scatter([x], [y], s=285, color=ACCENT, edgecolor='white',
                      linewidth=1.7, zorder=8)
        model.text(x, y, str(number), ha='center', va='center', color='white',
                   fontsize=9.2, weight='bold', zorder=9)

    legend.set_xlim(0, 1)
    legend.set_ylim(0, 1)
    legend.axis('off')
    legend.set_title('(b) Hardware roles in the autonomy stack', loc='left')
    y_positions = [0.86, 0.72, 0.58, 0.44, 0.30, 0.16]
    for (number, _, _, title, detail), y in zip(callouts, y_positions):
        color = '#D9F0EA' if number in (3, 4, 5) else '#E6E8F3'
        patch = FancyBboxPatch(
            (0.08, y - 0.055), 0.86, 0.105,
            boxstyle='round,pad=0.012,rounding_size=0.012',
            facecolor=color, edgecolor='white', linewidth=1.2)
        legend.add_patch(patch)
        legend.text(0.04, y, str(number), ha='center', va='center', fontsize=9.2,
                    color='white', weight='bold',
                    bbox={'boxstyle': 'circle,pad=0.30', 'facecolor': ACCENT, 'edgecolor': 'white'})
        legend.text(0.12, y + 0.017, title, ha='left', va='center', fontsize=9.4,
                    color=INK, weight='bold')
        legend.text(0.12, y - 0.022, detail, ha='left', va='center', fontsize=8.1,
                    color=MUTED)
    legend.text(0.08, 0.035,
                'Perception: 3–5   ·   Manipulation/mobility: 1–2, 6',
                fontsize=8.2, color=MUTED, weight='bold')

    fig.text(0.015, 0.015,
             'Source: integrated project URDF rendered in RViz · digital model, not a hardware photograph',
             fontsize=8.3, color=MUTED)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=190, bbox_inches='tight', facecolor='white')
    fig.savefig(output.with_suffix('.pdf'), bbox_inches='tight', facecolor='white')
    plt.close(fig)

    metadata = {
        'source_image': str(Path(args.image)),
        'source_type': 'integrated_project_urdf_rviz_render',
        'is_hardware_photograph': False,
        'components': [
            {'number': number, 'label': title, 'role': detail, 'image_xy_px': [x, y]}
            for number, x, y, title, detail in callouts
        ],
    }
    Path(args.metadata_output).write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')


def main():
    args = parse_args()
    render(args)
    print(json.dumps({'output': args.output}))


if __name__ == '__main__':
    main()
