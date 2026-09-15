# Project Directory and File Guide

Inventory checked on 2026-09-16. This guide describes locations and ownership;
it does not certify that a feature has passed hardware validation.

## Addresses and Path Mapping

| Location | Address or path |
| --- | --- |
| GitHub repository | https://github.com/ZZY5825/Track-robot-workspace |
| Git clone URL | https://github.com/ZZY5825/Track-robot-workspace.git |
| Thesis repository | https://github.com/ZZY5825/agilex-bunker-thesis |
| Current robot workspace | /home/track-robot/track_robot_ws |
| ROS installation on this robot | /opt/ros/foxy |
| Workspace inside a fresh repository clone | Track-robot-workspace/track_robot_ws |

The repository root is **not** the ROS workspace root. For example,
repository `track_robot_ws/src/track_robot_perception/` corresponds to
local `~/track_robot_ws/src/track_robot_perception/`.

The current local workspace and GitHub main have different development
states. A local path is not evidence that its contents have been published.
Use the commit history to identify a published version.

## Repository Root

| Path from repository root | Classification and purpose |
| --- | --- |
| README.md | Public project overview, demonstrations and entry points |
| RELEASES.md | Published release notes |
| docs/ | Repository-level documentation and presentation assets |
| artifacts/ | Repository-level demonstration and supporting assets |
| src/bunker_pro2/ | Bunker robot description and visualization package |
| src/piper_description/ | PiPER arm description and visualization package |
| track_robot_ws/ | ROS workspace sources, guides and selected evidence |

Root-level `src/` and `track_robot_ws/src/` are distinct. Do not merge them
or build duplicate copies of a package. The current robot also has a local
`~/track_robot_ws/src/bunker_pro2/`; GitHub publishes that model at root
`src/bunker_pro2/`. Consult its package README before installing it.

## Workspace Files

Paths below are relative to `~/track_robot_ws/` on the robot, or
`track_robot_ws/` in the repository unless marked local-only.

| Path | File category | Storage rule |
| --- | --- | --- |
| src/ | ROS source packages | Version reviewed source and reusable configuration |
| docs/guides/ | Current operating and replay procedures | Version; use for operating instructions |
| docs/architecture/ | Architecture and design decisions | Version; designs are not proof of completion |
| docs/development/ | Dated implementation plans | Version as development history |
| docs/superpowers/ | Additional dated specifications and plans | Retain existing references |
| tools/ | Standalone capture and model-evaluation utilities | Version source, not generated caches |
| config/ | Configuration guidance and machine-specific settings | See config/README.md |
| config/local/ | Generated runtime configuration | Local-only |
| config/*.measured.yaml | Robot-specific measured calibration | Local-only by default |
| artifacts/ | Reports, manifests, figures and experiment outputs | Commit selected small evidence after review |
| rosbags/ | Recordings and recording metadata | Keep raw payloads local; version selected metadata |
| models/ | Model checkpoints and isolated runtimes | Local-only; do not upload weights by default |
| dataset/ | Dataset and CAD inputs | Local-only by default |
| simulation/ | Local simulation work | Check availability in the target revision |
| dinov3_feature_outputs/ | Generated model features | Local-only |
| build/, install/, log/ | Colcon outputs | Generated; do not edit or commit |
| agilex-bunker-thesis/ | Local nested thesis checkout | Publish through its own repository |
| .local-git/, .worktrees/ | Local Git metadata and worktrees | Local-only; never copy into a publication |
| .agents/, .codex/, .superpowers/ | Agent workflow state | Local-only |

Both `artifacts/semantic_search/` and `artifacts/semantic-search/` exist.
Keep existing paths intact: the former contains structured manifests/reports,
while the latter includes dated validation runs. Record new evidence under the
path expected by its generating tool and link it from the relevant guide.

## Source Package Map

| Workspace-relative path | Responsibility |
| --- | --- |
| src/track_robot_perception/ | Camera tracking, gestures, camera/LiDAR integration and LIO configuration |
| src/track_robot_semantic_search/ | Language-conditioned object search and perception |
| src/track_robot/track_robot_bringup/ | System launch files and operator tooling |
| src/track_robot/track_robot_sensor_bringup/ | Sensor startup integration |
| src/track_robot/track_robot_drivers/ | Robot driver integration |
| src/track_robot/track_robot_interfaces/ | Shared ROS messages and services |
| src/track_robot/track_robot_lidar_tracking/ | LiDAR target tracking |
| src/track_robot/track_robot_control/ | Target-following controllers |
| src/track_robot/track_robot_decision/ | Decision logic |
| src/track_robot/track_robot_safety/ | Obstacle and motion safety components |
| src/track_robot/track_robot_semantic_memory/ | Persistent semantic object memory |
| src/track_robot/track_robot_semantic_search_rviz_plugins/ | RViz semantic-search interface |
| src/track_robot/track_robot_navigation/ | Navigation integration published on GitHub; absent from this local snapshot |
| src/third_party_ros/ | Third-party ROS sources, including Point-LIO |
| src/track_robot_core/, src/lidar_mos_filter/ | Separately managed core and filtering source trees |

Some source directories are ignored by the local Git rules but have files
already tracked in the published repository. An ignore rule does not untrack
existing files. Review explicit diffs when updating vendor code.

Within a package: `launch/` holds launch entry points, `config/` reusable
parameters, `src/` and Python modules implementation, `include/` headers,
`msg/` and `srv/` interfaces, `rviz/` visualization settings, `test/` tests,
and `docs/` package-specific explanations.

## Find the Right Document

- [Workspace documentation index](README.md)
- [Operator guide index](guides/README.md)
- [Human-tracking progress](../src/track_robot_perception/docs/human_tracking_progress.md)
- [Human-tracking LiDAR methods](../src/track_robot_perception/docs/lidar_phase4_methods.md)
- [Point-LIO and RS-Helios integration](../src/track_robot_perception/docs/point_lio_rshelios.md)
- [Semantic-search package](../src/track_robot_semantic_search/README.md)
- [Machine-local configuration](../config/README.md)
- [Rosbag storage and replay](../rosbags/README.md)

## Publication Rules

1. Commit source, reusable configurations and documentation together when they
   describe the same reviewed change.
2. Keep raw bags, model weights, build outputs, credentials and machine-local
   settings out of ordinary source commits.
3. Publish only selected experiment evidence, with its input/run provenance.
4. Preserve independent thesis and third-party repository ownership.
5. Documentation-only organization does not require a release tag. Create a
   release when there is a defined, validated software milestone.

This update adds navigation and classification only; it does not relocate
source files, change runtime configuration, or publish pending algorithm work.

