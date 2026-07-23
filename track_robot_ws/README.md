# Track Robot Workspace

ROS 2 Foxy workspace for the Track Robot platform, including sensor bringup,
human tracking, language-conditioned semantic search, persistent semantic
memory, safety components, simulation support, and recorded validation data.

## Start Here

- [Workspace documentation](docs/README.md)
- [Semantic-search Phase 1 and Phase 2 live test guide](docs/guides/semantic-search/phase2-recording-and-evaluation.md)
- [Semantic-search rosbag workflow](docs/guides/semantic-search/rosbag-workflow.md)
- [Human-tracking rosbag replay guide](docs/guides/human-tracking/rosbag-replay.md)

The managed semantic-search tools use **ROS Domain 20**. The modular
semantic-search bringup is passive: it does not authorize or start navigation,
base control, motion controllers, or a `/cmd_vel` publisher.

## Workspace Layout

| Path | Purpose |
| --- | --- |
| `src/` | First-party ROS packages and separately identified third-party source |
| `docs/` | Current guides, architecture, and dated development plans |
| `artifacts/` | Small, versioned manifests, calibration evidence, and evaluation reports |
| `rosbags/` | Local raw rosbag recordings; database payloads are not committed |
| `config/` | Machine-local measured configuration |
| `models/` | Local model checkpoints and isolated runtimes; not committed |
| `dataset/` | Local CAD and dataset inputs; not committed |
| `simulation/` | Simulation configuration, tools, tests, and generated reports |
| `tools/` | Standalone workspace utilities |
| `build/`, `install/`, `log/` | Generated colcon output; do not edit or commit |

Package-specific README files and implementation documentation remain beside
their owning ROS packages under `src/`.

## Build

```bash
cd ~/track_robot_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Hardware setup and test procedures may have additional safety and calibration
requirements. Follow the relevant guide instead of starting drivers or motion
nodes ad hoc.
