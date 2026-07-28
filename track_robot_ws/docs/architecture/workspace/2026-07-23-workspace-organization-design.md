# Track Robot Workspace Organization Design

**Date:** 2026-07-23
**Status:** Approved for implementation

## Problem

The workspace currently mixes several different kinds of content:

- user guides are stored beside rosbag recordings;
- versioned manifests, calibration outputs, and evaluation reports are stored
  under `rosbags/` even though they are not recordings;
- project architecture and implementation history are grouped under the
  tool-specific name `docs/superpowers/`;
- package-local documentation and project-level documentation do not have a
  clear distinction;
- the repository has no root navigation document explaining where a user
  should start.

This makes the workspace difficult to navigate and makes ordinary Git diffs
look less intentional than they are.

## Goals

1. Give every top-level directory one clear responsibility.
2. Separate user documentation, raw recordings, and versioned evidence.
3. Replace tool-specific documentation paths with names that describe their
   engineering purpose.
4. Preserve ROS package boundaries and existing runtime behaviour.
5. Preserve all raw recordings and all valid test evidence.
6. Provide clear entry points for operators, developers, and reviewers.

## Non-goals

- Renaming ROS packages, nodes, topics, services, actions, or launch arguments.
- Moving source files between ROS packages.
- Reorganizing vendored or third-party source trees.
- Moving model checkpoints, datasets, or simulation assets.
- Changing semantic-search algorithms or hardware behaviour.
- Starting ROS nodes or hardware services as part of the migration.

## Target Layout

```text
track_robot_ws/
├── README.md
├── docs/
│   ├── README.md
│   ├── guides/
│   │   ├── human-tracking/
│   │   │   └── rosbag-replay.md
│   │   └── semantic-search/
│   │       ├── rosbag-workflow.md
│   │       └── phase2-recording-and-evaluation.md
│   ├── architecture/
│   │   ├── workspace/
│   │   └── semantic-search/
│   └── development/
│       └── plans/
│           └── semantic-search/
├── artifacts/
│   └── semantic_search/
│       ├── README.md
│       ├── manifests/
│       ├── annotations/
│       ├── calibration/
│       └── reports/
├── rosbags/
│   ├── README.md
│   ├── human_tracking/
│   │   └── recordings/
│   └── semantic_search/
│       └── recordings/
├── dataset/
├── models/
├── config/
├── simulation/
├── tools/
└── src/
```

Generated `build/`, `install/`, `log/`, cache, model, and raw rosbag payloads
remain ignored by Git.

## Content Classification

### Project documentation

`docs/` is the authoritative location for documentation that applies to the
workspace as a whole:

- `docs/guides/` contains current operator procedures;
- `docs/architecture/` contains architecture and approved designs;
- `docs/development/plans/` contains dated implementation plans and historical
  engineering checkpoints.

The old `docs/superpowers/` name is removed because it exposes the authoring
tool rather than the purpose of the documents.

### Package-local documentation

A document remains in `src/<package>/` when at least one of these is true:

- it documents only that package's API or implementation;
- it is installed by the package;
- package tests intentionally verify its presence or content;
- it must evolve atomically with the package implementation.

Package `README.md` files remain with their packages. Project-level guides may
link to them, but do not duplicate their detailed content.

### Recordings

`rosbags/` contains only rosbag recordings and a short index explaining storage
rules. Existing human-tracking recordings move under
`rosbags/human_tracking/recordings/`. Future semantic-search recordings use
`rosbags/semantic_search/recordings/`.

Rosbag payloads remain local and ignored. Versioned `metadata.yaml` files move
with their recording directories.

### Versioned evidence

`artifacts/semantic_search/` contains small, reviewable files that describe or
evaluate recordings:

- manifests and annotations;
- calibration inputs and review records;
- benchmark, replay, and evaluation reports.

These files remain tracked by Git. Code, CMake rules, tests, and documentation
that use them must point to the new paths.

### Machine-local configuration

`config/` is the documented location for measured, machine-specific
configuration such as the Phase 2 camera-to-LiDAR extrinsic. The directory
contains a tracked README, while measured calibration files remain ignored
unless they are deliberately promoted into a reviewed package configuration.

### Agent-local files

Internal agent execution reports under `.superpowers/` are not project
documentation. Tracked copies are removed, and the directory remains ignored.
No engineering evidence under `artifacts/` is removed.

## Migration Map

| Current location | New location |
| --- | --- |
| `rosbags/human_tracking_rosbag_test_guide.md` | `docs/guides/human-tracking/rosbag-replay.md` |
| `rosbags/semantic_search/semantic_search_rosbag_guide.md` | `docs/guides/semantic-search/rosbag-workflow.md` |
| `rosbags/semantic_search/phase2_recording_guide.md` | `docs/guides/semantic-search/phase2-recording-and-evaluation.md` |
| `rosbags/semantic_search/manifests/` | `artifacts/semantic_search/manifests/` |
| `rosbags/semantic_search/calibration/` | `artifacts/semantic_search/calibration/` |
| `rosbags/semantic_search/reports/` | `artifacts/semantic_search/reports/` |
| `docs/superpowers/specs/` | `docs/architecture/semantic-search/` |
| `docs/superpowers/plans/` | `docs/development/plans/semantic-search/` |
| `rosbags/human_tracking_lidar_*` | `rosbags/human_tracking/recordings/human_tracking_lidar_*` |

## Navigation

The root `README.md` explains:

- what the workspace contains;
- which directories are source, documentation, evidence, recordings, models,
  datasets, and generated output;
- where Phase 1 and Phase 2 semantic-search instructions live;
- that ROS Domain 20 is the managed semantic-search domain.

`docs/README.md` indexes current guides separately from architecture and
historical plans. `rosbags/README.md` explains that raw recordings are local
data. `artifacts/semantic_search/README.md` defines the evidence contract.

## Reference and Compatibility Policy

The migration updates:

- active shell commands and workspace-relative paths;
- Markdown links;
- JSON paths used as evidence references;
- CMake test-data paths;
- Python test constants and assertions;
- package README links.

Historical plan filenames keep their dates and descriptive names. References
inside them are updated when they are intended to locate a current file.

There will be no compatibility symlinks for old documentation or evidence
paths. Keeping duplicate paths would recreate the ambiguity this migration is
intended to remove. Runtime ROS interfaces remain unchanged.

## Safety

- Raw recording directories are moved, never copied and deleted in separate
  steps.
- The recording directory names and contents remain unchanged.
- Before and after inventory counts must match.
- No rosbag database, model checkpoint, dataset, or calibration record is
  deleted.
- No ROS node, service, driver, or hardware process is started.
- Generated or local-only files are not added to Git.

## Validation

The migration is complete only when:

1. every documented destination has an index and a clear purpose;
2. tracked old paths are absent;
3. no active reference points to an old path;
4. JSON and JSONL evidence files remain parseable;
5. semantic-search manifest validation passes;
6. package tests that depend on documentation or evidence paths pass;
7. the workspace builds with `colcon build`;
8. `colcon test` reports no new failures;
9. Git shows moves where content is unchanged rather than unexplained
   delete-and-recreate churn;
10. no ROS or hardware process is left running.

## Rollback

Tracked files can be restored from the migration commit. Raw recording moves
are reversible by moving each unchanged recording directory back to its
original location. The migration must be committed separately from functional
changes so it can be reverted without affecting semantic-search behaviour.
