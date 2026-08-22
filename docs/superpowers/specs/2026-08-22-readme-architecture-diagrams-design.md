# README Architecture Diagrams Design

**Date:** 2026-08-22
**Status:** Approved in conversation
**Audience:** GitHub visitors first, with enough technical fidelity for ROS developers

## Objective

Add a concise, academically styled visual architecture set to the English
repository README. The diagrams must explain the robot platform and its three
headline capabilities without presenting planned or model-only integration as
operational behavior.

## Deliverables

Create five diagrams:

1. **System overview** — shared physical platform, ROS 2 foundation, three
   independent capability pipelines, and the shared motion-safety boundary.
2. **Hardware and compute topology** — Bunker Pro 2, Jetson AGX Orin, ZED2i,
   RS-Helios-32, Phidget Spatial IMU, PiPER, and L515 status.
3. **Semantic-search pipeline** — text query, visual grounding, ZED registered
   depth, semantic memory, target selection, planning, supervised navigation,
   and the separate LiDAR obstacle path.
4. **Human-following pipeline** — gesture-authorized camera identity, generic
   LiDAR tracklets, selected-target fusion, decision, avoidance, session
   supervision, and final motion authorization.
5. **Point-LIO pipeline** — RoboSense input, IMU frame/time adaptation,
   Point-LIO outputs, and the relevant frame bridge.

## Visual System

- Generate repository-owned SVG assets from committed Graphviz DOT sources.
- Use a white canvas, restrained line weights, rounded rectangular nodes, and
  one consistent sans-serif font stack.
- Use the same semantic colors in every diagram:
  - blue: sensors and physical inputs;
  - violet: perception and learned models;
  - teal: localization, fusion, and memory;
  - amber: planning and task decisions;
  - red: safety and authorization;
  - graphite: compute and actuation;
  - gray dashed: optional, model-only, or intentionally independent links.
- Use solid arrows for runtime data flow and dashed arrows for control,
  authorization, model-only, or non-integrated relationships. Each diagram
  carries only the legend it needs.
- All diagram text and README copy remain English.

## Accuracy Boundaries

- Semantic object position in the active profile comes from timestamp-matched
  ZED registered depth. RS-Helios remains active for obstacle mapping,
  costmaps, and motion safety; it is not shown mutating semantic target
  position.
- Semantic search and human following share hardware and safety infrastructure
  but not target identity, feature mission state, or supervisors.
- `cmd_vel_gate` is shown as the sole final `/cmd_vel` publisher.
- Phase 4B Nav2 currently uses Bunker odometry in `odom`; Point-LIO is shown as
  an independent localization capability, not as the active Nav2 localization
  source.
- PiPER is operational in the combined URDF/JointState/TF model only. Arm
  control is not shown as part of the autonomy runtime.
- The arm-mounted L515 is labeled visual-model only; no operational L515
  driver or TF is claimed.

## Repository Layout

```text
docs/architecture/diagrams/
  README.md
  system-overview.dot
  hardware-topology.dot
  semantic-search-pipeline.dot
  human-following-pipeline.dot
  point-lio-pipeline.dot

docs/assets/readme/architecture/
  system-overview.svg
  hardware-topology.svg
  semantic-search-pipeline.svg
  human-following-pipeline.svg
  point-lio-pipeline.svg
```

The root README embeds the generated SVG files. DOT files are the editable
sources; generated SVG files are committed for GitHub rendering.

## README Placement

- Replace the current large Mermaid block in **System Architecture** with the
  system-overview SVG and a short implementation-boundary note.
- Add the semantic-search, human-following, and Point-LIO diagrams to their
  corresponding feature sections.
- Add the hardware topology immediately before the hardware/software stack
  table.
- Keep detailed ROS topics, services, actions, state machines, and the complete
  TF tree in linked technical documentation rather than overloading the root
  README.

## Validation

- Every DOT source renders successfully with Graphviz `dot`.
- Every SVG is valid XML, contains a `viewBox`, and has no external asset or
  font dependency.
- PNG previews of all five diagrams are visually inspected for clipping,
  illegible labels, crossed edges, misleading arrows, and inconsistent color
  use.
- All local image references in the README resolve.
- `git diff --check` passes.
- The final diff contains only the design/plan, diagram sources/assets, and the
  README integration required for this work.
