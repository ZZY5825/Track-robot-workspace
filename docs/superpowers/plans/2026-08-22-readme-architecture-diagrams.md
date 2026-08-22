# README Architecture Diagrams Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add five accurate, visually consistent SVG architecture diagrams to the English GitHub README.

**Architecture:** Commit Graphviz DOT as the editable source and Graphviz-generated SVG as the GitHub-facing asset. Keep the root README visitor-oriented while preserving current runtime boundaries: ZED depth owns semantic 3D position, the two task pipelines own separate state, Point-LIO is independent from Phase 4B Nav2, and all executable velocity passes through the safety supervisor and final gate.

**Tech Stack:** Graphviz DOT/SVG, Markdown/HTML image embedding, Python standard-library XML and path validation.

## Global Constraints

- All diagram and README text is English.
- Use one shared color and typography system across all five diagrams.
- `cmd_vel_gate` is the only final `/cmd_vel` publisher shown.
- PiPER is labeled URDF/JointState/TF only; L515 is labeled visual-model only.
- Point-LIO must not be shown as the active Phase 4B Nav2 localization source.
- DOT sources and generated SVG files are both committed.

---

### Task 1: Establish the diagram source contract

**Files:**
- Create: `docs/architecture/diagrams/README.md`

**Interfaces:**
- Consumes: Graphviz `dot` from `/usr/bin/dot`.
- Produces: documented render commands and the color/edge semantics used by every DOT file.

- [x] **Step 1: Document source ownership and exact render command**

Document that `*.dot` files are authoritative and render each file with:

```bash
dot -Tsvg docs/architecture/diagrams/<name>.dot \
  -o docs/assets/readme/architecture/<name>.svg
```

Define the shared palette and solid-versus-dashed edge meaning from the approved design.

- [x] **Step 2: Verify the documentation has no placeholders**

Run:

```bash
rg -n 'TBD|TODO|placeholder' docs/architecture/diagrams/README.md
```

Expected: no matches.

### Task 2: Create the system and hardware diagrams

**Files:**
- Create: `docs/architecture/diagrams/system-overview.dot`
- Create: `docs/architecture/diagrams/hardware-topology.dot`
- Create: `docs/assets/readme/architecture/system-overview.svg`
- Create: `docs/assets/readme/architecture/hardware-topology.svg`

**Interfaces:**
- Consumes: repository launch composition and combined Bunker/PiPER URDF facts.
- Produces: visitor-level platform overview and hardware-status diagram.

- [x] **Step 1: Write the system overview source**

Use five ranked layers: physical inputs, ROS 2 foundation, three independent capability lanes, task/navigation, and shared safety/actuation. Show semantic search and human following reaching safety; show Point-LIO outputs separately with an explicit `independent localization` note.

- [x] **Step 2: Write the hardware topology source**

Place Jetson AGX Orin centrally. Connect ZED2i, RS-Helios-32, Phidget Spatial IMU, Bunker interface, and combined robot description. Draw PiPER and L515 with dashed gray status links labeled `URDF / JointState / TF` and `visual model only`.

- [x] **Step 3: Render SVG and temporary PNG previews**

Run:

```bash
dot -Tsvg docs/architecture/diagrams/system-overview.dot -o docs/assets/readme/architecture/system-overview.svg
dot -Tsvg docs/architecture/diagrams/hardware-topology.dot -o docs/assets/readme/architecture/hardware-topology.svg
dot -Tpng -Gdpi=150 docs/architecture/diagrams/system-overview.dot -o /tmp/system-overview.png
dot -Tpng -Gdpi=150 docs/architecture/diagrams/hardware-topology.dot -o /tmp/hardware-topology.png
```

Expected: four commands exit 0; previews have no clipping or crossed primary-flow edges.

### Task 3: Create the three capability diagrams

**Files:**
- Create: `docs/architecture/diagrams/semantic-search-pipeline.dot`
- Create: `docs/architecture/diagrams/human-following-pipeline.dot`
- Create: `docs/architecture/diagrams/point-lio-pipeline.dot`
- Create: `docs/assets/readme/architecture/semantic-search-pipeline.svg`
- Create: `docs/assets/readme/architecture/human-following-pipeline.svg`
- Create: `docs/assets/readme/architecture/point-lio-pipeline.svg`

**Interfaces:**
- Consumes: current launch graphs, implemented topics, and documented ownership boundaries.
- Produces: one README-ready diagram per headline capability.

- [x] **Step 1: Write the semantic-search source**

Show query and ZED RGB entering YOLO-World/CLIP with optional DINOv3 appearance support; timestamp-matched ZED depth joins semantic observations before bounded memory, target selection, approach planning, and supervised Nav2. Draw the RS-Helios obstacle path separately into costmaps/safety.

- [x] **Step 2: Write the human-following source**

Show camera identity and LiDAR geometry as parallel inputs to selected-target association and the three-model IMM state. Continue through FollowDecision, controller, local trajectory avoidance, safety supervisor, gate, and Bunker. Draw session-supervisor authorization as dashed red control edges and RC/E-stop/base health as fail-closed inputs.

- [x] **Step 3: Write the Point-LIO source**

Show native RoboSense PointCloud2 and raw Phidget IMU, the IMU frame/time adapter, Point-LIO, its four public outputs, and the `camera_init → body → base_link → rslidar` frame bridge. Add a gray note that Phase 4B currently uses Bunker odometry.

- [x] **Step 4: Render SVG and temporary PNG previews**

Render all three sources with both `-Tsvg` to the repository asset paths and `-Tpng -Gdpi=150` to `/tmp`.

Expected: all commands exit 0; labels are legible and no capability is connected to an unimplemented runtime dependency.

### Task 4: Integrate the diagrams into the README

**Files:**
- Modify: `README.md`

**Interfaces:**
- Consumes: the five generated SVG paths.
- Produces: a visitor-oriented architecture narrative with accessible alt text.

- [x] **Step 1: Replace the existing Mermaid overview**

Embed `docs/assets/readme/architecture/system-overview.svg` under `System Architecture` with centered HTML and an alt string naming the three capabilities and shared safety boundary. Retain the note that semantic position uses ZED depth.

- [x] **Step 2: Add capability diagrams**

Embed the matching SVG beneath the `Find → Remember → Approach`, `Gesture → Lock → Follow`, and `Sense → Estimate → Map` headings. Keep existing prose and operator links.

- [x] **Step 3: Add the hardware topology**

Embed `hardware-topology.svg` immediately before the hardware/software stack table and add one sentence explaining the model-only PiPER/L515 status.

### Task 5: Validate the complete documentation change

**Files:**
- Test: `README.md`
- Test: `docs/architecture/diagrams/*.dot`
- Test: `docs/assets/readme/architecture/*.svg`

**Interfaces:**
- Consumes: the final documentation tree.
- Produces: evidence that all assets render and all README references resolve.

- [x] **Step 1: Re-render every SVG from source**

Run a shell loop over every DOT file and write the corresponding SVG. Expected: five successful renders.

- [x] **Step 2: Validate XML and embedded dependencies**

Use Python `xml.etree.ElementTree` to parse every SVG, assert each root is SVG, assert a `viewBox` exists, and reject `http://`, `https://`, and `file://` references in image/font/style URLs.

- [x] **Step 3: Validate README image references**

Extract local HTML `src` values from `README.md` and assert every referenced file exists. Expected: zero missing references.

- [x] **Step 4: Inspect scope and whitespace**

Run:

```bash
git diff --check
git status --short
git diff --stat origin/main...HEAD
```

Expected: no whitespace errors and only the approved design/plan, five DOT sources, five SVG assets, the diagram README, and root README are changed.

- [x] **Step 5: Commit the deliverable**

```bash
git add README.md docs/architecture/diagrams docs/assets/readme/architecture docs/superpowers/plans/2026-08-22-readme-architecture-diagrams.md
git commit -m "docs: add system architecture diagrams"
```
