# Workspace Organization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reorganize `track_robot_ws` so project documentation, raw rosbag recordings, and versioned semantic-search evidence have distinct, documented locations without changing ROS runtime behaviour.

**Architecture:** Keep every ROS package and third-party source tree in place. Move project-level documents into purpose-named `docs/` sections, move small versioned evidence into `artifacts/semantic_search/`, and move raw recordings into typed `rosbags/*/recordings/` directories. Update all active path consumers and validate each migration boundary before committing it.

**Tech Stack:** ROS 2 Foxy, colcon, Python 3/pytest, CMake/ament, rosbag2, Markdown, JSON/JSONL, Git.

## Global Constraints

- Do not rename ROS packages, nodes, topics, services, actions, or launch arguments.
- Do not move source files between ROS packages.
- Do not modify `src/third_party_ros/`, `src/track_robot_core/`, or other externally managed source trees.
- Do not move or commit model checkpoints, datasets, simulation outputs, or generated colcon output.
- Preserve every rosbag database and every valid manifest, calibration record, and evaluation report.
- Keep ROS Domain 20 as the managed semantic-search domain.
- Do not start ROS nodes, hardware drivers, motion controllers, or `/cmd_vel` publishers during this migration.
- Keep each migration category in its own commit so it can be reverted independently.
- Use `/home/track-robot/track_robot_ws/.local-git` as the current local Git directory because the managed workspace exposes an empty `.git` mount.

---

## Planned File Structure

### New navigation files

- `README.md`: workspace purpose, directory map, and primary operator links.
- `docs/README.md`: current guides, architecture, and development-history index.
- `docs/guides/README.md`: explains guide ownership and current-vs-historical status.
- `artifacts/semantic_search/README.md`: evidence types, version-control rules, and relationship to recordings.
- `rosbags/README.md`: raw data layout and Git policy.
- `config/README.md`: machine-local measured configuration policy.

### Moved project documentation

- `docs/guides/human-tracking/rosbag-replay.md`
- `docs/guides/semantic-search/rosbag-workflow.md`
- `docs/guides/semantic-search/phase2-recording-and-evaluation.md`
- `docs/architecture/semantic-search/*.md`
- `docs/development/plans/semantic-search/*.md`

### Moved evidence

- `artifacts/semantic_search/manifests/**`
- `artifacts/semantic_search/calibration/**`
- `artifacts/semantic_search/reports/**`
- `artifacts/semantic_search/annotations/` for future annotation JSONL files

### Moved raw recordings

- `rosbags/human_tracking/recordings/human_tracking_lidar_20260706_145752/`
- `rosbags/human_tracking/recordings/human_tracking_lidar_20260706_145900/`
- `rosbags/human_tracking/recordings/human_tracking_lidar_20260706_150711/`
- `rosbags/human_tracking/recordings/human_tracking_lidar_20260706_150918/`
- `rosbags/semantic_search/recordings/` for future Phase 0–2 recordings

### Preserved package-local documentation

- `src/track_robot_perception/README.md`
- `src/track_robot_perception/docs/**`
- `src/track_robot/track_robot_decision/docs/**`
- `src/track_robot/track_robot_safety/docs/**`
- `src/track_robot/track_robot_semantic_memory/README.md`
- `src/track_robot_semantic_search/README.md`

These remain with their packages because they describe or are installed with
one package.

---

### Task 1: Centralize project documentation and remove agent-only reports

**Files:**

- Create: `README.md`
- Create: `docs/README.md`
- Create: `docs/guides/README.md`
- Move: `rosbags/human_tracking_rosbag_test_guide.md` to `docs/guides/human-tracking/rosbag-replay.md`
- Move: `rosbags/semantic_search/semantic_search_rosbag_guide.md` to `docs/guides/semantic-search/rosbag-workflow.md`
- Move: `rosbags/semantic_search/phase2_recording_guide.md` to `docs/guides/semantic-search/phase2-recording-and-evaluation.md`
- Move: `docs/superpowers/specs/*.md` to `docs/architecture/semantic-search/`
- Move: `docs/superpowers/plans/*.md` to `docs/development/plans/semantic-search/`
- Modify: `src/track_robot/track_robot_bringup/test/test_quick_start_doc.py`
- Modify: all tracked Markdown files that reference the old documentation paths
- Delete: `.superpowers/sdd/*.md`

**Interfaces:**

- Consumes: the approved directory design in `docs/architecture/workspace/2026-07-23-workspace-organization-design.md`.
- Produces: stable human-facing documentation paths used by package tests and later artifact/recording path updates.

- [ ] **Step 1: Point the quick-start contract test at the approved guide path**

Change the `GUIDE` constant in
`src/track_robot/track_robot_bringup/test/test_quick_start_doc.py` to:

```python
GUIDE = (
    WORKSPACE_ROOT
    / 'docs'
    / 'guides'
    / 'semantic-search'
    / 'phase2-recording-and-evaluation.md'
)
```

- [ ] **Step 2: Run the contract test and verify the new path is not present yet**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/track_robot/track_robot_bringup/test/test_quick_start_doc.py
```

Expected: collection succeeds and tests fail with `FileNotFoundError` for
`docs/guides/semantic-search/phase2-recording-and-evaluation.md`.

- [ ] **Step 3: Create the destination directories and move the guides**

Run:

```bash
mkdir -p docs/guides/human-tracking
mkdir -p docs/guides/semantic-search
git --git-dir=.local-git --work-tree=. mv \
  rosbags/human_tracking_rosbag_test_guide.md \
  docs/guides/human-tracking/rosbag-replay.md
git --git-dir=.local-git --work-tree=. mv \
  rosbags/semantic_search/semantic_search_rosbag_guide.md \
  docs/guides/semantic-search/rosbag-workflow.md
git --git-dir=.local-git --work-tree=. mv \
  rosbags/semantic_search/phase2_recording_guide.md \
  docs/guides/semantic-search/phase2-recording-and-evaluation.md
```

- [ ] **Step 4: Move architecture documents and implementation plans**

Run:

```bash
mkdir -p docs/architecture/semantic-search
mkdir -p docs/development/plans/semantic-search
git --git-dir=.local-git --work-tree=. mv \
  docs/superpowers/specs/*.md \
  docs/architecture/semantic-search/
git --git-dir=.local-git --work-tree=. mv \
  docs/superpowers/plans/*.md \
  docs/development/plans/semantic-search/
```

Expected: `docs/superpowers/` no longer contains tracked files.

- [ ] **Step 5: Replace documentation-path references**

Apply these exact textual replacements to tracked text files:

```text
docs/superpowers/specs/ -> docs/architecture/semantic-search/
docs/superpowers/plans/ -> docs/development/plans/semantic-search/
rosbags/human_tracking_rosbag_test_guide.md -> docs/guides/human-tracking/rosbag-replay.md
rosbags/semantic_search/semantic_search_rosbag_guide.md -> docs/guides/semantic-search/rosbag-workflow.md
rosbags/semantic_search/phase2_recording_guide.md -> docs/guides/semantic-search/phase2-recording-and-evaluation.md
```

Do not rewrite the “Migration Map” table in the approved workspace design; it
intentionally records the old locations.

- [ ] **Step 6: Add the root and documentation indexes**

Create `README.md` with:

- a one-paragraph workspace description;
- a table for `src/`, `docs/`, `artifacts/`, `rosbags/`, `config/`, `models/`,
  `dataset/`, `simulation/`, and generated directories;
- links to the Phase 1/2 guide and the human-tracking rosbag replay guide;
- a safety note that semantic-search management uses ROS Domain 20 and does
  not imply permission to start motion nodes.

Create `docs/README.md` with separate “Current operator guides”,
“Architecture”, and “Development history” sections. Create
`docs/guides/README.md` stating that current executable procedures live here
while dated plans are historical records.

- [ ] **Step 7: Remove tracked agent-execution reports**

Run:

```bash
git --git-dir=.local-git --work-tree=. rm -r .superpowers/sdd
```

Expected: only agent-local review reports are removed; no file under
`rosbags/semantic_search/reports/` is removed in this task.

- [ ] **Step 8: Verify documentation paths and quick-start behaviour**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/track_robot/track_robot_bringup/test/test_quick_start_doc.py
rg -n 'docs/superpowers/(specs|plans)|rosbags/human_tracking_rosbag_test_guide.md|rosbags/semantic_search/(semantic_search_rosbag_guide|phase2_recording_guide).md' \
  --glob '!docs/architecture/workspace/2026-07-23-workspace-organization-design.md' \
  --glob '!docs/development/plans/workspace/2026-07-23-workspace-organization.md' \
  .
```

Expected: all quick-start tests pass and `rg` returns no results.

- [ ] **Step 9: Commit the documentation migration**

Run:

```bash
git --git-dir=.local-git --work-tree=. add README.md docs \
  src/track_robot/track_robot_bringup/test/test_quick_start_doc.py
git --git-dir=.local-git --work-tree=. commit \
  -m "docs: organize workspace documentation"
```

---

### Task 2: Separate semantic-search evidence from raw recordings

**Files:**

- Create: `artifacts/semantic_search/README.md`
- Create: `artifacts/semantic_search/annotations/.gitkeep`
- Move: `rosbags/semantic_search/manifests/**` to `artifacts/semantic_search/manifests/**`
- Move: `rosbags/semantic_search/calibration/**` to `artifacts/semantic_search/calibration/**`
- Move: `rosbags/semantic_search/reports/**` to `artifacts/semantic_search/reports/**`
- Modify: `src/track_robot/track_robot_semantic_memory/CMakeLists.txt`
- Modify: `src/track_robot/track_robot_bringup/test/test_quick_start_doc.py`
- Modify: `src/track_robot_semantic_search/README.md`
- Modify: moved guides, architecture documents, plans, manifests, and reports that contain evidence paths

**Interfaces:**

- Consumes: documentation paths established by Task 1.
- Produces: `artifacts/semantic_search/{manifests,annotations,calibration,reports}` as the only versioned semantic-search evidence root.

- [ ] **Step 1: Add failing evidence-path assertions to the guide contract**

Add this test to
`src/track_robot/track_robot_bringup/test/test_quick_start_doc.py`:

```python
def test_quick_start_keeps_evidence_outside_raw_rosbags():
    text = _guide()

    assert 'artifacts/semantic_search/manifests/' in text
    assert 'artifacts/semantic_search/annotations/' in text
    assert 'artifacts/semantic_search/reports/' in text
    assert 'rosbags/semantic_search/manifests/' not in text
    assert 'rosbags/semantic_search/annotations/' not in text
    assert 'rosbags/semantic_search/reports/' not in text
```

- [ ] **Step 2: Run the new test and verify it fails**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/track_robot/track_robot_bringup/test/test_quick_start_doc.py \
  -k evidence
```

Expected: FAIL because the guide still uses `rosbags/semantic_search/` for
versioned evidence.

- [ ] **Step 3: Move manifests, calibration records, and reports**

Run:

```bash
mkdir -p artifacts/semantic_search
git --git-dir=.local-git --work-tree=. mv \
  rosbags/semantic_search/manifests \
  artifacts/semantic_search/manifests
git --git-dir=.local-git --work-tree=. mv \
  rosbags/semantic_search/calibration \
  artifacts/semantic_search/calibration
git --git-dir=.local-git --work-tree=. mv \
  rosbags/semantic_search/reports \
  artifacts/semantic_search/reports
mkdir -p artifacts/semantic_search/annotations
```

Add an empty `.gitkeep` under `annotations/`.

- [ ] **Step 4: Update all evidence-path consumers**

Apply these exact replacements to tracked text files:

```text
rosbags/semantic_search/manifests/ -> artifacts/semantic_search/manifests/
rosbags/semantic_search/annotations/ -> artifacts/semantic_search/annotations/
rosbags/semantic_search/calibration/ -> artifacts/semantic_search/calibration/
rosbags/semantic_search/reports/ -> artifacts/semantic_search/reports/
```

Update the installed report path in
`src/track_robot/track_robot_semantic_memory/CMakeLists.txt` to:

```cmake
${CMAKE_CURRENT_SOURCE_DIR}/../../../artifacts/semantic_search/reports/phase2_association_calibration_2026-07-16.json
```

Update relative links in the moved artifact README files to point to
`docs/guides/semantic-search/phase2-recording-and-evaluation.md`.

- [ ] **Step 5: Add the evidence index**

Create `artifacts/semantic_search/README.md` explaining:

- manifests describe immutable recording inputs;
- annotations are reviewable JSONL labels;
- calibration contains measured or manually reviewed calibration evidence;
- reports contain generated but intentionally versioned benchmark/evaluation
  results;
- raw `.db3`, `.mcap`, and runtime logs must remain under `rosbags/` or outside
  Git.

- [ ] **Step 6: Validate JSON, JSONL, and manifests**

Run:

```bash
find artifacts/semantic_search -type f -name '*.json' -print0 \
  | xargs -0 -n1 python3 -m json.tool
python3 -c "import json, pathlib; p=pathlib.Path('artifacts/semantic_search/reports/phase1_observations_2026-07-15.jsonl'); [json.loads(line) for line in p.read_text().splitlines() if line.strip()]; print('jsonl valid')"
bash -lc 'source /opt/ros/foxy/setup.bash && source install/setup.bash && ros2 run track_robot_semantic_search semantic_search_manifest validate artifacts/semantic_search/manifests/legacy/human_tracking_lidar_20260706_150711.json'
```

Expected: every JSON command exits zero, `jsonl valid` is printed, and the
manifest validator reports success.

- [ ] **Step 7: Run evidence-path tests**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/track_robot/track_robot_bringup/test/test_quick_start_doc.py
bash -lc 'source /opt/ros/foxy/setup.bash && source /home/track-robot/track_robot_ws/install/setup.bash && cd /home/track-robot/track_robot_ws/src/track_robot_semantic_search && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test/test_manifest.py'
```

Expected: all tests pass.

- [ ] **Step 8: Commit the evidence migration**

Run:

```bash
git --git-dir=.local-git --work-tree=. add artifacts docs src rosbags
git --git-dir=.local-git --work-tree=. commit \
  -m "refactor: separate semantic search evidence"
```

---

### Task 3: Rehome raw rosbag recordings without changing their content

**Files:**

- Create: `rosbags/README.md`
- Move: four `rosbags/human_tracking_lidar_*` recording directories to `rosbags/human_tracking/recordings/`
- Create: `rosbags/semantic_search/recordings/.gitkeep`
- Modify: `artifacts/semantic_search/manifests/legacy/human_tracking_lidar_20260706_150711.json`
- Modify: current guides, package README files, architecture documents, and plans containing recording paths
- Modify: `src/track_robot/track_robot_bringup/test/test_quick_start_doc.py`

**Interfaces:**

- Consumes: artifact paths established by Task 2.
- Produces: a raw-data-only `rosbags/` tree with stable typed recording roots.

- [ ] **Step 1: Add failing recording-layout assertions**

Add this test to
`src/track_robot/track_robot_bringup/test/test_quick_start_doc.py`:

```python
def test_quick_start_uses_typed_recording_directory():
    text = _guide()

    assert 'rosbags/semantic_search/recordings/' in text
    assert 'rosbags/semantic_search/raw/' not in text
    assert 'rosbags/semantic_search/bags/' not in text
```

- [ ] **Step 2: Run the new test and verify it fails**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/track_robot/track_robot_bringup/test/test_quick_start_doc.py \
  -k recording
```

Expected: FAIL because the guide still uses the old `bags/` path.

- [ ] **Step 3: Capture the pre-move recording inventory**

Run:

```bash
find rosbags -maxdepth 2 -type f \
  \( -name '*.db3' -o -name '*.db3-shm' -o -name '*.db3-wal' -o -name 'metadata.yaml' \) \
  -printf '%f\t%s\n' | sort > /tmp/track_robot_rosbag_inventory.before
wc -l /tmp/track_robot_rosbag_inventory.before
```

Expected: 14 files are recorded in the inventory.

- [ ] **Step 4: Move each recording atomically on the same filesystem**

Run:

```bash
mkdir -p rosbags/human_tracking/recordings
mkdir -p rosbags/semantic_search/recordings
mv rosbags/human_tracking_lidar_20260706_145752 \
  rosbags/human_tracking/recordings/
mv rosbags/human_tracking_lidar_20260706_145900 \
  rosbags/human_tracking/recordings/
mv rosbags/human_tracking_lidar_20260706_150711 \
  rosbags/human_tracking/recordings/
mv rosbags/human_tracking_lidar_20260706_150918 \
  rosbags/human_tracking/recordings/
touch rosbags/semantic_search/recordings/.gitkeep
git --git-dir=.local-git --work-tree=. add -A rosbags
```

Do not copy and delete the 11 GiB recording payloads in separate operations.

- [ ] **Step 5: Update recording paths**

Apply these exact replacements to tracked text files:

```text
rosbags/human_tracking_lidar_ -> rosbags/human_tracking/recordings/human_tracking_lidar_
rosbags/semantic_search/raw/ -> rosbags/semantic_search/recordings/
rosbags/semantic_search/bags/ -> rosbags/semantic_search/recordings/
```

Set the legacy manifest field to:

```json
"relative_path": "rosbags/human_tracking/recordings/human_tracking_lidar_20260706_150711"
```

- [ ] **Step 6: Add the rosbag index**

Create `rosbags/README.md` documenting:

- `human_tracking/recordings/` and `semantic_search/recordings/`;
- that recording payloads are local and ignored by Git;
- that manifests and evaluation evidence live under
  `artifacts/semantic_search/`;
- that users must stop live camera/LiDAR drivers before replaying equivalent
  topics on the same ROS domain.

- [ ] **Step 7: Verify content inventory and rosbag readability**

Run:

```bash
find rosbags -maxdepth 4 -type f \
  \( -name '*.db3' -o -name '*.db3-shm' -o -name '*.db3-wal' -o -name 'metadata.yaml' \) \
  -printf '%f\t%s\n' | sort > /tmp/track_robot_rosbag_inventory.after
diff -u /tmp/track_robot_rosbag_inventory.before \
  /tmp/track_robot_rosbag_inventory.after
ros2 bag info rosbags/human_tracking/recordings/human_tracking_lidar_20260706_145752
ros2 bag info rosbags/human_tracking/recordings/human_tracking_lidar_20260706_145900
ros2 bag info rosbags/human_tracking/recordings/human_tracking_lidar_20260706_150711
ros2 bag info rosbags/human_tracking/recordings/human_tracking_lidar_20260706_150918
```

Expected: inventory diff is empty and all four `ros2 bag info` commands print
valid sqlite3 metadata.

- [ ] **Step 8: Revalidate the moved legacy manifest and guide test**

Run:

```bash
bash -lc 'source /opt/ros/foxy/setup.bash && source install/setup.bash && ros2 run track_robot_semantic_search semantic_search_manifest validate artifacts/semantic_search/manifests/legacy/human_tracking_lidar_20260706_150711.json'
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/track_robot/track_robot_bringup/test/test_quick_start_doc.py
```

Expected: manifest validation succeeds and all quick-start tests pass.

- [ ] **Step 9: Commit the recording-path migration**

Run:

```bash
git --git-dir=.local-git --work-tree=. add -A \
  rosbags artifacts docs src
git --git-dir=.local-git --work-tree=. commit \
  -m "refactor: organize rosbag recordings"
```

---

### Task 4: Establish the machine-local configuration area

**Files:**

- Create: `config/README.md`
- Modify: `.gitignore`
- Move locally without tracking: `semantic_search_perception.yaml` to `config/local/semantic_search_perception.yaml`
- Modify: `README.md`

**Interfaces:**

- Consumes: the root navigation established by Task 1.
- Produces: one documented location for measured machine-local configuration without committing device-specific values.

- [ ] **Step 1: Add ignore rules for local configuration**

Add:

```gitignore
# Machine-local measured configuration
config/local/
config/*.measured.yaml
```

Keep package-owned example and default configurations tracked under
`src/<package>/config/`.

- [ ] **Step 2: Create the configuration policy**

Create `config/README.md` explaining:

- measured camera/LiDAR extrinsics belong in
  `config/camera_extrinsic.measured.yaml`;
- generated launch parameter snapshots belong in `config/local/`;
- examples and defaults remain in their owning ROS package;
- machine-specific measured files are ignored until deliberately reviewed.

- [ ] **Step 3: Move the existing ignored runtime snapshot**

Run:

```bash
mkdir -p config/local
mv semantic_search_perception.yaml \
  config/local/semantic_search_perception.yaml
```

Expected: the file content is unchanged and Git does not list it as a new
tracked file.

- [ ] **Step 4: Verify root cleanliness and ignore behaviour**

Run:

```bash
test -f config/local/semantic_search_perception.yaml
test ! -e semantic_search_perception.yaml
git --git-dir=.local-git --work-tree=. check-ignore \
  config/local/semantic_search_perception.yaml
git --git-dir=.local-git --work-tree=. status --short
```

Expected: `check-ignore` prints the local snapshot path; Git lists only
`.gitignore`, `config/README.md`, and intended root README changes for this
task.

- [ ] **Step 5: Commit the configuration policy**

Run:

```bash
git --git-dir=.local-git --work-tree=. add .gitignore README.md config/README.md
git --git-dir=.local-git --work-tree=. commit \
  -m "docs: define machine local configuration"
```

---

### Task 5: Run the full organization and regression gate

**Files:**

- Verify: all files changed by Tasks 1–4
- Modify only if a validation failure identifies a stale path or broken link

**Interfaces:**

- Consumes: the organized workspace from Tasks 1–4.
- Produces: evidence that the migration changed filesystem organization but not ROS runtime behaviour.

- [ ] **Step 1: Verify deprecated physical paths are absent**

Run:

```bash
test ! -e docs/superpowers
test ! -e rosbags/human_tracking_rosbag_test_guide.md
test ! -e rosbags/semantic_search/manifests
test ! -e rosbags/semantic_search/calibration
test ! -e rosbags/semantic_search/reports
test -z "$(git --git-dir=.local-git --work-tree=. \
  ls-files '.superpowers/**')"
```

Expected: every command exits zero.

- [ ] **Step 2: Scan for stale active references**

Run:

```bash
rg -n 'docs/superpowers/(specs|plans)|rosbags/human_tracking_rosbag_test_guide.md|rosbags/semantic_search/(semantic_search_rosbag_guide|phase2_recording_guide|manifests|annotations|calibration|reports|raw|bags)/' \
  --glob '!docs/architecture/workspace/2026-07-23-workspace-organization-design.md' \
  --glob '!docs/development/plans/workspace/2026-07-23-workspace-organization.md' \
  --glob '!src/track_robot/track_robot_bringup/test/test_quick_start_doc.py' \
  .
```

Expected: no results.

- [ ] **Step 3: Validate all evidence serialization**

Run:

```bash
find artifacts/semantic_search -type f -name '*.json' -print0 \
  | xargs -0 -n1 python3 -m json.tool
python3 -c "import json, pathlib; files=pathlib.Path('artifacts/semantic_search').rglob('*.jsonl'); [json.loads(line) for p in files for line in p.read_text().splitlines() if line.strip()]; print('all semantic-search JSONL valid')"
```

Expected: all commands exit zero.

- [ ] **Step 4: Run the affected Python test suites**

Run:

```bash
cd src/track_robot/track_robot_bringup
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test
cd ../track_robot_sensor_bringup
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test
cd ../../track_robot_semantic_search
bash -lc 'source /opt/ros/foxy/setup.bash && source /home/track-robot/track_robot_ws/install/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test'
cd ../../..
```

Expected: all tests pass.

- [ ] **Step 5: Build the affected ROS packages**

Run:

```bash
bash -lc 'source /opt/ros/foxy/setup.bash && colcon build --symlink-install --packages-select track_robot_bringup track_robot_sensor_bringup track_robot_semantic_search track_robot_semantic_memory'
```

Expected: all four packages finish successfully.

- [ ] **Step 6: Run colcon tests and inspect results**

Run:

```bash
bash -lc 'source /opt/ros/foxy/setup.bash && source install/setup.bash && colcon test --packages-select track_robot_bringup track_robot_sensor_bringup track_robot_semantic_search track_robot_semantic_memory --return-code-on-test-failure'
colcon test-result --verbose
```

Expected: no test failures.

- [ ] **Step 7: Verify no ROS or hardware process was started**

Run:

```bash
ps -eo pid,ppid,stat,cmd | rg \
  'ros2 (launch|run|bag play)|rslidar_sdk_node|zed_(wrapper|camera)|bunker_base_node' \
  || true
```

Expected: no migration-owned process appears.

- [ ] **Step 8: Review the final Git change set**

Run:

```bash
git --git-dir=.local-git --work-tree=. status --short --branch
git --git-dir=.local-git --work-tree=. diff --check main...HEAD
git --git-dir=.local-git --work-tree=. diff --stat main...HEAD
git --git-dir=.local-git --work-tree=. log --oneline main..HEAD
```

Expected: the worktree is clean, `diff --check` is silent, moves are visible
as reorganizations, and functional source changes are limited to path
consumers and tests.
