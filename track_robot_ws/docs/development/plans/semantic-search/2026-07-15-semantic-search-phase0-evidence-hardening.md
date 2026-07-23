# Semantic Search Phase 0 Evidence Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close the four confirmed Phase 0 evidence gaps, regenerate trustworthy three-rate replay evidence, freeze Phase 0, and hand off to Phase 1.

**Architecture:** Keep the passive semantic-search package and existing human-tracking runtime isolated. Add a strict closed-rosbag reader, explicit online-versus-replay freshness, evaluation report contract 1.1.0 with measurable source-window gates, and a comparator that recomputes acceptance from one validated manifest plus exactly three formal reports.

**Tech Stack:** ROS 2 Foxy, Python 3.8, Python standard-library `sqlite3`, rclpy, rosbag2 sqlite3, PyYAML 5.3, psutil, pytest 4.6, Jetson tegrastats, local-only Git.

## Global Constraints

- Work only in `/home/track-robot/track_robot_ws/.worktrees/semantic-search-phase0` on `feature/semantic-search-phase0`.
- Use local Git only. Do not add a remote, push, upload, or modify the user's GitHub Workspace release pages.
- Do not add Python, ROS, CUDA, TensorRT, PyTorch, or system dependencies.
- Preserve ROS 2 Foxy and Python 3.8 compatibility; do not use newer Python syntax.
- Do not add Phase 1 model, inference, language, memory, or motion code.
- The semantic package must not publish `Twist`, `/cmd_vel`, motion permission, or motion intent.
- Never modify these protected files:
  - `src/track_robot_bringup/launch/human_tracking_simplified.launch.py`
  - `src/track_robot_bringup/launch/human_tracking_rosbag_replay.launch.py`
  - `src/track_robot_perception/config/human_tracking.yaml`
  - `src/track_robot_decision/config/outdoor_decision.yaml`
  - `src/track_robot_decision/config/motion_safety_supervisor.yaml`
- Follow red-green-refactor for every production behavior. Capture the focused RED output before editing production code.
- Use fresh serial replay processes. Signal only processes started for the current run; never use `pkill` or `killall`.
- The accepted report policy is exactly `foxy_wall_time_scaled` with arrival-monotonic freshness, a 45.0-second source target, 0.90 minimum source coverage, and rate/wall-duration pairs 0.5/90.0, 1.0/45.0, and 2.0/22.5.
- Replace the tracked 2026-07-14 report with `artifacts/semantic_search/reports/phase0_baseline_2026-07-15.json`; Git history retains the old report.
- The design authority is `docs/architecture/semantic-search/2026-07-15-semantic-search-phase0-evidence-hardening-design.md`.

## File Responsibility Map

- `track_robot_semantic_search/manifest.py`: manifest validation, closed-bag inventory, stable checksum, and atomic JSON helpers.
- `track_robot_semantic_search/localization_health.py`: pure localization state and epoch decisions.
- `track_robot_semantic_search/localization_health_node.py`: ROS input capture, freshness time-base selection, TF checks, and diagnostic publication.
- `track_robot_semantic_search/evaluation.py`: deterministic topic/resource metrics and hard-gate computation.
- `track_robot_semantic_search/evaluator_node.py`: bounded ROS replay window, readiness filtering, final pairing, and atomic report output.
- `track_robot_semantic_search/compare_reports.py`: strict report parsing, gate recomputation, provenance comparison, and formal rate-set acceptance.
- `schemas/evaluation_report.schema.json`: machine-readable report 1.1.0 contract.
- `launch/semantic_search_phase0.launch.py`: passive two-node launch and explicit freshness/timing-policy propagation.
- `config/semantic_search_phase0.yaml`: static topic and threshold configuration only; launch-owned dynamic values remain absent.
- `test/test_manifest.py`, `test/test_localization_health.py`, `test/test_evaluation.py`, and `test/test_launch_contract.py`: regression contracts.
- semantic-search rosbag/operator documents: formal commands, policy boundaries, and capability disclaimers.

---

### Task 1: Fail-Closed Closed-Rosbag Integrity

**Files:**
- Modify: `src/track_robot_semantic_search/track_robot_semantic_search/manifest.py`
- Modify: `src/track_robot_semantic_search/test/test_manifest.py`

**Interfaces:**
- Produces: `read_closed_rosbag(bag_dir: Path) -> Tuple[Mapping[str, Any], str]`, returning the checked `rosbag2_bagfile_information` mapping and deterministic closed-bag SHA-256.
- Preserves: `build_legacy_manifest(...)` and `build_field_manifest(...)` public signatures.
- Consumes later: Task 5 uses `build_legacy_manifest(...)` against the original non-symlinked legacy bag.

- [ ] **Step 1: Add valid sqlite bag fixture helpers**

Add standard-library sqlite setup and metadata that includes the real Foxy key:

```python
import sqlite3


def write_sqlite_storage(path):
    connection = sqlite3.connect(str(path))
    try:
        connection.execute(
            'CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT, type TEXT)')
        connection.execute(
            'CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER, '
            'timestamp INTEGER, data BLOB)')
        connection.commit()
    finally:
        connection.close()


def write_closed_bag(root, name='bag'):
    bag = root / name
    bag.mkdir(parents=True)
    storage = bag / '{}_0.db3'.format(name)
    write_sqlite_storage(storage)
    metadata = {
        'rosbag2_bagfile_information': {
            'storage_identifier': 'sqlite3',
            'relative_file_paths': [storage.name],
            'starting_time': {'nanoseconds_since_epoch': 12},
            'duration': {'nanoseconds': 34},
            'topics_with_message_count': [],
        },
    }
    (bag / 'metadata.yaml').write_text(
        yaml.safe_dump(metadata), encoding='utf-8')
    return bag, storage
```

Update existing builder fixtures to use valid sqlite storage and
`relative_file_paths`.

- [ ] **Step 2: Write focused failing closed-bag tests**

Add one test per failure family:

```python
@pytest.mark.parametrize('suffix', ['-wal', '-shm'])
def test_manifest_builder_rejects_active_sqlite_sidecars(tmp_path, suffix):
    bag, storage = write_closed_bag(tmp_path)
    Path(str(storage) + suffix).write_bytes(b'active')
    with pytest.raises(ManifestError, match='closed rosbag'):
        build_legacy_manifest(bag, 'bag', tmp_path)


def test_manifest_builder_rejects_unlisted_storage(tmp_path):
    bag, _ = write_closed_bag(tmp_path)
    write_sqlite_storage(bag / 'unlisted.db3')
    with pytest.raises(ManifestError, match='unlisted'):
        build_legacy_manifest(bag, 'bag', tmp_path)


def test_manifest_builder_rejects_symlinked_storage(tmp_path):
    bag, storage = write_closed_bag(tmp_path)
    target = tmp_path / 'external.db3'
    storage.rename(target)
    storage.symlink_to(target)
    with pytest.raises(ManifestError, match='symlink'):
        build_legacy_manifest(bag, 'bag', tmp_path)


def test_manifest_builder_rejects_invalid_sqlite(tmp_path):
    bag, storage = write_closed_bag(tmp_path)
    storage.write_bytes(b'not sqlite')
    with pytest.raises(ManifestError, match='SQLite'):
        build_legacy_manifest(bag, 'bag', tmp_path)


def test_manifest_builder_rejects_file_changed_during_hash(
        tmp_path, monkeypatch):
    bag, storage = write_closed_bag(tmp_path)
    original = manifest_module.sha256_tree

    def hash_then_change(path):
        digest = original(path)
        with storage.open('ab') as stream:
            stream.write(b'changed')
        return digest

    monkeypatch.setattr(manifest_module, 'sha256_tree', hash_then_change)
    with pytest.raises(ManifestError, match='changed while hashing'):
        build_legacy_manifest(bag, 'bag', tmp_path)
```

Add malformed YAML, missing mapping/key, unsafe `relative_file_paths`, and
unsupported storage identifier cases, each expecting `ManifestError` rather
than raw `KeyError`, `TypeError`, or `yaml.YAMLError`.

- [ ] **Step 3: Run the RED tests**

Run:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot_semantic_search/test/test_manifest.py -q
```

Expected: the new tests fail because active sidecars, symlinks, invalid sqlite,
metadata errors, and in-flight changes are not yet rejected.

- [ ] **Step 4: Implement the checked inventory and stable hash**

Implement these focused helpers in `manifest.py`:

```python
import sqlite3
from typing import List, Tuple


def _file_identity(path: Path) -> Tuple[int, int, int, int]:
    metadata = path.stat()
    return (
        int(metadata.st_dev), int(metadata.st_ino), int(metadata.st_size),
        int(metadata.st_mtime_ns))


def _check_sqlite(path: Path) -> None:
    uri = 'file:{}?mode=ro&immutable=1'.format(path.resolve().as_posix())
    try:
        connection = sqlite3.connect(uri, uri=True)
        try:
            result = connection.execute('PRAGMA quick_check(1)').fetchone()
        finally:
            connection.close()
    except sqlite3.Error as error:
        raise ManifestError('invalid SQLite storage: {}'.format(path.name)) \
            from error
    if result != ('ok',):
        raise ManifestError('SQLite quick_check failed: {}'.format(path.name))


def read_closed_rosbag(bag_dir: Path):
    bag_dir = Path(bag_dir)
    metadata_path = bag_dir / 'metadata.yaml'
    if bag_dir.is_symlink() or metadata_path.is_symlink():
        raise ManifestError('closed rosbag cannot contain symlinks')
    if not bag_dir.is_dir() or not metadata_path.is_file():
        raise ManifestError('closed rosbag requires metadata.yaml')
    metadata_identity = _file_identity(metadata_path)
    try:
        with metadata_path.open('r', encoding='utf-8') as stream:
            document = _require_mapping(
                yaml.safe_load(stream), 'rosbag metadata document')
        metadata = _require_mapping(
            document['rosbag2_bagfile_information'],
            'rosbag2_bagfile_information')
        if metadata['storage_identifier'] != 'sqlite3':
            raise ManifestError('closed rosbag storage must be sqlite3')
        relative_paths = metadata['relative_file_paths']
        if not isinstance(relative_paths, list) or not relative_paths:
            raise ManifestError(
                'relative_file_paths must be a non-empty array')
    except (OSError, UnicodeError, yaml.YAMLError, KeyError, TypeError) as error:
        raise ManifestError('invalid rosbag metadata: {}'.format(error)) \
            from error
    if _file_identity(metadata_path) != metadata_identity:
        raise ManifestError('closed rosbag changed while reading metadata')

    storage_paths = []
    for index, value in enumerate(relative_paths):
        relative = _relative_path(
            value, 'relative_file_paths[{}]'.format(index))
        if not relative.endswith('.db3'):
            raise ManifestError('storage path must end with .db3')
        storage = bag_dir / Path(relative)
        if storage.is_symlink() or not storage.is_file():
            raise ManifestError(
                'closed rosbag storage must be a regular non-symlink file')
        storage_paths.append(storage)

    for item in bag_dir.rglob('*'):
        if item.is_symlink():
            raise ManifestError('closed rosbag cannot contain symlinks')
    expected = {metadata_path.resolve()}
    expected.update(path.resolve() for path in storage_paths)
    actual = {item.resolve() for item in bag_dir.rglob('*') if item.is_file()}
    unexpected = sorted(path.name for path in actual - expected)
    if unexpected:
        raise ManifestError(
            'closed rosbag contains unlisted files: {}'.format(
                ', '.join(unexpected)))

    inventory = [metadata_path] + storage_paths
    before = {str(path): _file_identity(path) for path in inventory}
    for storage in storage_paths:
        _check_sqlite(storage)
    digest = sha256_tree(bag_dir)
    after = {str(path): _file_identity(path) for path in inventory}
    if before != after:
        raise ManifestError('closed rosbag changed while hashing')
    return metadata, digest
```

Change `sha256_tree()` so sidecars affect its digest if it is called directly.
Manifest builders call `read_closed_rosbag()` and use its returned metadata and
digest. Catch `ManifestError` separately so contract failures are preserved;
wrap remaining YAML/filesystem shape failures as
`ManifestError('invalid rosbag metadata: ...')`.

- [ ] **Step 5: Run focused and package tests**

Run the focused manifest test, then:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot_semantic_search/test -q
```

Expected: all manifest tests pass; the package suite has zero failures.

- [ ] **Step 6: Commit Task 1**

```bash
git add \
  src/track_robot_semantic_search/track_robot_semantic_search/manifest.py \
  src/track_robot_semantic_search/test/test_manifest.py
git diff --cached --check
git commit -m "fix: require immutable closed rosbags"
```

---

### Task 2: Separate Online and Historical Replay Freshness

**Files:**
- Modify: `src/track_robot_semantic_search/track_robot_semantic_search/localization_health.py`
- Modify: `src/track_robot_semantic_search/track_robot_semantic_search/localization_health_node.py`
- Modify: `src/track_robot_semantic_search/launch/semantic_search_phase0.launch.py`
- Modify: `src/track_robot_semantic_search/test/test_localization_health.py`
- Modify: `src/track_robot_semantic_search/test/test_launch_contract.py`

**Interfaces:**
- Produces: `message_is_fresh(message, now_ns, timeout_sec, time_base='source_clock', arrival_ns=None) -> bool`.
- Produces: launch argument and node parameter `freshness_time_base`, accepted values `source_clock` and `arrival_monotonic`.
- Produces: launch argument `timing_policy`, accepted formal value `foxy_wall_time_scaled` and default `online_source_time`.
- Extends: `LocalizationSample` with `source_timestamp_rollback: bool`.
- Consumes later: evaluator report records the same launch-owned freshness and timing values.

- [ ] **Step 1: Write failing pure freshness and rollback tests**

```python
def test_arrival_freshness_accepts_historical_source_stamp():
    message = odometry(100)
    assert message_is_fresh(
        message,
        now_ns=10_200_000_000,
        timeout_sec=0.25,
        time_base='arrival_monotonic',
        arrival_ns=10_000_000_000,
    ) is True


def test_arrival_freshness_rejects_missing_future_or_stale_arrival():
    message = odometry(100)
    assert message_is_fresh(
        message, 10, 0.25, 'arrival_monotonic', None) is False
    assert message_is_fresh(
        message, 9, 0.25, 'arrival_monotonic', 10) is False
    assert message_is_fresh(
        message, 300_000_001, 0.25,
        'arrival_monotonic', 1) is False


def test_explicit_source_rollback_starts_new_epoch_in_arrival_mode():
    health = evaluator()
    assert health.update(sample(
        100, source_timestamp_rollback=False)).mode == (
            MemoryMode.LOCAL_SESSION)
    rolled = health.update(sample(
        200, source_timestamp_rollback=True))
    assert rolled.reason == 'timestamp_rollback'
    assert rolled.epoch_changed is True
    assert rolled.epoch_id == 2
```

Add validation tests for an unknown freshness mode and a node-harness test that
stores monotonic arrival time in each IMU/local/world callback.

- [ ] **Step 2: Write failing launch contract tests**

Update the exact launch-argument set to include `freshness_time_base` and
`timing_policy`. Assert the localization node receives
`freshness_time_base`, the evaluator receives both values, defaults remain
`source_clock` and `online_source_time`, and neither dynamic key appears in the
node-specific YAML.

- [ ] **Step 3: Run the RED tests**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot_semantic_search/test/test_localization_health.py \
  src/track_robot_semantic_search/test/test_launch_contract.py -q
```

Expected: failures show the old function lacks the time-base/arrival API, the
sample lacks rollback evidence, and launch lacks both arguments.

- [ ] **Step 4: Implement explicit freshness and rollback capture**

Implement strict time-base selection:

```python
def message_is_fresh(
        message, now_ns, timeout_sec, time_base='source_clock',
        arrival_ns=None):
    if message is None:
        return False
    if time_base == 'source_clock':
        reference_ns = stamp_ns(message.header.stamp)
    elif time_base == 'arrival_monotonic':
        if arrival_ns is None:
            return False
        reference_ns = int(arrival_ns)
    else:
        raise ValueError('unsupported freshness_time_base: {}'.format(
            time_base))
    age_ns = int(now_ns) - reference_ns
    return 0 <= age_ns <= int(float(timeout_sec) * 1000000000)
```

Declare and validate `freshness_time_base` in the node. Each sensor callback
stores `(message, time.monotonic_ns())`, compares the new source stamp to that
topic's prior stamp, and sets a one-shot rollback flag. `_publish()` uses ROS
now for `source_clock`, monotonic now for `arrival_monotonic`, passes the
rollback flag into `LocalizationSample`, then clears the flag only after the
evaluator consumes it. World-pose timestamp rollback remains checked by the
existing world-specific logic.

Add both launch arguments. Pass `freshness_time_base` to localization and both
values to evaluator. Keep them out of `semantic_search_phase0.yaml` so Foxy
node-specific YAML cannot override launch values.

- [ ] **Step 5: Run focused and package tests**

Run the two focused files and the complete semantic package suite. Expected:
all tests pass and online `source_clock` behavior remains unchanged.

- [ ] **Step 6: Commit Task 2**

```bash
git add \
  src/track_robot_semantic_search/track_robot_semantic_search/localization_health.py \
  src/track_robot_semantic_search/track_robot_semantic_search/localization_health_node.py \
  src/track_robot_semantic_search/launch/semantic_search_phase0.launch.py \
  src/track_robot_semantic_search/test/test_localization_health.py \
  src/track_robot_semantic_search/test/test_launch_contract.py
git diff --cached --check
git commit -m "fix: separate replay and online freshness"
```

---

### Task 3: Report 1.1.0, Window Completeness, and Recomputed Gates

**Files:**
- Modify: `src/track_robot_semantic_search/track_robot_semantic_search/evaluation.py`
- Modify: `src/track_robot_semantic_search/track_robot_semantic_search/evaluator_node.py`
- Modify: `src/track_robot_semantic_search/schemas/evaluation_report.schema.json`
- Modify: `src/track_robot_semantic_search/test/test_evaluation.py`

**Interfaces:**
- `EvaluationAccumulator(...)` additionally requires `wall_duration_sec`, `timing_policy`, and `freshness_time_base`.
- `TopicSeries.report()` adds `source_start_ns`, `source_end_ns`, `source_span_sec`, and `receive_span_sec`.
- Report top level adds exact `manifest_capabilities` mapping.
- Report `run` contains exactly `run_id`, `phase`, `replay_rate`, `timing_policy`, `wall_duration_sec`, `target_source_duration_sec`, `minimum_source_coverage_ratio`, and `freshness_time_base`.
- Produces: `compute_hard_gates(report, manifest_capabilities) -> Dict[str, bool]`, shared by accumulator and comparator.

- [ ] **Step 1: Update test helpers to express a complete formal window**

Construct metrics with explicit policy:

```python
def make_metrics(
        manifest_sha256='a' * 64, run_id='rate-1.0', replay_rate=1.0,
        wall_duration_sec=45.0):
    return EvaluationAccumulator(
        manifest=manifest(),
        manifest_sha256=manifest_sha256,
        run_id=run_id,
        software_revision='unit-test-revision',
        config_sha256='c' * 64,
        replay_rate=replay_rate,
        wall_duration_sec=wall_duration_sec,
        timing_policy='foxy_wall_time_scaled',
        freshness_time_base='arrival_monotonic',
    )


def observe_complete_healthy_replay(metrics):
    receive_step = int(1_000_000_000 / metrics.replay_rate)
    for index in range(42):
        source = index * 1_000_000_000
        receive = index * receive_step
        metrics.observe_topic('image', source, receive)
        metrics.observe_topic('lidar', source + 20_000_000, receive)
        metrics.observe_pair_offset(20_000_000)
        metrics.observe_localization('OBSERVATION_ONLY', 1)
```

Use at least 40.5 seconds of source span so the 0.90 coverage gate passes for a
45-second target.

- [ ] **Step 2: Write RED tests for strict window and policy evidence**

Add these tests:

```python
def test_one_sample_per_required_topic_cannot_pass():
    metrics = make_metrics()
    metrics.observe_topic('image', 1, 1)
    metrics.observe_topic('lidar', 1, 1)
    metrics.observe_pair_offset(0)
    metrics.observe_localization('OBSERVATION_ONLY', 1)
    report = metrics.finalize()
    assert report['gates']['required_topic_window_complete'] is False
    assert report['passed'] is False


def test_receive_source_ratio_must_match_replay_rate():
    metrics = make_metrics(replay_rate=2.0, wall_duration_sec=22.5)
    for index in range(42):
        stamp = index * 1_000_000_000
        metrics.observe_topic('image', stamp, stamp)
        metrics.observe_topic('lidar', stamp, stamp)
        metrics.observe_pair_offset(0)
        metrics.observe_localization('OBSERVATION_ONLY', 1)
    assert metrics.finalize()['gates']['replay_rate_consistent'] is False


def test_startup_localization_transitions_must_reach_declared_mode():
    local = pose_metrics(world_pose=False)
    local.observe_localization('OBSERVATION_ONLY', 1)
    local.observe_localization('LOCAL_SESSION', 1)
    assert local.finalize()['gates'][
        'manifest_localization_mode_respected'] is True
```

Also test local regression, world ordered startup, world failure to reach WORLD,
strict report/run field sets, finite spans, and `passed == all(gates.values())`.

- [ ] **Step 3: Write RED tests for node readiness boundaries**

Use a lightweight harness around unbound callbacks:

```python
def localization_diagnostic(mode='OBSERVATION_ONLY', epoch='1'):
    status = SimpleNamespace(
        name='semantic_search/localization',
        values=[
            SimpleNamespace(key='memory_mode', value=mode),
            SimpleNamespace(key='epoch_id', value=epoch),
        ],
    )
    return SimpleNamespace(status=[status])


def test_node_excludes_pre_ready_diagnostics_and_semantics_but_not_safety():
    node = object.__new__(SemanticSearchEvaluatorNode)
    node.metrics = make_metrics()
    node.started_ros_ns = 0
    node.diagnostic_callback(localization_diagnostic())
    node.region_callback(SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=1)),
        regions=[object()],
    ))
    node.intent_callback(SimpleNamespace(forward_permitted=True))
    assert node.metrics.localization_mode_sequence == []
    assert node.metrics.semantic_region_count == 0
    assert node.metrics.forward_permission_violations == 1

    node.metrics.observe_topic('image', 0, 0)
    node.metrics.observe_topic('lidar', 20_000_000, 20_000_000)
    node.diagnostic_callback(localization_diagnostic())
    node.region_callback(SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
        regions=[object()],
    ))
    assert node.metrics.localization_mode_sequence == ['OBSERVATION_ONLY']
    assert node.metrics.semantic_region_count == 1
```

- [ ] **Step 4: Run the RED tests**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot_semantic_search/test/test_evaluation.py -q
```

Expected: failures show missing 1.1 fields/gates and old pre-window behavior.

- [ ] **Step 5: Implement deterministic 1.1 metrics and shared gate computation**

Set constants:

```python
REPORT_SCHEMA_VERSION = '1.1.0'
TIMING_POLICY = 'foxy_wall_time_scaled'
MINIMUM_SOURCE_COVERAGE_RATIO = 0.90
REPLAY_RATE_RELATIVE_TOLERANCE = 0.15
```

`TopicSeries.report()` returns count/rates/hash plus integer first/last source
stamps and rounded source/receive spans. `EvaluationAccumulator` validates all
new constructor inputs, computes `target_source_duration_sec` as
`wall_duration_sec * replay_rate`, copies the exact manifest capabilities, and
calls one pure `compute_hard_gates(...)` function. Required-window acceptance
uses count >= 2 and source span >= target * 0.90. Replay-rate acceptance uses
the receive/source rate ratio for every required topic and 15 percent relative
tolerance.

Implement localization transitions as monotonic state ranks with required final
states: OBSERVATION_ONLY-only for capability-poor data, optional startup
OBSERVATION_ONLY then LOCAL_SESSION for local data, and optional
OBSERVATION_ONLY then LOCAL_SESSION then WORLD for world data. Missing final
mode or any regression fails.

Update the schema with strict nested required fields, `additionalProperties:
false`, finite non-negative numeric constraints, exact capability keys, and the
exact five gate keys.

- [ ] **Step 6: Implement evaluator readiness filtering**

Add an accumulator method returning whether every manifest-required topic has
at least one observation. In the node, diagnostics and semantic callbacks
return without counting until this is true. Do not guard `intent_callback`.
Pass `duration_sec`, `timing_policy`, and `freshness_time_base` from declared
parameters into the accumulator. Keep elapsed duration anchored at the first
sensor and keep finish-only stamp pairing.

- [ ] **Step 7: Run focused and package tests**

Run `test_evaluation.py`, then the full semantic package. Expected: all tests
pass, no non-finite JSON values, and all old pairing/resource tests remain
green.

- [ ] **Step 8: Commit Task 3**

```bash
git add \
  src/track_robot_semantic_search/track_robot_semantic_search/evaluation.py \
  src/track_robot_semantic_search/track_robot_semantic_search/evaluator_node.py \
  src/track_robot_semantic_search/schemas/evaluation_report.schema.json \
  src/track_robot_semantic_search/test/test_evaluation.py
git diff --cached --check
git commit -m "fix: require complete replay evidence windows"
```

---

### Task 4: Strict Three-Rate Comparator and Operator Contract

**Files:**
- Modify: `src/track_robot_semantic_search/track_robot_semantic_search/compare_reports.py`
- Modify: `src/track_robot_semantic_search/test/test_evaluation.py`
- Modify: `docs/guides/semantic-search/rosbag-workflow.md`
- Modify: `artifacts/semantic_search/reports/README.md`
- Modify: `src/track_robot_semantic_search/README.md`
- Modify: `docs/development/plans/semantic-search/2026-07-13-semantic-search-phase0-contracts-replay-profiling.md`

**Interfaces:**
- Replaces: `compare(paths)` with `compare(manifest_path, paths)`.
- CLI: `semantic_search_compare_reports --manifest MANIFEST REPORT_05 REPORT_10 REPORT_20`.
- Consumes: Task 3 `compute_hard_gates(report, manifest['capabilities'])`.
- Produces: comparator result with sorted `replay_rates`, source hashes,
  recomputed gates per path, failures, and `passed`.

- [ ] **Step 1: Write failing rate-set and manifest tests**

```python
def test_compare_requires_exact_formal_rate_set(tmp_path):
    manifest_path, reports = write_formal_reports(tmp_path)
    assert compare(manifest_path, reports[:1])['passed'] is False
    duplicate = [reports[1], reports[1], reports[1]]
    result = compare(manifest_path, duplicate)
    assert result['passed'] is False
    assert 'duplicate report path' in result['failures']


def test_compare_recomputes_and_rejects_forged_true_gates(tmp_path):
    manifest_path, reports = write_formal_reports(tmp_path)
    payload = json.loads(reports[0].read_text(encoding='utf-8'))
    payload['topic_metrics']['image']['source_span_sec'] = 1.0
    payload['gates'] = {name: True for name in payload['gates']}
    payload['passed'] = True
    reports[0].write_text(json.dumps(payload), encoding='utf-8')
    result = compare(manifest_path, reports)
    assert result['passed'] is False
    assert any('stored gates differ' in item for item in result['failures'])
```

Add this parameterized policy matrix, plus manifest/checksum and malformed
metric cases:

```python
@pytest.mark.parametrize(
    ('field', 'value', 'failure'),
    [
        ('timing_policy', 'other', 'timing policy'),
        ('wall_duration_sec', 44.0, 'wall duration'),
        ('freshness_time_base', 'source_clock', 'freshness time base'),
        ('minimum_source_coverage_ratio', 0.5, 'coverage ratio'),
    ],
)
def test_compare_rejects_wrong_formal_policy(
        tmp_path, field, value, failure):
    manifest_path, reports = write_formal_reports(tmp_path)
    payload = json.loads(reports[1].read_text(encoding='utf-8'))
    payload['run'][field] = value
    reports[1].write_text(json.dumps(payload), encoding='utf-8')
    result = compare(manifest_path, reports)
    assert result['passed'] is False
    assert any(failure in item for item in result['failures'])


def test_compare_rejects_manifest_capability_mismatch(tmp_path):
    manifest_path, reports = write_formal_reports(tmp_path)
    payload = json.loads(reports[0].read_text(encoding='utf-8'))
    payload['manifest_capabilities']['imu'] = True
    reports[0].write_text(json.dumps(payload), encoding='utf-8')
    assert compare(manifest_path, reports)['passed'] is False


def test_compare_rejects_manifest_checksum_mismatch(tmp_path):
    manifest_path, reports = write_formal_reports(tmp_path)
    payload = json.loads(reports[0].read_text(encoding='utf-8'))
    payload['manifest_sha256'] = 'f' * 64
    reports[0].write_text(json.dumps(payload), encoding='utf-8')
    result = compare(manifest_path, reports)
    assert result['passed'] is False
    assert any('manifest checksum' in item for item in result['failures'])


def test_compare_rejects_nonfinite_nested_metric(tmp_path):
    manifest_path, reports = write_formal_reports(tmp_path)
    payload = json.loads(reports[0].read_text(encoding='utf-8'))
    payload['topic_metrics']['image']['source_rate_hz'] = float('nan')
    reports[0].write_text(json.dumps(payload), encoding='utf-8')
    assert compare(manifest_path, reports)['passed'] is False
```

The forged-gate case covers receive/source scaling and stored `passed`; the
exact-rate-set case covers duplicate and missing rates.

- [ ] **Step 2: Run the comparator RED tests**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot_semantic_search/test/test_evaluation.py \
  -k 'compare' -q
```

Expected: new calls and assertions fail against the permissive comparator.

- [ ] **Step 3: Implement strict manifest-backed comparison**

Use `load_manifest()` and `sha256_file()` to validate the supplied manifest.
Reject a non-list or any path repeated after `Path.resolve()`. Require exactly
three successfully parsed reports and the unique rates 0.5, 1.0, and 2.0.
Validate the exact duration map `{0.5: 90.0, 1.0: 45.0, 2.0: 22.5}`, policy,
target, coverage, and freshness values. Compare manifest checksum and exact
capabilities. Call `compute_hard_gates()` for each report; reject any stored
gate or `passed` value that differs from recomputation. Preserve fail-closed
loading and provenance comparison.

Update CLI parsing:

```python
parser = argparse.ArgumentParser()
parser.add_argument('--manifest', required=True, type=Path)
parser.add_argument('reports', nargs='+', type=Path)
arguments = parser.parse_args(argv)
result = compare(arguments.manifest, arguments.reports)
```

- [ ] **Step 4: Run comparator and full semantic tests**

Expected: all comparator cases and the full semantic suite pass.

- [ ] **Step 5: Update operator and plan documentation**

Update every compare command to include `--manifest`. Add formal launch
overrides `timing_policy:=foxy_wall_time_scaled` and
`freshness_time_base:=arrival_monotonic`. Document report 1.1.0, 90 percent
source-window coverage, replay-rate scaling, exact three-rate acceptance, and
the fact that the comparator recomputes gates.

In both report README and replay guide state verbatim in substance: the legacy
baseline proves contracts, replay mechanics, and diagnostics only; it does not
prove semantic perception, 3D object memory, language grounding, motion safety,
or active-search safety.

- [ ] **Step 6: Run documentation and passive-scope checks**

```bash
git diff --check
rg -n "--manifest|foxy_wall_time_scaled|arrival_monotonic|0.90" \
  rosbags/semantic_search \
  docs/development/plans/semantic-search/2026-07-13-semantic-search-phase0-contracts-replay-profiling.md
rg -n -i "semantic perception|3D object memory|language grounding|motion safety" \
  rosbags/semantic_search
```

Use `rg --` when a pattern begins with `--`. Expected: all required policy and
capability-boundary text is present; `git diff --check` exits zero.

- [ ] **Step 7: Commit Task 4**

```bash
git add \
  src/track_robot_semantic_search/track_robot_semantic_search/compare_reports.py \
  src/track_robot_semantic_search/test/test_evaluation.py \
  src/track_robot_semantic_search/README.md \
  docs/guides/semantic-search/rosbag-workflow.md \
  artifacts/semantic_search/reports/README.md \
  docs/development/plans/semantic-search/2026-07-13-semantic-search-phase0-contracts-replay-profiling.md
git diff --cached --check
git commit -m "fix: enforce formal replay evidence contract"
```

---

### Task 5: Regenerate Formal Evidence and Freeze Phase 0

**Files:**
- Modify: `artifacts/semantic_search/manifests/legacy/human_tracking_lidar_20260706_150711.json`
- Delete: `artifacts/semantic_search/reports/phase0_baseline_2026-07-14.json`
- Create: `artifacts/semantic_search/reports/phase0_baseline_2026-07-15.json`
- Verify only: protected runtime/config files listed in Global Constraints

**Interfaces:**
- Consumes: installed manifest CLI, passive launch, evaluator 1.1.0, and strict comparator.
- Produces: one tracked 1.0x baseline plus temporary 0.5x/2.0x reports, all tied to one full implementation revision.

- [ ] **Step 1: Capture the fixed implementation revision and protected hashes**

```bash
git status --short
git rev-parse HEAD
sha256sum \
  src/track_robot_bringup/launch/human_tracking_simplified.launch.py \
  src/track_robot_bringup/launch/human_tracking_rosbag_replay.launch.py \
  src/track_robot_perception/config/human_tracking.yaml \
  src/track_robot_decision/config/outdoor_decision.yaml \
  src/track_robot_decision/config/motion_safety_supervisor.yaml \
  > /tmp/semantic_search_phase0_hardening_protected.sha256
```

Expected: tracked worktree clean before evidence generation. Use the full HEAD
SHA as `software_revision` in all three reports.

- [ ] **Step 2: Fresh build and test before replay**

```bash
source /opt/ros/foxy/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon build \
  --packages-select track_robot_interfaces track_robot_semantic_search
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test \
  --packages-select track_robot_interfaces track_robot_semantic_search
colcon test-result --verbose
```

Expected: both packages build, nonzero tests run, zero errors, failures, and
skips. Run the existing decision launch regression outside the DDS-restricted
sandbox if the sandbox reports `getifaddrs: Operation not permitted`.

- [ ] **Step 3: Regenerate and validate the legacy manifest from the real bag**

Use the original workspace's real directory, not the worktree symlink:

```bash
ros2 run track_robot_semantic_search semantic_search_manifest create-legacy \
  /home/track-robot/track_robot_ws/rosbags/human_tracking/recordings/human_tracking_lidar_20260706_150711 \
  artifacts/semantic_search/manifests/legacy/human_tracking_lidar_20260706_150711.json \
  --dataset-id human_tracking_lidar_20260706_150711 \
  --workspace-root /home/track-robot/track_robot_ws
ros2 run track_robot_semantic_search semantic_search_manifest validate \
  artifacts/semantic_search/manifests/legacy/human_tracking_lidar_20260706_150711.json
```

Expected: closed-bag checks pass; manifest still declares camera/LiDAR true and
IMU/local/world/query/annotation/active-motion false.

- [ ] **Step 4: Run each formal replay serially with complete teardown**

Define absolute formal inputs once:

```bash
WORKTREE=/home/track-robot/track_robot_ws/.worktrees/semantic-search-phase0
MANIFEST=$WORKTREE/artifacts/semantic_search/manifests/legacy/human_tracking_lidar_20260706_150711.json
OUTPUT_10=$WORKTREE/artifacts/semantic_search/reports/phase0_baseline_2026-07-15.json
REVISION=$(git rev-parse HEAD)
```

For each rate, start a fresh tegrastats process, launch, then non-looping bag
player. Use these exact launch values:

```text
use_sim_time:=false
start_evaluator:=true
timing_policy:=foxy_wall_time_scaled
freshness_time_base:=arrival_monotonic
manifest_path:=$MANIFEST
software_revision:=$REVISION
replay_rate:=0.5 duration_sec:=90.0 output_path:=/tmp/semantic_search_rate_05.json
replay_rate:=1.0 duration_sec:=45.0 output_path:=$OUTPUT_10
replay_rate:=2.0 duration_sec:=22.5 output_path:=/tmp/semantic_search_rate_20.json
```

Match each `ros2 bag play` rate to the launch rate. After the evaluator atomically
writes and the JSON parses: wait for that bag player to exit, stop/wait optional
diagnostic echo, stop/wait tegrastats, stop/wait the launch, then confirm the
exact bag path is absent from `ps -eo pid,ppid,stat,cmd` and both semantic node
names are absent from `ros2 node list`. Only then start the next rate.

- [ ] **Step 5: Run strict comparator and evidence assertions**

```bash
ros2 run track_robot_semantic_search semantic_search_compare_reports \
  --manifest artifacts/semantic_search/manifests/legacy/human_tracking_lidar_20260706_150711.json \
  /tmp/semantic_search_rate_05.json \
  artifacts/semantic_search/reports/phase0_baseline_2026-07-15.json \
  /tmp/semantic_search_rate_20.json
```

Expected: exact rates 0.5/1.0/2.0, recomputed gates all true,
`failures: []`, and `passed: true`. Parse all reports and assert schema 1.1.0,
full revision equality, manifest/config equality, source coverage >= 0.90,
arrival freshness, wall-time policy, nonempty core resources/tegrastats,
OBSERVATION_ONLY-only localization, zero motion intents, and zero forward
violations.

- [ ] **Step 6: Replace only the tracked evidence files and commit**

Remove the old baseline with `git rm`, then stage only the regenerated manifest,
old deletion, and new evaluator-generated 1.0x JSON. Do not stage `/tmp` reports,
tegrastats logs, raw bags, or symlinks.

```bash
git rm artifacts/semantic_search/reports/phase0_baseline_2026-07-14.json
git add \
  artifacts/semantic_search/manifests/legacy/human_tracking_lidar_20260706_150711.json \
  artifacts/semantic_search/reports/phase0_baseline_2026-07-15.json
git diff --cached --check
git diff --cached --name-status
git commit -m "test: harden phase zero replay baseline"
```

Expected staged scope: one manifest modification, one old report deletion, one
new report addition.

- [ ] **Step 7: Run the final completion audit**

Run fresh build/tests again; strict comparator again; decision launch
regression; `sha256sum -c /tmp/semantic_search_phase0_hardening_protected.sha256`;
no-motion publisher and `/cmd_vel` scans; install-prefix/launch-argument checks;
incomplete-marker scan; `git diff --check`; `git status --short`; exact bag/node
residual checks. Expected: every gate passes and tracked worktree is clean.

- [ ] **Step 8: Independent final review and Phase 0 freeze**

Generate a fixed review package from base `5ac1eea` through final HEAD. Dispatch
a fresh read-only reviewer against both Phase 0 specifications and both Phase 0
plans. Fix and re-review every Critical or Important finding. After approval,
record Phase 0 frozen status in the ignored local SDD ledger and proceed to a
separate Phase 1 design/plan; do not add Phase 1 code to this branch correction.
