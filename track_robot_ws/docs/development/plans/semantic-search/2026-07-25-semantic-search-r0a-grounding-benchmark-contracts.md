# Semantic Search R0A Grounding Benchmark Contracts Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a model-independent, leakage-resistant benchmark that selects an English open-vocabulary target localizer from frozen validation and held-out test evidence before any candidate is integrated into the Jetson ROS runtime.

**Architecture:** Keep model execution outside the benchmark package and ingest a bounded, versioned prediction document from each candidate. Validate English query scope and dataset integrity, select a score threshold on validation only, evaluate frozen test metrics, and choose a candidate only when accuracy, latency, memory, compatibility, and licence gates all pass.

**Tech Stack:** Python 3.8, NumPy, OpenCV, JSON Schema, pytest, ROS 2 Foxy Python packaging; model workers may run in isolated desktop or Jetson environments but communicate through JSON artifacts.

## Global Constraints

- The approved design is `docs/architecture/semantic-search/2026-07-25-semantic-search-phase1r-3r-visual-grounding-recovery-design.md`.
- Do not upgrade JetPack 5.0.2, Ubuntu 20.04, ROS 2 Foxy, system Python 3.8, CUDA 11.4, or TensorRT 8.4.1.
- The first-release query is printable-ASCII English describing one object with visible attributes; software validates the enforceable ASCII/length boundary and does not claim language identification.
- Test labels must be human-authored or human-verified; unreviewed teacher output is not test truth.
- Validation selects the operating threshold. Test metrics never change a threshold or gate.
- Train, validation, and test are separated by session, physical object instance, and image digest.
- Initial model gates are top-1 recall at IoU 0.50 at least 0.85, target-absent false-accept rate at most 0.05, median accepted-positive IoU at least 0.50, P95 complete-path latency at most 150 ms, semantic rate at least 5 Hz, and incremental CUDA reserve at most 1536 MiB.
- The current top-level `.git` directory is empty and unusable; each task ends with an explicit verification checkpoint instead of a commit.
- R0A installs no Grounding DINO, YOLO-World, SAM, or other model dependency and downloads no checkpoint.

---

## File structure

Create focused pure-Python modules in `track_robot_semantic_search`:

- `grounding_query.py`: enforce and normalize the first-release query contract.
- `grounding_dataset.py`: parse dataset documents, verify image identity, and reject split leakage.
- `grounding_predictions.py`: parse model-independent prediction artifacts.
- `grounding_evaluation.py`: threshold selection and held-out detection metrics.
- `grounding_selection.py`: release gates and deterministic candidate choice.
- `grounding_evaluation_cli.py`: atomic per-candidate report generation.
- `grounding_selection_cli.py`: atomic multi-candidate selection report.

Add one schema for each external document and keep tests beside the package's
existing semantic-search tests. Do not add model-specific imports to any of
these modules.

---

### Task 1: Define the enforceable English object-query contract

**Files:**
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/grounding_query.py`
- Create: `src/track_robot_semantic_search/test/test_grounding_query.py`

**Interfaces:**
- Produces: `GroundingQuery(raw_text: str, normalized_text: str)`.
- Produces: `normalize_grounding_query(value: str) -> GroundingQuery`.
- Rejects: empty, non-string, non-printable, non-ASCII, or over-160-character input.

- [ ] **Step 1: Write the failing query tests**

```python
import pytest

from track_robot_semantic_search.grounding_query import (
    normalize_grounding_query,
)


def test_normalizes_one_english_visible_attribute_query():
    query = normalize_grounding_query(
        '  A   tall blue cylindrical container  ')
    assert query.raw_text == 'A tall blue cylindrical container'
    assert query.normalized_text == 'a tall blue cylindrical container'


@pytest.mark.parametrize('value', [
    '',
    '   ',
    '蓝色杯子',
    'blue\\ncontainer',
    'x' * 161,
    None,
])
def test_rejects_out_of_contract_query(value):
    with pytest.raises(ValueError, match='grounding query'):
        normalize_grounding_query(value)


def test_punctuation_needed_by_open_vocabulary_prompt_is_preserved():
    query = normalize_grounding_query('a blue, toothpaste-like container')
    assert query.normalized_text == 'a blue, toothpaste-like container'
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```bash
python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_grounding_query.py
```

Expected: test collection fails because `grounding_query` does not exist.

- [ ] **Step 3: Implement the complete query contract**

```python
from dataclasses import dataclass


@dataclass(frozen=True)
class GroundingQuery:
    raw_text: str
    normalized_text: str


def normalize_grounding_query(value: str) -> GroundingQuery:
    if not isinstance(value, str):
        raise ValueError('grounding query must be a string')
    if any(ord(character) < 0x20 or ord(character) > 0x7e
           for character in value):
        raise ValueError(
            'grounding query must contain printable ASCII only')
    raw = ' '.join(
        part for part in value.strip().split(' ') if part)
    if not raw or len(raw) > 160:
        raise ValueError(
            'grounding query must contain 1 to 160 characters')
    return GroundingQuery(raw_text=raw, normalized_text=raw.lower())
```

Check the original value before normalizing spaces so tabs and newlines are
rejected rather than silently normalized.

- [ ] **Step 4: Run the focused tests and verify GREEN**

Run the Step 2 command. Expected: all query tests pass.

- [ ] **Step 5: Record checkpoint**

Record the two created files and the exact passing test count in the execution
notes.

---

### Task 2: Add a versioned grounding dataset and split-integrity validator

**Files:**
- Create: `src/track_robot_semantic_search/schemas/grounding_dataset.schema.json`
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/grounding_dataset.py`
- Create: `src/track_robot_semantic_search/test/test_grounding_dataset.py`

**Interfaces:**
- Consumes: Task 1 `normalize_grounding_query`.
- Produces: `GroundingBox(x: float, y: float, width: float, height: float)`.
- Produces: `GroundingCase(case_id: str, split: str, image_path: Path, image_sha256: str, session_id: str, physical_object_id: str, query: GroundingQuery, target_present: bool, boxes: Tuple[GroundingBox, ...], scenario_tags: Tuple[str, ...], label_review_status: str)`.
- Produces: `GroundingDataset(dataset_id: str, cases: Tuple[GroundingCase, ...])`.
- Produces: `load_grounding_dataset(document_path: Path, verify_images: bool = True) -> GroundingDataset`.

- [ ] **Step 1: Write failing schema and loader tests**

Create test fixtures under `tmp_path` with a real 8-by-8 PNG written through
OpenCV. Cover:

```python
def test_loads_verified_positive_and_negative_cases(tmp_path):
    dataset = load_grounding_dataset(
        write_dataset_fixture(tmp_path, split_cases()))
    assert dataset.dataset_id == 'grounding-r0'
    assert [case.split for case in dataset.cases] == [
        'train', 'validation', 'test']
    assert dataset.cases[0].boxes[0] == GroundingBox(1.0, 2.0, 3.0, 4.0)


@pytest.mark.parametrize('mutation,reason', [
    ('positive_without_box', 'target_present'),
    ('negative_with_box', 'target_present'),
    ('box_outside_image', 'image bounds'),
    ('wrong_digest', 'sha256'),
    ('duplicate_case_id', 'case_id'),
    ('unsafe_relative_path', 'relative path'),
])
def test_rejects_invalid_dataset_case(tmp_path, mutation, reason):
    with pytest.raises(ValueError, match=reason):
        load_grounding_dataset(
            write_dataset_fixture(tmp_path, split_cases(), mutation))


@pytest.mark.parametrize('leakage', [
    'session_id', 'physical_object_id', 'image_sha256'])
def test_rejects_train_validation_test_leakage(tmp_path, leakage):
    with pytest.raises(ValueError, match='split leakage'):
        load_grounding_dataset(
            write_leaking_fixture(tmp_path, leakage))
```

The schema test loads `grounding_dataset.schema.json` and asserts:

```python
assert schema['properties']['schema_version']['const'] == '1.0.0'
assert schema['additionalProperties'] is False
assert set(schema['properties']['cases']['items']['required']) == {
    'case_id', 'split', 'image_relative_path', 'image_sha256',
    'image_width', 'image_height', 'session_id', 'physical_object_id',
    'query_text', 'target_present', 'ground_truth_boxes_xywh',
    'scenario_tags', 'label_review_status',
}
```

- [ ] **Step 2: Run focused tests and verify RED**

```bash
python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_grounding_dataset.py
```

Expected: missing module and schema failures.

- [ ] **Step 3: Create the complete dataset schema**

The root document is:

```json
{
  "schema_version": "1.0.0",
  "dataset_id": "grounding-r0",
  "cases": [
    {
      "case_id": "session-a-frame-0001-query-1",
      "split": "train",
      "image_relative_path": "images/session-a-frame-0001.png",
      "image_sha256": "64-lowercase-hex-characters",
      "image_width": 1280,
      "image_height": 720,
      "session_id": "session-a",
      "physical_object_id": "blue-container-01",
      "query_text": "a tall blue cylindrical container",
      "target_present": true,
      "ground_truth_boxes_xywh": [[600.0, 320.0, 30.0, 110.0]],
      "scenario_tags": ["cluttered", "distance_1_to_2m"],
      "label_review_status": "human_verified"
    }
  ]
}
```

Constrain `split` to `train`, `validation`, or `test`;
`label_review_status` to `human_authored` or `human_verified`; every path to
the existing safe-relative-path regular expression; SHA-256 to lowercase
64-hex; dimensions to positive integers; boxes to exactly four finite
non-negative numbers with positive width and height; scenario tags to at most
16 unique non-empty strings; and cases to at most 100000 entries.

- [ ] **Step 4: Implement parsing, image verification, and leakage checks**

Use focused helpers with these signatures:

```python
def _safe_relative_path(value: object) -> str
def _sha256_file(path: Path) -> str
def _box_from_xywh(value: object, width: int, height: int) -> GroundingBox
def _reject_split_leakage(cases: Sequence[GroundingCase]) -> None
```

`load_grounding_dataset()` must:

1. parse UTF-8 JSON and require schema version `1.0.0`;
2. reject unknown root and case keys;
3. normalize every query with Task 1;
4. require a non-empty box list exactly when `target_present` is true;
5. reject boxes outside declared image bounds;
6. reject duplicate case IDs;
7. when `verify_images` is true, require a regular non-symlink image, verify
   SHA-256, decode with `cv2.imread`, and compare decoded dimensions;
8. reject a `session_id`, non-empty `physical_object_id`, or image SHA-256 that
   occurs in more than one split.

Do not require `physical_object_id` for target-absent cases; represent it as an
empty string and exclude it from leakage indexing.

- [ ] **Step 5: Run focused tests and verify GREEN**

Run the Step 2 command. Expected: all dataset tests pass.

- [ ] **Step 6: Record checkpoint**

Record the schema, module, test file, and exact passing count.

---

### Task 3: Define model-independent grounding prediction artifacts

**Files:**
- Create: `src/track_robot_semantic_search/schemas/grounding_predictions.schema.json`
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/grounding_predictions.py`
- Create: `src/track_robot_semantic_search/test/test_grounding_predictions.py`

**Interfaces:**
- Consumes: Task 2 `GroundingBox`.
- Produces: `GroundingDetection(box: GroundingBox, score: float, label: str)`.
- Produces: `GroundingPrediction(case_id: str, complete_path_ms: float, detections: Tuple[GroundingDetection, ...])`.
- Produces: `GroundingPredictionSet(dataset_id: str, candidate_id: str, model_identity: Mapping[str, str], platform: Mapping[str, str], input_size: Tuple[int, int], incremental_cuda_reserved_mib: float, release_evidence: Mapping[str, bool], predictions: Mapping[str, GroundingPrediction])`.
- Produces: `load_grounding_predictions(path: Path) -> GroundingPredictionSet`.

- [ ] **Step 1: Write failing prediction-contract tests**

Cover a valid prediction document plus rejection of:

- duplicate or missing case IDs;
- more than 256 detections per case;
- non-finite latency or score;
- score outside `[0, 1]`;
- invalid or empty model/checkpoint/checksum identity;
- absolute checkpoint path in the report;
- negative or non-finite incremental memory;
- non-boolean runtime, platform-compatibility, or licence evidence;
- malformed input size;
- boxes with non-positive dimensions.

The valid assertion is:

```python
result = load_grounding_predictions(path)
assert result.candidate_id == 'yolo_world_s_1280'
assert result.input_size == (1280, 1280)
assert result.predictions['test-1'].detections[0].score == 0.91
```

- [ ] **Step 2: Run focused tests and verify RED**

```bash
python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_grounding_predictions.py
```

Expected: missing module and schema failures.

- [ ] **Step 3: Create the exact prediction schema**

Use this root shape:

```json
{
  "schema_version": "1.0.0",
  "dataset_id": "grounding-r0",
  "candidate_id": "yolo_world_s_1280",
  "model_identity": {
    "implementation": "yolo-world",
    "code_revision": "full-revision-string",
    "checkpoint_id": "yolo_world_v2_s.pth",
    "checkpoint_sha256": "64-lowercase-hex-characters",
    "licence": "GPL-3.0"
  },
  "platform": {
    "role": "jetson_candidate",
    "hardware": "Jetson AGX Orin",
    "os": "Ubuntu 20.04",
    "python": "3.8.10",
    "pytorch": "1.13.0",
    "device": "cuda"
  },
  "input_size": [1280, 1280],
  "incremental_cuda_reserved_mib": 1024.0,
  "release_evidence": {
    "runtime_available": true,
    "platform_compatible": true,
    "licence_approved": true
  },
  "predictions": [
    {
      "case_id": "test-1",
      "complete_path_ms": 80.0,
      "detections": [
        {
          "box_xywh": [600.0, 320.0, 30.0, 110.0],
          "score": 0.91,
          "label": "a tall blue cylindrical container"
        }
      ]
    }
  ]
}
```

All root and nested objects use `additionalProperties: false`. Bound prediction
records to 100000 and detections per record to 256. The report stores
identifiers and checksums, never an absolute local checkpoint path. Require all
three `release_evidence` fields to be booleans. A desktop teacher sets
`platform_compatible` to false and remains evaluable but cannot be selected for
the Jetson runtime.

- [ ] **Step 4: Implement the strict parser**

Use pure Python validation so runtime behavior does not depend on the optional
`jsonschema` package. Reject unknown fields, duplicate case IDs, non-finite
numbers, unbounded strings, invalid SHA-256, malformed sizes, and invalid
detection boxes. Preserve detections in input order because the evaluator
sorts by score with deterministic geometry ties.

- [ ] **Step 5: Run focused tests and verify GREEN**

Run the Step 2 command. Expected: all prediction tests pass.

- [ ] **Step 6: Record checkpoint**

Record the schema, parser, tests, and exact passing count.

---

### Task 4: Select validation thresholds and evaluate held-out localization

**Files:**
- Create: `src/track_robot_semantic_search/schemas/grounding_evaluation_report.schema.json`
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/grounding_evaluation.py`
- Create: `src/track_robot_semantic_search/test/test_grounding_evaluation.py`

**Interfaces:**
- Consumes: Tasks 2 and 3 dataset/prediction values.
- Produces: `intersection_over_union(first: GroundingBox, second: GroundingBox) -> float`.
- Produces: `metrics_at_threshold(cases, predictions, threshold) -> Mapping[str, object]`.
- Produces: `select_validation_threshold(cases, predictions) -> Mapping[str, object]`.
- Produces: `evaluate_grounding_candidate(dataset, prediction_set) -> Mapping[str, object]`.

- [ ] **Step 1: Write failing metric tests**

Use exact synthetic boxes and scores to prove:

```python
def test_iou_is_exact_for_known_overlap():
    first = GroundingBox(0.0, 0.0, 10.0, 10.0)
    second = GroundingBox(5.0, 0.0, 10.0, 10.0)
    assert intersection_over_union(first, second) == pytest.approx(1.0 / 3.0)


def test_top1_uses_highest_score_then_geometry_tie_break():
    metrics = metrics_at_threshold(cases, predictions, 0.5)
    assert metrics['top1_recall_iou_50'] == 1.0
    assert metrics['median_accepted_positive_iou'] == 1.0


def test_absent_prediction_counts_as_false_accept():
    metrics = metrics_at_threshold(absent_cases, predictions, 0.5)
    assert metrics['target_absent_false_accept_rate'] == 1.0


def test_validation_threshold_never_reads_test_cases():
    selected = select_validation_threshold(
        validation_cases, validation_predictions)
    assert selected['threshold'] == pytest.approx(0.7)
    assert selected['status'] == 'selected'
```

Also cover no positive validation cases, no absent validation cases, no
threshold meeting the quality gates, empty detections, multiple ground-truth
boxes, finite P95 interpolation, and missing or extra prediction records.

- [ ] **Step 2: Run focused tests and verify RED**

```bash
python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_grounding_evaluation.py
```

Expected: missing evaluation module and schema failures.

- [ ] **Step 3: Implement deterministic metrics**

For each case:

1. retain detections with `score >= threshold`;
2. sort by descending score, then `x`, `y`, `width`, `height`, and label;
3. use only the first detection for top-1 release metrics;
4. for a positive case, use the maximum IoU against its human-verified boxes;
5. count recall success when IoU is at least 0.50;
6. for an absent case, count any retained detection as a false accept.

Report:

```python
{
    'positive_case_count': int,
    'absent_case_count': int,
    'top1_recall_iou_50': float,
    'target_absent_false_accept_rate': float,
    'median_accepted_positive_iou': float,
    'empty_output_rate': float,
}
```

Metrics requiring a missing class of case are `None`, not zero or pass.

- [ ] **Step 4: Implement validation-only threshold selection**

Candidate thresholds are `1.0`, every unique finite detection score in the
validation records, and `0.0`, sorted descending. A threshold is eligible only
when validation has at least one positive and one absent case and it achieves:

- recall at least 0.85;
- false-accept rate at most 0.05;
- median accepted-positive IoU at least 0.50.

Select the highest eligible threshold, which is the most conservative
operating point meeting recall. If none passes, return:

```python
{
    'status': 'unavailable',
    'threshold': None,
    'reason': 'no_validation_threshold_meets_quality_gates',
}
```

`evaluate_grounding_candidate()` verifies that dataset and prediction IDs and
case sets match, selects the threshold from validation only, evaluates test
cases once at that frozen threshold, and adds complete-path latency summary,
incremental CUDA reserve, release evidence, model identity, platform, dataset
content checksum, whether every test label is human reviewed, and per-scenario
test metrics. The checksum is the SHA-256 of canonical JSON containing the
dataset ID and all parsed case fields (including image SHA-256 and review
status) sorted by case ID; it is independent of manifest formatting and local
absolute paths.

- [ ] **Step 5: Create and test the report schema**

Require schema version `1.0.0`, dataset and candidate IDs, model identity,
platform, validation selection, test metrics, resource metrics, release gates,
and reasons. Permit `null` only for genuinely unavailable metrics or threshold.
Use `additionalProperties: false` at every object level.

- [ ] **Step 6: Run focused tests and verify GREEN**

Run the Step 2 command. Expected: all evaluation tests pass.

- [ ] **Step 7: Record checkpoint**

Record the evaluator, schema, tests, and exact passing count.

---

### Task 5: Add fail-closed grounding candidate selection

**Files:**
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/grounding_selection.py`
- Create: `src/track_robot_semantic_search/test/test_grounding_selection.py`

**Interfaces:**
- Consumes: Task 4 evaluation reports.
- Produces: `GroundingSelection(status, selected_candidate_id, rejected, ranking)`.
- Produces: `select_grounding_candidate(reports: Sequence[Mapping[str, object]]) -> GroundingSelection`.

- [ ] **Step 1: Write failing selection tests**

```python
def test_selects_highest_recall_candidate_that_passes_every_gate():
    result = select_grounding_candidate([
        report('fast', recall=0.86, false_accept=0.04,
               median_iou=0.60, p95_ms=70.0, peak_mib=800.0),
        report('accurate', recall=0.93, false_accept=0.03,
               median_iou=0.64, p95_ms=130.0, peak_mib=1200.0),
        report('too_slow', recall=0.99, false_accept=0.01,
               median_iou=0.80, p95_ms=151.0, peak_mib=900.0),
    ])
    assert result.status == 'selected'
    assert result.selected_candidate_id == 'accurate'
    assert result.rejected['too_slow'] == ['latency_p95_at_most_150_ms']


def test_ties_use_false_accept_iou_latency_then_candidate_id():
    result = select_grounding_candidate(tied_reports())
    assert result.ranking == [
        'lower-false-accept',
        'higher-iou',
        'lower-latency',
        'lexical-id',
    ]


def test_no_passing_candidate_is_explicitly_unavailable():
    result = select_grounding_candidate([failed_report('candidate')])
    assert result.status == 'unavailable'
    assert result.selected_candidate_id is None
```

Also reject duplicate candidate IDs, mismatched dataset IDs, mismatched
dataset content checksums, malformed checksums, unreviewed test labels,
missing metrics, non-finite values, and non-boolean compatibility or licence
fields. Reports are comparable only when both `dataset_id` and the canonical
`dataset_checksum` match.

- [ ] **Step 2: Run focused tests and verify RED**

```bash
python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_grounding_selection.py
```

Expected: missing selection module failure.

- [ ] **Step 3: Implement exact hard gates and ranking**

Hard gates are:

```python
GATES = {
    'validation_threshold_selected': (
        lambda report:
        report['validation_selection']['status'] == 'selected' and
        0.0 <= report['validation_selection']['threshold'] <= 1.0),
    'runtime_available': lambda report: report['runtime_available'] is True,
    'platform_compatible': lambda report: report['platform_compatible'] is True,
    'licence_approved': lambda report: report['licence_approved'] is True,
    'human_reviewed_test_labels': (
        lambda report: report['human_reviewed_test_labels'] is True),
    'top1_recall_iou_50_at_least_0_85': (
        lambda report: report['test_metrics']['top1_recall_iou_50'] >= 0.85),
    'false_accept_rate_at_most_0_05': (
        lambda report:
        report['test_metrics']['target_absent_false_accept_rate'] <= 0.05),
    'median_iou_at_least_0_50': (
        lambda report:
        report['test_metrics']['median_accepted_positive_iou'] >= 0.50),
    'latency_p95_at_most_150_ms': (
        lambda report: report['resources']['p95_complete_path_ms'] <= 150.0),
    'semantic_rate_at_least_5_hz': (
        lambda report: report['resources']['semantic_rate_hz'] >= 5.0),
    'incremental_cuda_at_most_1536_mib': (
        lambda report:
        report['resources']['incremental_cuda_reserved_mib'] <= 1536.0),
}
```

Rank passing candidates by:

1. descending test recall;
2. ascending false-accept rate;
3. descending median IoU;
4. ascending P95 complete-path latency;
5. lexical candidate ID.

Selection remains unavailable when any required metric is `None`. Preserve a
complete rejection-reason list per candidate.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run the Step 2 command. Expected: all selection tests pass.

- [ ] **Step 5: Record checkpoint**

Record the module, tests, and exact passing count.

---

### Task 6: Package atomic evaluation and selection CLIs

**Files:**
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/grounding_evaluation_cli.py`
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/grounding_selection_cli.py`
- Modify: `src/track_robot_semantic_search/setup.py`
- Create: `src/track_robot_semantic_search/test/test_grounding_cli.py`

**Interfaces:**
- Produces command:
  `semantic_search_grounding_evaluate --dataset DATASET.json --predictions PREDICTIONS.json --output REPORT.json`.
- Produces command:
  `semantic_search_grounding_select --report REPORT.json [--report REPORT.json ...] --output SELECTION.json`.
- Uses: existing `manifest.write_json_atomic`.

- [ ] **Step 1: Write failing CLI tests**

Use `tmp_path` fixtures and invoke `run()` functions directly:

```python
def test_evaluation_cli_writes_report_atomically(tmp_path):
    exit_code = evaluation_run(dataset_path, predictions_path, output_path)
    assert exit_code == 0
    assert json.loads(output_path.read_text())['candidate_id'] == 'candidate-a'
    assert not list(tmp_path.glob('*.tmp'))


def test_selection_cli_returns_two_when_no_candidate_passes(tmp_path):
    exit_code = selection_run([failed_report_path], output_path)
    assert exit_code == 2
    assert json.loads(output_path.read_text())['status'] == 'unavailable'


def test_grounding_commands_are_packaged():
    source = Path(SETUP_PATH).read_text(encoding='utf-8')
    assert 'semantic_search_grounding_evaluate' in source
    assert 'semantic_search_grounding_select' in source
```

Also prove malformed inputs return non-zero, write no partial output, and emit
one concise error to stderr.

- [ ] **Step 2: Run CLI tests and verify RED**

```bash
python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_grounding_cli.py
```

Expected: missing CLI module or entry-point failures.

- [ ] **Step 3: Implement both CLIs**

Each CLI must separate:

```python
def run(...) -> int
def parser() -> argparse.ArgumentParser
def main(argv=None) -> None
```

`main()` calls `raise SystemExit(run(...))`. `run()` catches only expected
`OSError`, `UnicodeError`, `json.JSONDecodeError`, and `ValueError`, prints the
bounded reason to stderr, and returns `2`. It writes only through
`write_json_atomic()` after complete validation.

Add setup entry points:

```python
'semantic_search_grounding_evaluate = '
'track_robot_semantic_search.grounding_evaluation_cli:main',
'semantic_search_grounding_select = '
'track_robot_semantic_search.grounding_selection_cli:main',
```

- [ ] **Step 4: Run focused tests and verify GREEN**

Run the Step 2 command. Expected: all CLI tests pass.

- [ ] **Step 5: Record checkpoint**

Record the CLI modules, setup change, tests, and exact passing count.

---

### Task 7: Document R0A artifact production and verify package regression

**Files:**
- Create: `docs/guides/semantic-search/grounding-model-evaluation.md`
- Modify: `docs/README.md`
- Modify: `src/track_robot_semantic_search/README.md`

**Interfaces:**
- Documents the exact dataset, prediction, evaluation, and selection commands.
- Declares that model-specific R0B/R0C runners must emit the Task 3 artifact.

- [ ] **Step 1: Write the operator guide**

Include these exact commands:

```bash
source /opt/ros/foxy/setup.bash
source /home/track-robot/track_robot_ws/install/setup.bash

semantic_search_grounding_evaluate \
  --dataset /absolute/path/to/grounding_dataset.json \
  --predictions /absolute/path/to/candidate_predictions.json \
  --output /absolute/path/to/candidate_evaluation.json

semantic_search_grounding_select \
  --report /absolute/path/to/yolo_world_s_1280_evaluation.json \
  --report /absolute/path/to/grounding_dino_evaluation.json \
  --output /absolute/path/to/grounding_selection.json
```

Explain:

- how validation freezes the threshold before test evaluation;
- why sessions, physical object IDs, and image hashes cannot cross splits;
- that test labels require human review;
- that predictions contain no checkpoint path or raw model tensor;
- exit code `0` means a complete report or selected candidate;
- exit code `2` means invalid evidence or no passing candidate;
- R0A does not install or run a model;
- R0B and R0C each receive their own implementation plan after their execution
  environments are inventoried.

Link the guide from both documentation indexes.

- [ ] **Step 2: Run all new R0A tests**

```bash
python3 -m pytest -q \
  src/track_robot_semantic_search/test/test_grounding_query.py \
  src/track_robot_semantic_search/test/test_grounding_dataset.py \
  src/track_robot_semantic_search/test/test_grounding_predictions.py \
  src/track_robot_semantic_search/test/test_grounding_evaluation.py \
  src/track_robot_semantic_search/test/test_grounding_selection.py \
  src/track_robot_semantic_search/test/test_grounding_cli.py
```

Expected: all R0A tests pass.

- [ ] **Step 3: Run semantic-search package regression**

```bash
python3 -m pytest -q src/track_robot_semantic_search/test
```

Expected: all package tests pass with no regression.

- [ ] **Step 4: Build the affected ROS package**

```bash
source /opt/ros/foxy/setup.bash
colcon build \
  --base-paths src \
  --packages-select track_robot_semantic_search \
  --symlink-install
```

Expected: `track_robot_semantic_search` finishes successfully.

- [ ] **Step 5: Validate installed CLI help**

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
semantic_search_grounding_evaluate --help
semantic_search_grounding_select --help
```

Expected: both commands exit zero and show their required input/output
arguments.

- [ ] **Step 6: Verify no ROS process was left running**

R0A starts no ROS node. Still verify the project domain:

```bash
export ROS_DOMAIN_ID=20
ros2 node list
```

Expected: R0A introduced no node. Do not stop unrelated user-owned nodes.

- [ ] **Step 7: Record final R0A checkpoint**

Record:

- changed file list;
- focused and full pytest counts;
- colcon result;
- CLI help result;
- confirmation that no model was installed or downloaded;
- confirmation that no ROS node or model service was started.

R0A is complete only when all evidence above is present. R0B begins by
inventorying the desktop RTX environment and implementing a Grounding DINO
teacher runner that emits the frozen Task 3 prediction artifact.
