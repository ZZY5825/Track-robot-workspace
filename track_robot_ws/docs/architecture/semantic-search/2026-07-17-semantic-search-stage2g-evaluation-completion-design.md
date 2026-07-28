# Semantic Search Stage 2G Evaluation and Completion Design

**Date:** 2026-07-17

## 1. Purpose

Stage 2G closes the Phase 2 engineering loop with reproducible evaluation,
profiling, regression evidence and an explicit field-evidence boundary. It does
not change the semantic-memory runtime algorithms delivered by Stages 2B–2F.

The repository already contains manifest and annotation extensions, a
normalized deterministic replay path, a fail-closed evaluator and a recording
guide. Those tools currently prove software mechanics, but the evaluator is too
permissive for completion evidence: task ranking and resource data pass merely
by being present, the full semantic-path latency and long-duration requirements
are not enforced, and the production best-candidate threshold has no
independent calibration contract.

## 2. Evidence boundary

Stage 2G has two independently reported outcomes:

1. **Software-tooling gate:** contracts, calibration, evaluator, normalized
   replay and regressions work from a clean checkout.
2. **Physical completion gate:** a newly recorded, connected-TF Phase 2 bag,
   checked human annotations, frozen threshold calibration data, all twelve
   scenarios and a Jetson runtime/resource profile satisfy the gates.

Synthetic fixtures may pass the software-tooling gate. They never satisfy the
physical completion gate. If the required physical inputs are absent, the
checked Stage 2G report says `unavailable`, lists each missing dependency and
keeps every production best-candidate profile fail-closed.

## 3. Considered approaches

### A. Keep the existing evaluator unchanged

This is small but unsafe: a short synthetic runtime and any non-empty resource
array can currently satisfy gates that were intended to represent field
completion.

### B. Harden the existing evidence path (selected)

Extend the existing evaluator and schemas, add a deterministic calibration
tool, and preserve the normalized replay and CLI entry points. This minimizes
runtime risk and makes missing evidence explicit.

### C. Build a second end-to-end bag evaluator

This duplicates manifest, annotation and replay responsibilities and cannot be
validated without the missing physical bag. It is unnecessary for Stage 2G.

## 4. Evaluation contract

The Phase 2 evaluation report moves to schema `2.0.0`. The evaluator remains
deterministic and rejects malformed, non-finite or internally inconsistent
evidence.

### 4.1 Pilot correctness gates

- zero ID switches, duplicate objects and incorrect merges;
- camera–LiDAR association precision at least 0.95 and recall at least 0.80;
- re-identification and stale-reactivation success at least 0.80 when their
  required trials are present;
- measured 3D consistency is reported, without inventing a numeric Phase 2
  gate before a real error distribution exists;
- deterministic normalized replay;
- Phase 2 update rate at least 5 Hz;
- Phase 2 core latency P95 at most 50 ms;
- zero dropped updates and bounded queue/history growth.

### 4.2 Full Stage 2G gates

- all twelve approved scenario classes are present;
- task-relevant candidate recall is at least 0.90;
- the task-ranking threshold is frozen by a separate calibration report before
  final-evaluation evidence is consumed;
- complete semantic-path latency P95 is at most 150 ms;
- long-duration runtime is at least 1,800 seconds;
- incremental CUDA reserved memory is at most 1,536 MiB;
- a resource profile contains CPU, GPU, resident memory and CUDA samples;
- the existing human-tracking regression passes.

CPU, GPU and resident-memory percentiles are mandatory report fields but have
no invented numeric Phase 2 limits. CUDA reserved memory uses the approved
first-release hard limit.

## 5. Task-threshold calibration

A new pure Python calibration module consumes bounded JSONL samples with:

- calibration dataset ID and split;
- query ID and candidate ID;
- finite relevance score in `[0, 1]`;
- human `task_relevant` label;
- optional scenario ID.

The calibrator rejects duplicate candidate identities, empty positive or
negative classes, non-calibration splits and oversized inputs. It evaluates
deterministic threshold candidates and selects the highest threshold meeting
candidate recall `>= 0.90` and hard-negative false confirmation `<= 0.05`;
ties prefer lower false-confirmation rate and then higher precision. The output
binds to the canonical input SHA-256 and records all confusion counts and
metrics. A report is `calibrated` only when both quality gates pass and at least
30 positive and 30 hard-negative samples are present; otherwise it is
`insufficient_evidence`.

The final evaluator accepts the calibration report as a separate input,
validates its schema and hash-shaped provenance, and requires the threshold
used by task predictions to equal the frozen threshold. No Stage 2G code writes
that value into a production YAML unless real calibration evidence is present.

## 6. Task evaluation semantics

Task metrics are computed from all non-ignored annotations, not only matched
predictions. A positive task annotation with no candidate prediction is a false
negative. A selected negative candidate is a false confirmation. Per-query
best rank still reports top-1 accuracy and mean reciprocal rank. The evaluator
reports candidate recall, confirmed precision and hard-negative false
confirmation rate separately.

Predictions used for task evaluation contain `query_id`, `task_relevant`,
`task_rank`, `task_selected`, `task_relevance` and
`task_threshold`. Values must be bounded and mutually consistent. Selection is
recomputed from `task_relevance >= task_threshold`; stored contradictory
decisions are rejected.

## 7. Runtime and resource evidence

Runtime input adds:

- `semantic_path_latency_ms` samples;
- existing `semantic_memory_core` samples;
- duration, update source stamps, drop count and bounded-growth result.

Resource input adds `cuda_reserved_memory_mib` samples. All arrays are non-empty,
finite and non-negative. Percentiles are recomputed; precomputed values are not
trusted.

## 8. Compatibility and migration

- Existing Phase 0/1 manifests remain valid.
- Existing Phase 2 annotation fields remain valid; Stage 2G adds optional
  task-evaluation fields rather than changing earlier records.
- The evaluator emits only report schema `2.0.0`; the old checked unavailable
  report remains historical evidence and is replaced by a new Stage 2G
  unavailable report.
- Existing normalized replay and runtime ROS interfaces are unchanged.
- Production best-candidate output remains disabled until real calibration is
  reviewed and intentionally wired.

## 9. Error handling and determinism

- Missing required evidence yields exit code `2` and status `unavailable`.
- Complete but below-threshold evidence yields exit code `2` and status
  `failed`.
- Invalid evidence or an invalid calibration binding yields exit code `1` and
  no success claim.
- Identical canonical inputs produce byte-identical reports.
- Atomic writes are retained.

## 10. Verification and stopping condition

Verification includes unit/schema tests, deterministic replay twice, four
package workspace regression, explicit opt-in DDS regression when required,
Git diff checks and a final process inventory. All ROS nodes, bag players and
test services are stopped after testing.

Stage 2G is reported as physically complete only if the real field evidence
exists and passes. On this host, the expected honest stopping state is
`software_complete_field_evidence_unavailable` unless the required pilot bag,
annotations and Jetson profile are discovered during implementation.
