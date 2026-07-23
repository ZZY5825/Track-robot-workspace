# Stage 2F Runtime Task Ranking and Memory Services Design

**Date:** 2026-07-17
**Status:** Approved for direct implementation by the user's 2026-07-17 instruction

## 1. Objective and boundary

Stage 2F connects the existing pure task-relevance and memory-service policies
to `semantic_memory_node`. It adds the `/semantic_memory/tasks` input, live
get/query/inspection/reset services, and a fail-closed
`/semantic_memory/best_candidate` output.

This stage does not add a text encoder to C++, change global identity rules,
enable uncalibrated re-identification, perform physical evaluation, or enter
Stage 2G. The best-candidate threshold remains uncalibrated and disabled in
checked production profiles. A test-only runtime configuration may enable it
to exercise the complete software path.

## 2. Considered approaches

### A. Separate synchronized runtime read model — selected

Keep `MemoryCore` as the only owner of object identity, geometry, permanent
semantic evidence and appearance banks. Extend `MemoryServiceStore` so it can
transactionally synchronize a complete bounded snapshot while preserving the
inspection state of surviving public keys. A small runtime coordinator builds
task evidence from the current core snapshot, owns the active task overlay,
and exposes deterministic service records and full object views.

This preserves the approved isolation boundary and makes the ROS layer a thin
transport adapter. The cost is one bounded copy of at most 256 compact object
records after a memory update.

### B. Put task and inspection state directly in `MemoryCore`

This gives one data structure but mixes replaceable task state with permanent
identity state and makes query-only descriptor scoring mutate or duplicate the
core. It is rejected because task replacement must not affect permanent
memory.

### C. Implement all behavior directly in ROS callbacks

This is the smallest initial diff, but it duplicates filtering, sorting and
reason mapping and is difficult to test without DDS. It is rejected because
service correctness should remain testable as pure C++.

## 3. Ownership and data flow

`MemoryCore` remains authoritative. After every successful LiDAR, visual,
re-identification, inspection or reset transaction, the node synchronizes the
complete `MemoryUpdateResult` into `RuntimeTaskServiceCoordinator`.

The coordinator stores:

- at most 256 copied `MemoryObject` read records;
- an inspection state for each surviving public object key;
- one optional active `SemanticTaskEvidence` plus its bounded query text;
- one bounded task relevance value per eligible object;
- the current memory epoch and an event cursor.

Synchronization validates the epoch, object count, unique keys and exact task
evidence key set before committing. A new memory epoch clears inspection
states and stale records. The active task remains available so newly observed
objects can be ranked after a reset or localization-domain change.

Task flow is:

1. Receive one reliable depth-one `SemanticTask`.
2. Validate producer epoch, query ID/version, non-negative source stamp,
   bounded non-empty provenance text and normalized descriptor provenance.
3. On task producer-epoch change or source-time rollback, clear the previous
   task overlay before accepting the new task.
4. Recompute all eligible objects from the maximum compatible appearance
   prototype and conservative permanent semantic evidence.
5. Synchronize scores without changing object keys, prototypes, lifecycle,
   spatial state or permanent labels.

C++ does not embed raw text. Permanent semantic evidence contributes only
when a bounded ASCII-normalized permanent label exactly equals the normalized
query text; otherwise the descriptor score is used alone. This avoids
inventing an uncalibrated fuzzy language model.

## 4. ROS contracts

### Topics

- Subscribe `/semantic_memory/tasks` as reliable, keep-last depth 1.
- Publish `/semantic_memory/best_candidate` as reliable transient-local
  `SemanticObjectArray`, containing zero or one object. Zero objects is the
  explicit no-winner representation.
- Existing `/semantic_memory/active_objects` messages are enriched with active
  query ID/version, task relevance and inspection state from the read model.

The best-candidate publisher is fail-closed. It emits one object only when the
threshold is explicitly calibrated, an active task exists, and the winner is
confirmed, active, not inspected and at or above the threshold. Otherwise it
emits an empty array.

### Services

- `GetSemanticObject`: distinguishes invalid key, stale epoch and missing key.
- `QuerySemanticObjects`:
  - `QUERY_ACTIVE_TASK` requires the exact active query ID/version;
  - `QUERY_DESCRIPTOR` validates and scores the supplied descriptor without
    replacing the active task;
  - pages contain at most 64 objects and use deterministic relevance/key order;
  - lifecycle and inspection flags are applied exactly as requested.
- `MarkSemanticObjectInspected`: validates the public key and state, preserves
  idempotence, updates the runtime read model, publishes the updated active
  snapshot/best candidate, and emits `EVENT_INSPECTION_CHANGED` only on change.
- `ResetSemanticMemory`: checks the optional expected epoch, advances the
  authoritative `MemoryCore` epoch before clearing, synchronizes the service
  store in the same copy-commit transaction, resets association/re-ID runtime
  state, and publishes an empty snapshot plus one reset event.

All service reason strings are stable, bounded to 256 characters and derived
from `ServiceReason` rather than callback-specific prose.

## 5. Reset and transaction rules

`MemoryCore::reset(domain)` is the only new identity mutation. It requires the
current domain, advances the memory epoch with the existing wrap-safe rule,
clears objects/source indexes/appearance banks, and returns a normal bounded
snapshot containing one `kMemoryReset` event.

The reset callback copies `MemoryCore`, `MemoryServiceStore` and task overlay;
it applies and cross-checks the new epoch on all copies before committing.
Failure leaves every live component unchanged.

Inspection and task changes never modify `MemoryCore`. They update only the
bounded read model and the ROS view. Re-identification preserves the old
public key, so its existing inspection state also survives synchronization;
the replacement key is pruned.

## 6. Configuration and safety

Checked profiles add:

```yaml
tasks_topic: /semantic_memory/tasks
task_queue_depth: 1
best_candidate_topic: /semantic_memory/best_candidate
publish_best_candidate: true
task_appearance_weight: 0.75
task_semantic_weight: 0.25
best_candidate_threshold_calibrated: false
best_candidate_minimum_relevance: 1.0
```

Startup rejects non-finite/negative weights, a non-positive weight sum,
invalid queue bounds, and any calibrated threshold outside `[0,1]`. The safe
default publishes only empty best-candidate arrays until Stage 2G provides
threshold evidence.

## 7. Testing and acceptance

Pure tests must prove:

- snapshot synchronization is transactional, bounded and preserves inspection
  only for surviving keys in the same epoch;
- exact semantic matching and descriptor compatibility are deterministic;
- task changes do not mutate permanent object evidence;
- active-task and descriptor queries are bounded, sorted and isolated;
- inspection is idempotent and re-identification-key preservation works;
- reset advances the core/store epoch together and stale keys fail closed;
- best-candidate selection returns zero or one confirmed uninspected object and
  remains disabled without calibration.

ROS contract tests must prove all topics, services, parameters and event
mappings exist. An opt-in DDS fixture must create an object with appearance
evidence, publish a task, observe a best candidate, exercise get/query/mark,
verify the candidate disappears after inspection, reset the epoch, and verify
the old key is stale. Every launched process must be terminated in `finally`.

The stage is complete only when a fresh four-package build and regression pass,
the explicit DDS fixture passes, independent review has no Critical/Important
findings, a valid Stage 2F evidence report records that the physical pilot and
threshold calibration remain unverified, and no ROS process remains.

## 8. Rollback

Setting `publish_best_candidate=false` disables the output without affecting
memory. Removing the task publisher leaves LiDAR/appearance memory unchanged
and services still provide unranked deterministic access. The complete Stage
2F runtime can be rolled back by reverting its commits; no stored database or
external migration exists.
