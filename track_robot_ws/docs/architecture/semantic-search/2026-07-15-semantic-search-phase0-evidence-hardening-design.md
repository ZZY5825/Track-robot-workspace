# Semantic Search Phase 0 Evidence Hardening Design

**Date:** 2026-07-15

**Status:** Approved design; implementation pending

**Scope:** One bounded correction wave before Phase 0 is frozen and Phase 1 begins

## 1. Purpose

The existing Phase 0 run proves that the legacy rosbag can drive the passive
semantic-search diagnostics and evaluator. The final broad review found four
ways in which the reusable evidence tooling could accept incomplete input or
mis-evaluate a future field bag. This correction hardens the measuring system;
it does not add semantic perception, language grounding, object memory, or
motion behavior.

After this correction, Phase 0 is frozen. New feature work moves to Phase 1.

## 2. Considered Approaches

### 2.1 Minimal patches

Reject obvious sqlite sidecars, require three report filenames, and ignore the
first diagnostic message. This is fast but leaves the report unable to prove
the replay time policy, input coverage, or historical field-bag freshness.

### 2.2 Balanced evidence-contract correction — selected

Make closed-bag validation fail closed, version the report contract, record and
verify the wall-time replay policy, define an explicit measurement window, and
separate online source-time freshness from offline arrival-time freshness. Run
the existing three-rate replay again and replace the tracked baseline.

This addresses the confirmed correctness gaps without building a new replay
orchestration platform.

### 2.3 Full replay attestation service

Add a controller that starts every process, signs input/output inventories,
stores all three reports, and manages historical experiments. This is useful
later but is unnecessary Phase 0 infrastructure and would delay Phase 1.

## 3. Closed-Rosbag Integrity

Manifest creation must accept only a stable, closed rosbag2 sqlite3 directory.
The manifest builder will:

1. parse `metadata.yaml` through checked mappings and convert YAML, missing-key,
   type, and filesystem errors into a concise `ManifestError`;
2. require `storage_identifier: sqlite3` and a non-empty
   `relative_file_paths` list;
3. require every declared storage path to be a safe relative `.db3` path;
4. reject symlinked bag directories, metadata, or storage files, including
   links that point outside the dataset directory;
5. reject `.db3-wal`, `.db3-shm`, unlisted `.db3`, and other unexpected bag
   payload files;
6. open each storage file read-only and perform a lightweight SQLite
   `quick_check` before accepting it;
7. capture device, inode, size, and nanosecond modification time for metadata
   and every storage file before hashing, then require the same snapshot after
   hashing and metadata extraction;
8. hash only the verified closed-bag inventory in deterministic relative-path
   order.

The low-level checksum helper must not turn changing WAL/SHM contents into a
stable identity for manifest creation. Existing legacy evidence will be
regenerated from the real, non-symlinked bag in the original workspace.

## 4. Replay Evidence Contract 1.1.0

The evaluation report schema advances from `1.0.0` to `1.1.0`. Dataset and
annotation schemas remain at `1.0.0`.

Every report records:

- `replay_rate`;
- `timing_policy`, fixed to `foxy_wall_time_scaled` for these formal reports;
- `wall_duration_sec`;
- `target_source_duration_sec`;
- `minimum_source_coverage_ratio`, fixed to `0.90`;
- `freshness_time_base`, fixed to `arrival_monotonic` for historical wall-time
  replay;
- source start, source end, source span, receive span, count, source rate,
  receive rate, and source-sequence hash for every observed topic;
- the manifest capability snapshot used to determine required topics and
  localization expectations.

For the formal Foxy policy the only accepted triples are:

| Replay rate | Wall duration | Target source duration |
| ---: | ---: | ---: |
| 0.5 | 90.0 s | 45.0 s |
| 1.0 | 45.0 s | 45.0 s |
| 2.0 | 22.5 s | 45.0 s |

Required topics must contain at least two samples and cover at least 90 percent
of the target source duration. Where camera and LiDAR are both required, the
pair set must be non-empty and its P95 offset must remain at most 80 ms. Every
required topic therefore has a measurable nonzero source rate; receive rate
divided by source rate must agree with the declared replay rate within 15
percent.

The hard gates become:

- `required_topic_window_complete`;
- `sync_p95_at_most_80_ms`;
- `manifest_localization_mode_respected`;
- `replay_rate_consistent`;
- `no_forward_permission`.

`passed` is exactly the conjunction of these gates.

## 5. Explicit Evaluation Window

The evaluator starts elapsed-time and resource measurement at the first sensor
sample, as before. It separately marks diagnostic evaluation ready only after
every manifest-required sensor topic has produced a sample.

Before diagnostic readiness:

- localization diagnostics are ignored;
- semantic region, observation, and tracked-object messages are excluded from
  evidence counts;
- safety intent messages are **not** ignored. A forward-permission violation at
  any time during evaluator lifetime remains a hard failure.

Startup localization transitions are handled explicitly rather than hidden:

- observation-only manifests accept only `OBSERVATION_ONLY`;
- local-pose manifests may transition from startup `OBSERVATION_ONLY` to
  `LOCAL_SESSION`, but must reach `LOCAL_SESSION` and never regress;
- world-pose manifests may transition through `OBSERVATION_ONLY`,
  `LOCAL_SESSION`, and `WORLD` in that order, must reach `WORLD`, and never
  regress.

A truncated replay cannot pass merely because each topic appeared once.

## 6. Online and Historical Freshness

Localization health receives an explicit `freshness_time_base` parameter:

- `source_clock` is the default and preserves online behavior. Freshness uses
  ROS time and message header stamps.
- `arrival_monotonic` is selected explicitly for Foxy historical wall-time
  replay. Freshness uses monotonic arrival age, so old recorded timestamps are
  not permanently stale.

In both modes, source header stamps remain stored independently and a decreasing
source stamp triggers the existing epoch-reset behavior. Arrival-time freshness
must never hide source timestamp rollback.

The formal launch exposes the freshness choice and passes the same value into
the evaluator report, preventing a report from claiming a different time base
than the localization node used.

## 7. Comparator Behavior

The comparator CLI requires `--manifest MANIFEST` plus report paths. For Phase
0 formal acceptance it must:

1. require exactly three reports with the unique rate set `{0.5, 1.0, 2.0}`;
2. verify the exact policy, wall-duration mapping, 45-second source target,
   coverage threshold, and arrival-monotonic freshness mode;
3. validate the manifest and require its checksum and capabilities to match all
   reports;
4. require identical dataset, software revision, configuration checksum, model
   export list, and coverage declarations;
5. validate strict metric shapes and finite values;
6. recompute all five hard gates and `passed` from report metrics instead of
   trusting stored booleans;
7. reject duplicate paths, duplicate rates, missing rates, inconsistent
   receive/source scaling, forward-permission violations, and incomplete
   windows.

Sequence hashes remain visible report-only evidence under the approved
wall-time policy; they are not required to be identical because DDS delivery
can differ across rates.

## 8. Documentation and Capability Boundary

Operator-facing report and replay documentation will state exactly that the
legacy baseline proves contracts, replay mechanics, and diagnostics only. It
does not prove:

- semantic perception;
- 3D object memory;
- language grounding;
- motion safety or active-search safety.

The existing human-tracking launch, replay launch, perception configuration,
decision configuration, and motion-safety configuration remain protected and
unchanged.

## 9. Test and Evidence Strategy

Implementation follows red-green-refactor cycles. New regression tests cover:

- active WAL/SHM, symlinked files, unlisted storage, malformed metadata,
  unreadable/non-SQLite storage, and a file changing during hashing;
- one-frame and truncated replay rejection;
- pre-window diagnostics/semantic exclusion and pre-window safety retention;
- historical header stamps with arrival freshness and independent source-stamp
  rollback;
- missing, duplicate, and repeated report rates;
- incorrect wall durations, policies, freshness modes, metric shapes, stored
  gates, and receive/source scaling;
- recomputation of every hard gate and `passed`.

After unit and package tests pass, regenerate 0.5x, 1.0x, and 2.0x reports from
fresh processes. Delete the currently tracked 2026-07-14 baseline from the new
tree; Git history retains it. Only
`artifacts/semantic_search/reports/phase0_baseline_2026-07-15.json` is tracked as
the new 1.0x baseline, while the other two reports remain temporary comparator
inputs. Re-run the decision regression, protected-file hashes, no-motion scans,
install checks, and final independent review.

## 10. Delivery Boundary

This correction does not add a replay orchestrator, database, dashboard,
cryptographic signature service, model runtime, language encoder, DINO worker,
semantic memory, or motion component. When its tests, formal replay, and final
review pass, Phase 0 is complete and work moves to the separately designed
Phase 1.
