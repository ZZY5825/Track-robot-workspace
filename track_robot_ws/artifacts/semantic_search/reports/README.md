# Semantic Search Reports

Small JSON reports are versioned with the manifest and configuration checksums,
run ID, replay rate, software revision when available, source/receive rates, synchronization, evaluator
callback latency, localisation modes, resource samples, safety violations, and
explicit gates. Formal legacy evidence uses report schema `1.1.0` and exactly
three unique reports: 0.5x for 90.0 wall seconds, 1.0x for 45.0 wall seconds,
and 2.0x for 22.5 wall seconds. All runs use
`timing_policy=foxy_wall_time_scaled`,
`freshness_time_base=arrival_monotonic`, a 45.0-second source target, and
minimum source coverage `0.90`; each required topic also needs at least two
messages.

Run `semantic_search_compare_reports` with `--manifest` and all three reports.
The comparator validates the immutable manifest binding and matching
provenance, recomputes all five hard gates, and rejects any stored gates or
`passed` value that differs from recomputation.

Raw tegrastats, bags, images, masks, feature tensors, and model weights remain
external. The legacy baseline proves contracts, replay mechanics, and
diagnostics only; it does not prove semantic perception, 3D object memory,
language grounding, motion safety, or active-search safety.

Phase 2 camera/LiDAR calibration reports are also fail-closed. A report may
permit the Stage 2D attachment dependency only when it is generated from the
versioned sample schema with sufficient positive/negative human annotations,
positive hard-gate passes, separable normalized soft scores, precision at
least 95%, and recall at least 80%.
`phase2_association_calibration_2026-07-16.json` is calibrated from 22 unique
human-positive source frames and 79 negative pairs extracted from the real
`human_tracking_lidar_20260706_145752` RGB/LiDAR bag. At its selected threshold
the reviewed pilot has 100% precision and 81.8% recall. Stage 2D consumes this
dependency only through an explicit calibrated profile; the normal profile
remains shadow/safe-off.

`phase2_stage2d_runtime_2026-07-16.json` records the Stage 2D runtime software
gate: bounded global assignment, stable-key confirmation, delayed visual
supplementation without metric-state rewind, calibration fail-closed checks,
and the package-test result. It is software integration evidence, not a claim
that the opt-in DDS or physical re-entry pilot has run.

`phase2_stage2e_runtime_2026-07-16.json` records the Stage 2E runtime software
checkpoint: bounded appearance banks, current-frame descriptor eligibility,
bounded deterministic one-to-one re-identification, three-frame confirmation,
atomic identity transfer and an executed synthetic DDS leave/re-entry fixture.
It also records the safe-off production default and zero remaining test ROS
processes. It is not physical robot evidence: no field re-entry pilot or
re-identification threshold calibration has been performed.

`phase2_stage2f_runtime_2026-07-17.json` records the Stage 2F runtime software
checkpoint: typed task transport, bounded synchronized service state,
get/query/inspection/reset services, enriched active views and a reliable
transient-local zero-or-one best-candidate contract. The explicit synthetic
DDS suite passed task delivery, appearance-backed ranking, inspection, reset,
unhealthy-localization invalidation, pending-task rollback and domain-change
paths, with no ROS process left running. Every checked production profile
still has `best_candidate_threshold_calibrated=false`; no physical task-ranking
pilot or production threshold calibration is claimed.

`phase2_deterministic_replay_2026-07-16.json` is direct synthetic regression
evidence: the C++ normalized replay ran twice and produced the same output
SHA-256. It proves deterministic mechanics only. It does not prove field
association accuracy, re-ID, localization quality or Jetson resources.

`phase2_evaluation_2026-07-16.json` deliberately has status `unavailable`.
It binds to the only checked legacy manifest and shows all twelve field
scenarios, annotations, matched predictions, runtime/resource samples and
human-tracking regression evidence are absent. Null metrics mean “not
measured”; the report must be replaced with a new pilot-manifest-bound report
after following
`docs/guides/semantic-search/phase2-recording-and-evaluation.md`.

`phase2_deterministic_replay_2026-07-17.json` repeats the normalized replay
after Stage 2G event-map hardening. Both output hashes are byte-identical. The
new stable event-name test covers every memory event added through Stage 2F.

`phase2_evaluation_2026-07-17.json` uses strict report schema `2.0.0`. It adds
frozen task-threshold provenance, task recall and hard-negative gates, complete
semantic-path latency, a 30-minute source-span gate and CUDA reserved-memory
P95. It is intentionally `unavailable` because the workspace still has no
qualifying Phase 2 pilot, final annotations, task calibration dataset or Jetson
profile.

`phase2_stage2g_runtime_2026-07-17.json` is the Stage 2G software checkpoint.
It records the warning-clean four-package build, 599-test default regression,
explicit 3/3 DDS run, deterministic replay and zero remaining ROS processes.
It does not claim physical Phase 2 completion or a production best-candidate
threshold.
