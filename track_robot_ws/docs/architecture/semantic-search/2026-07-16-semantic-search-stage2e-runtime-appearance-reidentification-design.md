# Stage 2E Runtime Appearance Memory and Re-identification Design

**Date:** 2026-07-16
**Status:** Approved design
**Parent design:** `docs/architecture/semantic-search/2026-07-15-generalized-multi-object-3d-semantic-memory-design.md`
**Implementation worktree:** `/home/track-robot/track_robot_ws/.worktrees/semantic-search-phase0`

## 1. Goal and completion boundary

Stage 2E connects the existing bounded `AppearanceMemory` and
`ReidentificationTracker` primitives to the Stage 2D runtime without splitting
persistent identity ownership across the ROS node and `MemoryCore`.

The Stage 2E software checkpoint is complete when:

- confirmed, non-ambiguous, quality-valid camera evidence updates at most four
  compatible appearance prototypes per object;
- public semantic objects expose accurate appearance summary metadata;
- the runtime can score a complete, bounded, one-to-one re-identification frame
  between active replacement candidates and eligible lost objects;
- re-identification requires deterministic global assignment and three
  consecutive compatible source frames;
- confirmed re-identification preserves the old global object ID through one
  atomic `MemoryCore` identity-transfer transaction;
- archived objects, ambiguous evidence, invalid descriptors, cross-domain
  evidence, stale evidence and partial/oversized frames cannot mutate identity;
- deterministic unit, integration, conversion and launch-contract tests pass;
- missing physical leave-and-re-enter evidence is reported as unverified and
  does not block this software checkpoint.

Stage 2E does not claim that re-identification thresholds are field-calibrated.
Production re-identification mutation remains disabled by default until a real
pilot report authorizes it. Stage 2F task/service wiring and Stage 2G physical
evaluation are outside this stage.

## 2. Considered architectures

### 2.1 Node-owned appearance and re-identification state

The ROS node could maintain maps from global IDs to appearance banks and
confirmation state. This is the smallest immediate patch, but node state could
diverge from `MemoryCore` during domain changes, capacity eviction, rollback,
copy-commit failure or object archival. It is rejected.

### 2.2 Separate visual identity manager

A standalone manager would provide clean pure interfaces, but a confirmed
identity transfer would still require a cross-component transaction involving
the manager, `MemoryCore`, source indexes and event publication. It adds a
second persistent owner and is rejected for Stage 2E.

### 2.3 MemoryCore-owned visual identity state

`MemoryCore` owns bounded per-object appearance banks and performs identity
transfer. The existing pure appearance and re-identification components remain
reusable scoring/state helpers. The ROS node constructs evidence frames on
copies, then commits the coordinator and `MemoryCore` together as Stage 2D
already does. This is the selected architecture.

## 3. Data model and ownership

`MemoryCore` remains the only owner of persistent object identity. It adds a
bounded map:

```cpp
std::map<GlobalObjectKey, AppearanceMemory> appearance_banks_;
```

The map is bounded by `MemoryCoreConfig::max_objects`. Banks are created lazily
on the first accepted appearance observation and erased on capacity eviction,
domain reset, memory reset or removal of a temporary duplicate during re-ID.
An archived object's bank may remain available for query/history until capacity
eviction, but archived objects never enter automatic re-identification.

`VisualMemorySupplement` carries an optional full `AppearanceDescriptor` and a
quality value instead of only the Stage 2D boolean. Descriptor metadata and
values remain bounded by the existing descriptor compatibility contract.

`MemoryObject` exposes summaries rather than raw descriptor vectors:

- appearance prototype count;
- active encoder ID, checkpoint ID and descriptor version;
- deterministic appearance summary ID;
- re-identification state (`not_required`, `pending`, `confirmed`, `rejected`).

Raw prototypes remain internal to `MemoryCore`. A read-only bounded view is
available to pure scoring code but is not published on ROS topics.

The appearance summary ID is a versioned, deterministic content fingerprint:
`appearance-v1-<16 lowercase hex digits>`. The 64-bit FNV-1a input is a
canonical field sequence containing encoder ID, checkpoint ID, descriptor
version and dimension followed by each prototype in vector order, including
its IEEE-754 value bits, quality weights and update count. Integral values are
fed most-significant byte first. An empty bank publishes an empty summary ID
and zero prototypes. This identifier is diagnostic, not a security primitive.

The public re-identification state has snapshot-scoped semantics. A lost target
is `pending` after one or two accepted consecutive assignments and `rejected`
when it was eligible but rejected in the most recent complete frame. An object
is `confirmed` only in the snapshot produced by its successful identity
transfer; the next ordinary update returns it to `not_required`. Objects that
were not eligible in the most recent complete frame are `not_required`.

## 4. Appearance update path

The Stage 2D association transaction first validates the whole visual frame.
For a matched supplement, `MemoryCore::supplement_visual()` applies semantic and
association metadata, then submits appearance evidence only when all of these
conditions hold:

- association is confirmed and unambiguous;
- the object is active and not prediction-only;
- appearance memory is enabled;
- the descriptor is finite, non-zero, L2-normalized and within the configured
  dimension/metadata bounds;
- encoder ID, checkpoint ID, version and dimension are compatible with the
  existing bank;
- quality is finite and at least the configured minimum.

`AppearanceMemory::update()` remains transactional. It updates the nearest
prototype by confidence-weighted normalized EMA or creates a sufficiently
different high-quality prototype while the bank has fewer than four entries.
The best-quality view remains separate.

Rejected appearance evidence does not invalidate an otherwise safe confirmed
camera attachment. It leaves the bank and appearance counter unchanged and
returns an explicit appearance decision/reason for diagnostics. This prevents a
malformed optional appearance feature from rewinding valid Stage 2D association
state while still failing closed for visual-memory mutation.

## 5. Re-identification frame and scoring

The node constructs a complete re-identification frame after the Stage 2D
association/supplement transaction on copies. Candidates are active objects
whose current confirmed visual observation has a compatible descriptor.
Targets are lost objects in the same memory epoch/domain with a non-empty
compatible appearance bank. Archived objects are excluded before scoring.

Each candidate-target pair applies hard gates for:

- same memory epoch and localization/spatial domain;
- lost target lifecycle and active candidate lifecycle;
- target age at or below the configured maximum;
- spatial distance at or below the configured maximum;
- compatible encoder, checkpoint, version and descriptor dimension;
- finite normalized appearance, geometry and semantic evidence.

The combined score keeps the existing pure weights:

```text
0.60 * appearance + 0.25 * geometry + 0.15 * permanent semantics
```

Appearance similarity is the maximum cosine similarity across the target's
bounded prototypes. Geometry uses bounded extent consistency. Semantic
similarity uses only permanent non-task labels. Missing geometry or semantic
evidence contributes zero rather than bypassing appearance and hard gates.

The coordinator builds the full matrix before enforcing limits of 64 active
candidates, 256 lost targets and 1,024 pairs. It rejects oversized, duplicate,
non-finite or cross-epoch frames as a whole. Deterministic Hungarian assignment
enforces one-to-one candidate-target matches. The configured match threshold is
the virtual runner-up when no second candidate exists, and row/column ambiguity
blocks confirmation.

Confirmation is keyed by `(old GlobalObjectKey, new GlobalObjectKey)` rather
than a one-shot visual candidate ID. Three consecutive increasing source frames
are required. Domain, memory epoch, LiDAR epoch, visual producer epoch or source
time rollback clears coordinator state. All updates occur on a coordinator copy
and commit only after the complete frame succeeds.

## 6. Atomic identity transfer

`MemoryCore::reidentify(domain, old_key, replacement_key, evidence)` performs
one transaction after the coordinator returns confirmed:

1. Revalidate that both keys belong to the current memory epoch and are
   distinct.
2. Require the old object to be lost but not archived and the replacement to be
   active.
3. Require the replacement's source key and confirmed visual key to remain
   unchanged from the scored evidence.
4. Remove the old LiDAR source index and the replacement object's source index.
5. Preserve the old `GlobalObjectKey`, first-seen time, permanent labels,
   history and appearance bank. Task/service-layer inspection state is outside
   `MemoryCore` and is not modified by this transaction.
6. Move the replacement's current LiDAR source key, metric position, velocity,
   extent, covariance, source timestamps, support, visibility and confirmed
   visual attachment onto the old object.
7. Merge bounded permanent labels deterministically. Merge the replacement
   appearance bank into the old bank through the same compatibility, quality,
   normalization and four-prototype rules; incompatible evidence is discarded
   without affecting identity transfer.
8. Remove the temporary replacement object and its appearance bank.
9. Point the replacement LiDAR source key to the preserved old global ID.
10. Mark re-identification confirmed and emit exactly one
    `kReidentified`/`EVENT_REIDENTIFIED` event.

Any failed precondition returns a rejected result with no mutation. The method
operates on the pending `MemoryCore` copy used by the node. The node commits the
pending coordinator and core together, then publishes the resulting event and
snapshot. No observer can see an intermediate duplicate or missing source
index.

## 7. Configuration and rollout

The normal Stage 2 configuration remains safe:

```yaml
appearance_memory_enabled: true
reidentification_shadow_mode: true
reidentification_mutation_enabled: false
reidentification_calibration_status: uncalibrated
```

Appearance updates are allowed only behind the already calibrated Stage 2D
camera attachment gate. Re-identification scoring may run in bounded shadow
mode for evidence collection, but mutation requires all of:

- `reidentification_mutation_enabled: true`;
- shadow mode disabled;
- a checked calibration report with `status=calibrated` and
  `reidentification_allowed=true`;
- exact agreement between the report and runtime age, spatial, appearance,
  combined-score, ambiguity and confirmation parameters.

No such physical re-ID calibration report exists at this checkpoint, so the
checked production profile cannot enable mutation. Pure and node-contract tests
exercise the mutation path with explicit in-memory software fixtures; reports
label that evidence as software-only.

Rollback requires only:

```yaml
reidentification_mutation_enabled: false
```

This leaves Stage 2D attachment and bounded appearance learning operational.

## 8. Error handling and bounded behavior

- Invalid optional appearance descriptors are rejected without changing the
  appearance bank or its counters.
- Duplicate or older visual observations remain idempotently rejected by the
  Stage 2D source-time rules.
- A rejected re-ID frame cannot advance confirmation state.
- One candidate cannot claim two lost identities, and one lost identity cannot
  accept two candidates in one frame.
- Archived objects return an explicit blocked decision.
- Capacity eviction and every memory/domain reset erase associated banks and
  re-ID confirmation state.
- Saturating counters prevent integer wraparound.
- Appearance prototype count never exceeds four; descriptor dimension and all
  matrices remain explicitly bounded.
- Debug/diagnostic publication is optional and never controls mutation.

## 9. Testing and evidence

TDD coverage includes:

- accepted descriptor creates a prototype through `supplement_visual()`;
- a compatible observation performs normalized EMA and a diverse view creates
  a bounded second prototype;
- low-quality, ambiguous, prediction-only, malformed, non-finite, zero-norm and
  incompatible evidence leave the bank byte-equivalent;
- appearance rejection does not roll back valid association metadata;
- public ROS conversion reports exact appearance summary metadata;
- complete 2x2 re-ID assignment is deterministic and one-to-one;
- threshold boundary, row/column ambiguity, duplicate keys and oversized frames
  fail closed;
- candidate/global key changes and frame gaps restart three-frame confirmation;
- domain/epoch/rollback reset confirmation;
- confirmed transfer preserves the old global ID, moves the new source key,
  removes the temporary duplicate, merges bounded evidence and emits one event;
- failed transfer, archived target and incompatible evidence do not mutate core
  state;
- deterministic replay produces byte-equivalent IDs and re-ID events;
- default and calibrated-profile launch contracts remain safe-off for mutation;
- full three-package build/tests pass with only the documented opt-in DDS skip;
- process inspection proves no Stage 2E ROS node, service or bag replay remains
  after testing.

The Stage 2E report records software test counts, exact commands, safe-default
configuration and `physical_reentry_pilot_executed=false`. It must not claim a
field re-identification rate or calibrated threshold without the required bag
and annotations.

## 10. Stage boundary

Stop after Stage 2E software verification and documentation. Do not begin live
Stage 2F task/service integration or Stage 2G physical evaluation in the same
execution.
