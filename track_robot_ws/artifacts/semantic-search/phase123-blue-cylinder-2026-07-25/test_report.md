# Phase 1-3 blue-cylinder live test

- Date: 2026-07-25
- ROS domain: 20
- Physical target: small tall blue cylindrical container on the centre table
- Formal queries:
  - `a blue cylindrical toothpaste-like container`
  - `a tall blue bottle on a table`

## Test conditions

- RoboSense RSHELIOS was connected to `eth0`.
- Jetson address was set to `192.168.1.102/24`.
- `can0` had to be configured after reboot at 500000 bit/s.
- The official ZED ROS wrapper opened the ZED 2i but did not publish its first
  image.
- A temporary UVC test publisher was therefore used to publish the physical
  ZED left image, real ZED intrinsics, and a prototype camera extrinsic.
- The UVC source published continuously at approximately 14.99 Hz.
- UVC results are degraded-path evidence and do not pass the official ZED
  wrapper launch path.

## Formal Phase 1 result

| Query | Duration | Region messages | Non-empty messages |
| --- | ---: | ---: | ---: |
| toothpaste-like container | 20 s | 78 | 0 |
| tall blue bottle | 20 s | 77 | 0 |

The image-to-model pipeline ran at approximately 3.8-4.2 Hz, and query IDs
were propagated correctly. The target did not pass the formal absolute
threshold of `0.25`.

A diagnostic run with threshold `0.0` measured:

- maximum language/fused score: `0.19176748394966125`
- best scoring window: the full `1280x720` frame
- non-empty region messages: 77/77

Verdict: **Phase 1 transport and inference pass; physical-target semantic
acceptance fails.**

## Phase 2 result

With real sensors and restored CAN:

- LiDAR: `17.45 Hz`
- IMU: `250.09 Hz`
- odometry: `49.99 Hz`
- LiDAR tracklet arrays: `8.40 Hz`
- maximum tracklets per array: `10`

At the formal Phase 1 threshold there were no semantic observations and no
association debug messages.

With diagnostic threshold `0.0`:

- semantic observation arrays: `4.15 Hz`
- semantic observations: 83 in 20 seconds
- association debug messages: 403 in 20 seconds (`20.14 Hz`)
- matched decisions: 0
- observed reasons:
  - `shadow_mode_attachment_disabled`
  - `field_of_view: projection outside field of view`
  - `size_ratio: value outside allowed range`

One valid projected pair had:

- total score: approximately `0.514`
- calibrated match threshold: approximately `0.6303`
- transform, calibration, spatial-domain, field-of-view, and source-time gates
  passed
- inside fraction: `1.0`
- projected IoU: approximately `0.0357`

The runtime configuration intentionally uses:

- `association_shadow_mode: true`
- `camera_attachment_enabled: false`

Verdict: **LiDAR tracking and the cross-modal scoring interface pass; a real
visual-to-LiDAR match for this target is not accepted.**

## Phase 3 result

Before CAN initialization, `/odom` was absent and localization remained
`local_pose_stale`; objects remained one-observation tentative objects.

After configuring `can0` and restarting Bunker:

- localization mode: `MEMORY_LOCAL_SESSION`
- localization reason: `world_disabled`
- active-object arrays: `8.40 Hz`
- maximum active objects per array: `10`
- sampled stable object lifecycle: `CONFIRMED`
- sampled stable object observations: `479`
- sampled object coordinate frame: `odom`
- sampled support state: LiDAR-only

The semantic best-candidate arrays remained empty because Phase 1 did not pass
its formal threshold and shadow mode does not attach diagnostic visual
observations to memory objects.

Verdict: **LiDAR-only persistent object memory passes; language-conditioned
best-candidate output fails for this target.**

## Overall gate

**The complete Phase 1 -> Phase 2 -> Phase 3 language-conditioned target path
does not pass.**

Passing components:

- physical LiDAR transport and decoding
- physical IMU
- Bunker odometry after CAN setup
- image transport through the temporary UVC path
- CLIP inference and query propagation
- LiDAR tracklet generation
- cross-modal projection/scoring in diagnostic shadow mode
- persistent LiDAR-only semantic memory

Blocking components:

- official ZED ROS wrapper publishes no image
- the small blue target scores `0.1918`, below the formal `0.25` threshold
- association is configured as non-mutating shadow mode
- diagnostic pair score is below the calibrated association threshold
- no semantic label reaches a memory object, so best candidate is empty

## Shutdown verification

All nodes and services started by this test were stopped. The default DDS graph
and the project Fast DDS graph contained no remaining test nodes, and UDP ports
6699/7788 had no remaining listeners.

Observed shutdown defects:

- `rslidar_sdk_node` throws `std::system_error` during shutdown.
- `bunker_base_node` exits with signal 11 during shutdown.
