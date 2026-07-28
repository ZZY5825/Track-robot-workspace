# Semantic Search Rosbag Guide

## Safety and Scope

Phase 0 is passive. Do not start the live camera/LiDAR drivers while replaying
on the same ROS_DOMAIN_ID. Do not start decision, controller, planner, safety
motion, base, or cmd_vel nodes from this guide.

## Dataset Bundle

A recording is:

1. a closed rosbag2 directory;
2. one JSON manifest;
3. optional JSONL frame annotations;
4. one evaluator report;
5. optional raw tegrastats outside Git.

ROS 2 Foxy rosbag2 does not reliably preserve action service goal payloads.
Therefore query text, query ID, and source timestamp are recorded in the
adjacent manifest with semantic_search_manifest add-query.

## Record a New Field Bag

Terminal 1:

    source /opt/ros/foxy/setup.bash
    source ~/track_robot_ws/install/setup.bash
    export ROS_DOMAIN_ID=21
    ros2 bag record -o ~/track_robot_ws/rosbags/semantic_search/recordings/field_001 \
      /zed/zed_node/left/image_rect_color \
      /zed/zed_node/left/camera_info \
      /rslidar_points \
      /imu/data_raw \
      /odom \
      /localization/odometry \
      /tf \
      /tf_static \
      /semantic_search/regions \
      /semantic_search/observations \
      /semantic_search/tracked_objects \
      /semantic_search/localization_diagnostics \
      /semantic_search/motion_intent \
      /safety/state \
      /follow/cmd_vel_planned \
      /follow/cmd_vel_avoiding \
      /follow/cmd_vel_safe

Stop with Ctrl+C and wait for ros2 bag record to exit before checksumming.
Never commit db3, db3-wal, db3-shm, or tegrastats logs.

Terminal 2:

    tegrastats --interval 1000 \
      --logfile /tmp/semantic_search_field_001_tegrastats.log

Stop telemetry after recording:

    tegrastats --stop

## Create the Field Manifest

Before starting the manifest workflow, change to the workspace root. Keep this
working directory for create-field, add-query, add-object, add-trial, and
add-annotations so every workspace-relative path below is correct regardless of
the shell's initial directory:

    cd "$HOME/track_robot_ws"

Set all four variables to the immutable ID or checksum of the calibration
artifact actually used. The guard aborts if any value is missing:

    : "${CAMERA_INTRINSICS_ID:?set verified camera intrinsics ID}"
    : "${CAMERA_LIDAR_EXTRINSICS_ID:?set verified camera-LiDAR ID}"
    : "${LIDAR_IMU_EXTRINSICS_ID:?set verified LiDAR-IMU ID}"
    : "${LOCALIZATION_CONFIG_ID:?set verified localization config ID}"
    ros2 run track_robot_semantic_search semantic_search_manifest create-field \
      ~/track_robot_ws/rosbags/semantic_search/recordings/field_001 \
      ~/track_robot_ws/artifacts/semantic_search/manifests/field_001.json \
      --dataset-id field_001 \
      --workspace-root ~/track_robot_ws \
      --split validation \
      --site-id site_001 \
      --session-id session_001 \
      --lighting daylight \
      --surface outdoor_path \
      --weather dry \
      --camera-intrinsics-id "$CAMERA_INTRINSICS_ID" \
      --camera-lidar-extrinsics-id "$CAMERA_LIDAR_EXTRINSICS_ID" \
      --lidar-imu-extrinsics-id "$LIDAR_IMU_EXTRINSICS_ID" \
      --localization-config-id "$LOCALIZATION_CONFIG_ID"

Run this command only after the recorder has exited: it reads the closed bag
metadata and computes the immutable bag checksum. Do not add --world-pose or
--active-motion unless the corresponding topics, calibration, and
controlled-test authority are present. The validator rejects
unknown/unverified calibration IDs for field manifests.

## Add Query Events to the Manifest

Use the ROS source timestamp, not wall-clock time:

    : "${QUERY_STAMP_NS:?set the exact ROS event stamp in nanoseconds}"
    ros2 run track_robot_semantic_search semantic_search_manifest add-query \
      artifacts/semantic_search/manifests/field_001.json \
      --query-id 1 \
      --stamp-ns "$QUERY_STAMP_NS" \
      --text "fallen branch blocking the path" \
      --language en \
      --client-request-id field-001

Set QUERY_STAMP_NS from the operator/event log for this run. The guard prevents
a missing value from silently creating an invalid event.

## Register Objects, Trials, and Annotations

Declare physical-object grouping before a trial so that site/date/object splits
cannot be inferred from adjacent frames:

    : "${ACQUISITION_DATE:?set the recording date as YYYY-MM-DD}"
    ros2 run track_robot_semantic_search semantic_search_manifest add-object \
      artifacts/semantic_search/manifests/field_001.json \
      --object-id branch-001-field-001 \
      --physical-object-id branch-001 \
      --label "fallen branch" \
      --site-id site_001 \
      --acquisition-date "$ACQUISITION_DATE" \
      --source robot \
      --provenance human-labelled

Set the exact trial bounds from the same operator/event log used for the query
event, then register the positive passive trial. TRIAL_START_NS and
TRIAL_END_NS must be nanoseconds from the same ROS source clock and timebase as
the bag and QUERY_STAMP_NS; never use wall-clock time or another timebase:

    : "${TRIAL_START_NS:?set the exact trial start stamp}"
    : "${TRIAL_END_NS:?set the exact trial end stamp}"
    : "${NOMINAL_DISTANCE_M:?set the measured nominal distance in metres}"
    ros2 run track_robot_semantic_search semantic_search_manifest add-trial \
      artifacts/semantic_search/manifests/field_001.json \
      --trial-id trial-001 \
      --query-id 1 \
      --target-object-id branch-001-field-001 \
      --positive \
      --start-stamp-ns "$TRIAL_START_NS" \
      --end-stamp-ns "$TRIAL_END_NS" \
      --nominal-distance-m "$NOMINAL_DISTANCE_M" \
      --observation-stage passive \
      --site-id site_001 \
      --session-id session_001

After the labelling tool writes one JSON object per line using
annotation.schema.json, validate every record and register its immutable hash:

    ros2 run track_robot_semantic_search semantic_search_manifest add-annotations \
      artifacts/semantic_search/manifests/field_001.json \
      artifacts/semantic_search/annotations/field_001.jsonl \
      --workspace-root ~/track_robot_ws

The command rejects undeclared query, object, or trial IDs, records outside the
trial interval, invalid geometry, duplicate annotation paths, and malformed
JSONL.

## Legacy Replay

The pinned Foxy rosbag2 0.3.11 player cannot publish a replay clock. Run this
legacy evaluation on wall time and scale the evaluator duration inversely with
the replay rate so each report targets exactly 45.0 seconds of source time.
Formal evidence uses report schema `1.1.0`, the
`foxy_wall_time_scaled` timing policy, and `arrival_monotonic` freshness. Each
required topic must contain at least two messages and cover at least `0.90` of
the target source window (40.5 seconds).

Terminal 1:

    source /opt/ros/foxy/setup.bash
    source ~/track_robot_ws/install/setup.bash
    export ROS_DOMAIN_ID=20
    ros2 launch track_robot_semantic_search semantic_search_phase0.launch.py \
      use_sim_time:=false \
      start_evaluator:=true \
      replay_rate:=1.0 \
      timing_policy:=foxy_wall_time_scaled \
      freshness_time_base:=arrival_monotonic \
      manifest_path:=$HOME/track_robot_ws/artifacts/semantic_search/manifests/legacy/human_tracking_lidar_20260706_150711.json \
      output_path:=/tmp/semantic_search_rate_10.json \
      duration_sec:=45.0

Terminal 2:

    source /opt/ros/foxy/setup.bash
    source ~/track_robot_ws/install/setup.bash
    export ROS_DOMAIN_ID=20
    ros2 bag play \
      ~/track_robot_ws/rosbags/human_tracking/recordings/human_tracking_lidar_20260706_150711 \
      --rate 1.0

In a third terminal on ROS_DOMAIN_ID 20, inspect the passive localization
diagnostics without starting any motion-capable node:

    source /opt/ros/foxy/setup.bash
    source ~/track_robot_ws/install/setup.bash
    export ROS_DOMAIN_ID=20
    ros2 topic echo /semantic_search/localization_diagnostics

Use these exact rate and wall-duration pairs:

- `replay_rate:=0.5`, `--rate 0.5`, `duration_sec:=90.0`;
- `replay_rate:=1.0`, `--rate 1.0`, `duration_sec:=45.0`;
- `replay_rate:=2.0`, `--rate 2.0`, `duration_sec:=22.5`.

Change output names to rate_05, rate_10, and rate_20. Source rates and
synchronization use message header stamps; receive rates use evaluator arrival
times and therefore change with replay rate. Start the launch and bag from
fresh processes for every rate, and do not use `--loop`: timestamp rollback is
an epoch boundary and invalidates a single-run comparison.

Reports produced this way are comparable only within this pinned Foxy
wall-time policy. Do not mix them with reports from a future player that
publishes a native replay clock.

Complete each rate in this order:

1. Wait for the evaluator's `wrote` log, then confirm the atomically replaced
   report is present and valid JSON. Set `REPORT_PATH` to that rate's output:

       REPORT_PATH=/tmp/semantic_search_rate_10.json
       test -s "$REPORT_PATH" && python3 -m json.tool "$REPORT_PATH" >/dev/null

2. In Terminal 2, stop that rate's `ros2 bag play` with Ctrl+C if it is still
   running, or wait if it is already finishing. Wait for the process to exit.
3. Stop and wait for the Terminal 3 diagnostics echo if it was started.
4. Stop and wait for that rate's tegrastats process if telemetry was started.
5. Press Ctrl+C in Terminal 1 and wait for both localization and evaluator
   processes to exit.
6. In a sourced shell on ROS_DOMAIN_ID 20, run these read-only checks:

       BAG_PATH="$HOME/track_robot_ws/rosbags/human_tracking/recordings/human_tracking_lidar_20260706_150711"
       ps -eo pid,ppid,stat,cmd | \
         awk -v needle="ros2 bag play ${BAG_PATH}" \
           'NR > 1 && $4 != "awk" && index($0, needle)'
       ros2 node list | \
         grep -E '(^|/)(semantic_search_localization_health|semantic_search_evaluator)$'

Both checks must print no matches. Stop only the processes started for the
current rate; never use `pkill` or `killall`, and never signal another user's
ROS processes. Do not start the next rate until the bag player and both
semantic nodes are absent. After all three clean runs, confirm all three report
files exist before comparing them:

    ros2 run track_robot_semantic_search semantic_search_compare_reports \
      --manifest "$HOME/track_robot_ws/artifacts/semantic_search/manifests/legacy/human_tracking_lidar_20260706_150711.json" \
      /tmp/semantic_search_rate_05.json \
      /tmp/semantic_search_rate_10.json \
      /tmp/semantic_search_rate_20.json

The comparator accepts exactly these three unique rate/duration reports. It
loads the supplied manifest, validates every report against its checksum and
capabilities, then recomputes all five hard gates. It exits zero only when the
stored gates and `passed` value equal recomputation, all gates pass, and
dataset, software revision, configuration, model exports, coverage, and source
provenance agree. Malformed reports, forward-permission violations, or any
failed gate produce a nonzero exit and a `failures` list. Treat any nonzero
comparison as a failed replay comparison, not as a partial pass.

## Expected Legacy Result

- camera and LiDAR topics are present;
- P95 nearest image/cloud offset is at most 0.08 second;
- localisation mode is OBSERVATION_ONLY only;
- no forward-permission violation occurs;
- missing IMU/local/world pose remain declared capability gaps;
- the legacy baseline proves contracts, replay mechanics, and diagnostics
  only; it does not prove semantic perception, 3D object memory, language
  grounding, motion safety, or active-search safety.
