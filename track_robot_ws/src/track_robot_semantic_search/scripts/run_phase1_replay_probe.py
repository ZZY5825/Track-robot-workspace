#!/usr/bin/env python3

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import platform
from types import SimpleNamespace
import time

import numpy as np
import torch
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

from track_robot_perception.lidar_cluster_baseline_node import (
    LidarClusterBaselineNode,
    cloud_xyz_intensity,
)
from track_robot_semantic_search.benchmarking import (
    available_candidate,
    latency_summary,
    unavailable_candidate,
)
from track_robot_semantic_search.manifest import sha256_file, write_json_atomic
from track_robot_semantic_search.model_adapters import create_aligned_encoder
from track_robot_semantic_search.model_selection import select_candidate
from track_robot_semantic_search.phase1_baselines import (
    BASELINES,
    build_baseline_report,
)
from track_robot_semantic_search.region_scoring import score_regions
from track_robot_semantic_search.replay_probe import (
    decode_bgr_image,
    read_sampled_messages,
)


IMAGE_TOPIC = '/zed/zed_node/left/image_rect_color'
LIDAR_TOPIC = '/rslidar_points'


def synchronize():
    if torch.cuda.is_available():
        torch.cuda.synchronize()


def timed_complete_path(function):
    synchronize()
    start = time.monotonic()
    result = function()
    synchronize()
    return result, (time.monotonic() - start) * 1000.0


def score_map_max(encoding, query):
    image = np.asarray(encoding.embeddings, dtype=np.float32)
    text = np.asarray(query, dtype=np.float32)
    denominator = np.maximum(
        np.linalg.norm(image, axis=2) * np.linalg.norm(text), 1e-12)
    scores = np.sum(image * text.reshape(1, 1, -1), axis=2) / denominator
    return float(np.max(scores[np.asarray(encoding.valid_patch_mask, dtype=bool)]))


def image_provenance(row, image):
    return {
        'index': row['index'],
        'database_timestamp_ns': row['timestamp_ns'],
        'message_sha256': row['message_sha256'],
        'bgr_sha256': hashlib.sha256(image.tobytes()).hexdigest(),
        'width': int(image.shape[1]),
        'height': int(image.shape[0]),
    }


def run_clip(arguments, rows):
    before_allocated = torch.cuda.memory_allocated()
    before_reserved = torch.cuda.memory_reserved()
    load_start = time.monotonic()
    adapter = create_aligned_encoder(
        'openai_clip',
        model_name='ViT-B/32',
        checkpoint_path=str(arguments.clip_checkpoint),
        runtime_path=str(arguments.clip_runtime),
        device='cuda',
        grid_size=arguments.grid_size)
    synchronize()
    model_load_ms = (time.monotonic() - load_start) * 1000.0

    query_start = time.monotonic()
    query = adapter.encode_text(arguments.query)
    synchronize()
    query_init_ms = (time.monotonic() - query_start) * 1000.0

    def process(row):
        image = decode_bgr_image(row['data'])
        encoding = adapter.encode_image_grid(image)
        regions = score_regions(
            encoding.embeddings,
            query,
            encoding.valid_patch_mask,
            encoding.geometry,
            threshold_mode='quantile',
            quantile=0.90,
            min_area=1,
            max_regions=10)
        return image, encoding, regions

    (_, cold_encoding, _), cold_ms = timed_complete_path(
        lambda: process(rows[0]))
    for _ in range(arguments.warmups):
        timed_complete_path(lambda: process(rows[0]))
    torch.cuda.reset_peak_memory_stats()

    results = []
    latencies = []
    for row in rows:
        (image, encoding, regions), elapsed_ms = timed_complete_path(
            lambda row=row: process(row))
        latencies.append(elapsed_ms)
        results.append({
            **image_provenance(row, image),
            'latency_ms': elapsed_ms,
            'encoder_only_ms': float(encoding.inference_ms),
            'score': max(
                (float(region.peak_score) for region in regions),
                default=score_map_max(encoding, query)),
            'region_count': len(regions),
            'regions': [
                {
                    'x': region.x,
                    'y': region.y,
                    'width': region.width,
                    'height': region.height,
                    'score': region.score,
                    'peak_score': region.peak_score,
                }
                for region in regions
            ],
        })
    summary = latency_summary(latencies)
    capacity = 1000.0 / summary['p95_ms']
    peak_allocated = max(
        0.0,
        (torch.cuda.max_memory_allocated() - before_allocated) / 1048576.0)
    peak_reserved = max(
        0.0,
        (torch.cuda.memory_reserved() - before_reserved) / 1048576.0)
    records = [{
        'kind': 'language_camera',
        'score': item['score'],
        'latency_ms': item['latency_ms'],
        'output_rate_hz': capacity,
        'frame_index': item['index'],
        'database_timestamp_ns': item['database_timestamp_ns'],
        'message_sha256': item['message_sha256'],
        'query_text': arguments.query,
        'region_count': item['region_count'],
    } for item in results]
    return {
        'implementation': 'openai_clip',
        'model_name': 'ViT-B/32',
        'encoder_id': adapter.encoder_id,
        'checkpoint_id': adapter.checkpoint_id,
        'checkpoint_sha256': sha256_file(arguments.clip_checkpoint),
        'runtime_path': str(arguments.clip_runtime.resolve()),
        'runtime_revision': arguments.clip_revision,
        'licence': 'MIT (OpenAI CLIP code and released model weights)',
        'redistribution': 'subject to the bundled OpenAI CLIP MIT licence',
        'grid_size': arguments.grid_size,
        'query_text': arguments.query,
        'query_role': 'runtime probe only; not ground truth',
        'model_load_ms': model_load_ms,
        'query_init_ms': query_init_ms,
        'cold_complete_path_ms': cold_ms,
        'cold_encoder_only_ms': float(cold_encoding.inference_ms),
        'complete_path_latency': summary,
        'semantic_output_capacity_hz': capacity,
        'latency_gate_p95_at_most_150_ms': summary['p95_ms'] <= 150.0,
        'rate_gate_at_least_5_hz': capacity >= 5.0,
        'cuda_incremental': {
            'peak_allocated_mb': peak_allocated,
            'reserved_mb': peak_reserved,
            'memory_limit_mb': arguments.memory_limit_mb,
        },
        'results': results,
    }, records


def run_yolo(arguments, rows):
    from ultralytics import YOLO

    synchronize()
    before_allocated = torch.cuda.memory_allocated()
    before_reserved = torch.cuda.memory_reserved()
    load_start = time.monotonic()
    model = YOLO(str(arguments.yolo_weights))
    synchronize()
    model_load_ms = (time.monotonic() - load_start) * 1000.0

    def process(row):
        image = decode_bgr_image(row['data'])
        prediction = model.predict(
            source=image,
            device=0,
            imgsz=640,
            classes=[0],
            verbose=False)[0]
        boxes = prediction.boxes
        confidence = (
            boxes.conf.detach().float().cpu().numpy()
            if boxes is not None else np.empty(0, dtype=np.float32))
        coordinates = (
            boxes.xyxy.detach().float().cpu().numpy()
            if boxes is not None else np.empty((0, 4), dtype=np.float32))
        return image, confidence, coordinates

    (_, _, _), cold_ms = timed_complete_path(lambda: process(rows[0]))
    for _ in range(arguments.warmups):
        timed_complete_path(lambda: process(rows[0]))
    torch.cuda.reset_peak_memory_stats()
    results = []
    latencies = []
    for row in rows:
        (image, confidence, coordinates), elapsed_ms = timed_complete_path(
            lambda row=row: process(row))
        latencies.append(elapsed_ms)
        results.append({
            **image_provenance(row, image),
            'latency_ms': elapsed_ms,
            'score': float(confidence.max()) if confidence.size else 0.0,
            'region_count': int(confidence.size),
            'regions': [
                {
                    'x1': float(box[0]), 'y1': float(box[1]),
                    'x2': float(box[2]), 'y2': float(box[3]),
                    'score': float(score),
                }
                for box, score in zip(coordinates, confidence)
            ],
        })
    summary = latency_summary(latencies)
    capacity = 1000.0 / summary['p95_ms']
    records = [{
        'kind': 'fixed_detector',
        'score': item['score'],
        'latency_ms': item['latency_ms'],
        'output_rate_hz': capacity,
        'frame_index': item['index'],
        'database_timestamp_ns': item['database_timestamp_ns'],
        'message_sha256': item['message_sha256'],
        'region_count': item['region_count'],
    } for item in results]
    return {
        'implementation': 'ultralytics_yolov8_pose_person_class',
        'encoder_id': 'ultralytics:yolov8n-pose:person',
        'checkpoint_id': arguments.yolo_weights.name,
        'checkpoint_sha256': sha256_file(arguments.yolo_weights),
        'licence': 'AGPL-3.0 (Ultralytics runtime/model family)',
        'redistribution': 'review required before product redistribution',
        'model_load_ms': model_load_ms,
        'cold_complete_path_ms': cold_ms,
        'complete_path_latency': summary,
        'semantic_output_capacity_hz': capacity,
        'latency_gate_p95_at_most_150_ms': summary['p95_ms'] <= 150.0,
        'rate_gate_at_least_5_hz': capacity >= 5.0,
        'cuda_incremental': {
            'peak_allocated_mb': max(
                0.0, (torch.cuda.max_memory_allocated() - before_allocated) /
                1048576.0),
            'reserved_mb': max(
                0.0, (torch.cuda.memory_reserved() - before_reserved) /
                1048576.0),
        },
        'results': results,
    }, records


def lidar_configuration():
    return {
        'min_range': 0.5,
        'max_range': 15.0,
        'min_x': -15.0,
        'max_x': 15.0,
        'min_y': -8.0,
        'max_y': 8.0,
        'min_z': -2.0,
        'max_z': 3.0,
        'ground_z_threshold': -0.7,
        'dbscan_eps': 0.35,
        'dbscan_min_samples': 8,
        'min_cluster_points': 20,
        'max_cluster_points': 5000,
        'voxel_size': 0.20,
        'max_points_before_clustering': 5000,
        'min_cluster_height': 0.05,
        'max_cluster_height': 3.0,
        'min_cluster_width': 0.05,
        'max_cluster_width': 4.0,
    }


def run_lidar(rows):
    configuration = lidar_configuration()
    processor = SimpleNamespace(**configuration)

    def process(row):
        cloud = deserialize_message(row['data'], PointCloud2)
        input_points, input_intensity = cloud_xyz_intensity(cloud)
        points, intensity, counts = LidarClusterBaselineNode.filter_points(
            processor, input_points, input_intensity)
        points, intensity = LidarClusterBaselineNode.voxel_downsample(
            processor, points, intensity)
        if points.shape[0] > processor.max_points_before_clustering:
            indices = np.linspace(
                0, points.shape[0] - 1,
                processor.max_points_before_clustering,
                dtype=np.int64)
            points = points[indices]
            intensity = intensity[indices]
        labels = (
            LidarClusterBaselineNode.dbscan(
                points, processor.dbscan_eps, processor.dbscan_min_samples)
            if points.shape[0] else np.empty(0, dtype=np.int32))
        _, clusters = LidarClusterBaselineNode.filter_and_describe_clusters(
            processor, points, labels)
        return input_points, points, counts, clusters

    _, cold_ms = timed_complete_path(lambda: process(rows[0]))
    results = []
    latencies = []
    for row in rows:
        (raw, points, counts, clusters), elapsed_ms = timed_complete_path(
            lambda row=row: process(row))
        latencies.append(elapsed_ms)
        results.append({
            'index': row['index'],
            'database_timestamp_ns': row['timestamp_ns'],
            'message_sha256': row['message_sha256'],
            'latency_ms': elapsed_ms,
            'score': 1.0 if clusters else 0.0,
            'input_points': int(raw.shape[0]),
            'roi_points': counts['roi'],
            'downsampled_points': int(points.shape[0]),
            'cluster_count': len(clusters),
        })
    summary = latency_summary(latencies)
    capacity = 1000.0 / summary['p95_ms']
    records = [{
        'kind': 'lidar_geometry',
        'score': item['score'],
        'latency_ms': item['latency_ms'],
        'output_rate_hz': capacity,
        'frame_index': item['index'],
        'database_timestamp_ns': item['database_timestamp_ns'],
        'message_sha256': item['message_sha256'],
        'cluster_count': item['cluster_count'],
    } for item in results]
    config_bytes = json.dumps(
        configuration, sort_keys=True, separators=(',', ':')).encode()
    return {
        'implementation': 'track_robot_perception.lidar_cluster_baseline',
        'configuration': configuration,
        'configuration_sha256': hashlib.sha256(config_bytes).hexdigest(),
        'score_semantics': 'binary cluster presence; not semantic accuracy',
        'cold_complete_path_ms': cold_ms,
        'complete_path_latency': summary,
        'semantic_output_capacity_hz': capacity,
        'latency_gate_p95_at_most_150_ms': summary['p95_ms'] <= 150.0,
        'rate_gate_at_least_5_hz': capacity >= 5.0,
        'results': results,
    }, records


def write_jsonl_atomic(path, records):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        for record in records:
            stream.write(json.dumps(record, sort_keys=True) + '\n')
    temporary.replace(path)


def parser():
    root = argparse.ArgumentParser(
        description='Run real Phase 1 baselines on deterministic rosbag samples.')
    root.add_argument('--bag-db', type=Path, required=True)
    root.add_argument('--manifest', type=Path, required=True)
    root.add_argument('--clip-checkpoint', type=Path, required=True)
    root.add_argument('--clip-runtime', type=Path, required=True)
    root.add_argument('--clip-revision', required=True)
    root.add_argument('--yolo-weights', type=Path, required=True)
    root.add_argument('--dino-report', type=Path, required=True)
    root.add_argument('--output-dir', type=Path, required=True)
    root.add_argument('--software-revision', required=True)
    root.add_argument('--run-date', default='2026-07-15')
    root.add_argument('--samples', type=int, default=32)
    root.add_argument('--warmups', type=int, default=5)
    root.add_argument('--grid-size', type=int, default=4)
    root.add_argument('--query', default='a person')
    root.add_argument('--memory-limit-mb', type=float, default=1536.0)
    return root


def main(argv=None):
    arguments = parser().parse_args(argv)
    if not torch.cuda.is_available():
        raise SystemExit('CUDA is required for the Jetson Phase 1 replay probe')
    for path in (
            arguments.bag_db, arguments.manifest,
            arguments.clip_checkpoint, arguments.yolo_weights,
            arguments.dino_report):
        if not path.is_file():
            raise SystemExit('required file is missing: {}'.format(path))
    if not arguments.clip_runtime.is_dir():
        raise SystemExit(
            'CLIP runtime directory is missing: {}'.format(
                arguments.clip_runtime))

    image_rows = read_sampled_messages(
        arguments.bag_db, IMAGE_TOPIC, 'sensor_msgs/msg/Image',
        arguments.samples)
    lidar_rows = read_sampled_messages(
        arguments.bag_db, LIDAR_TOPIC, 'sensor_msgs/msg/PointCloud2',
        arguments.samples)

    clip, clip_records = run_clip(arguments, image_rows)
    torch.cuda.empty_cache()
    yolo, yolo_records = run_yolo(arguments, image_rows)
    torch.cuda.empty_cache()
    lidar, lidar_records = run_lidar(lidar_rows)
    observations = yolo_records + lidar_records + clip_records

    output_dir = arguments.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    observations_path = output_dir / (
        'phase1_observations_{}.jsonl'.format(arguments.run_date))
    write_jsonl_atomic(observations_path, observations)
    observation_sha = sha256_file(observations_path)

    model_evidence = {
        'baseline_1_fixed_detector': {
            'available': True,
            'encoder_id': yolo['encoder_id'],
            'checkpoint_id': 'sha256:{}'.format(yolo['checkpoint_sha256']),
            'licence': yolo['licence'],
        },
        'baseline_3_language_camera': {
            'available': True,
            'encoder_id': clip['encoder_id'],
            'checkpoint_id': 'sha256:{}'.format(clip['checkpoint_sha256']),
            'licence': clip['licence'],
        },
    }
    model_evidence_path = output_dir / (
        'phase1_model_evidence_{}.json'.format(arguments.run_date))
    write_json_atomic(model_evidence_path, model_evidence)

    manifest = json.loads(arguments.manifest.read_text(encoding='utf-8'))
    baseline_filename = {
        'baseline_1_fixed_detector': 'phase1_baseline_1_{}.json',
        'baseline_2_lidar_geometry': 'phase1_baseline_2_{}.json',
        'baseline_3_language_camera': 'phase1_baseline_3_{}.json',
    }
    baseline_reports = {}
    for baseline_id, _ in BASELINES:
        report = build_baseline_report(
            baseline_id=baseline_id,
            dataset_id=manifest['dataset_id'],
            manifest_sha256=sha256_file(arguments.manifest),
            manifest_capabilities=manifest['capabilities'],
            records=observations,
            software_revision=arguments.software_revision,
            model_evidence=model_evidence.get(baseline_id))
        report['artifacts']['observations'] = {
            'sha256': observation_sha,
            'sample_policy': '32 evenly spaced messages per source topic',
            'path': observations_path.name,
        }
        write_json_atomic(
            output_dir / baseline_filename[baseline_id].format(
                arguments.run_date),
            report)
        baseline_reports[baseline_id] = report

    candidate = available_candidate(
        'openai_clip_vit_b32',
        p95_latency_ms=clip['complete_path_latency']['p95_ms'],
        peak_memory_mb=clip['cuda_incremental']['reserved_mb'],
        memory_limit_mb=arguments.memory_limit_mb,
        phrase_region_recall=None)
    candidate.update({
        'checkpoint_sha256': clip['checkpoint_sha256'],
        'runtime_revision': clip['runtime_revision'],
        'reason': (
            'runtime, licence, memory and latency pass; accuracy remains '
            'not_evaluated because the legacy bag has no semantic labels'),
    })
    candidates = [
        unavailable_candidate(
            'siglip2_b',
            'No reviewed Python-3.8-compatible runtime/checkpoint is present.'),
        candidate,
    ]
    selection = select_candidate(candidates)
    dino_report = json.loads(
        arguments.dino_report.read_text(encoding='utf-8'))
    model_report = {
        'schema_version': '1.0.0',
        'run_id': 'phase1-model-selection-{}'.format(arguments.run_date),
        'platform': {
            'python': platform.python_version(),
            'pytorch': torch.__version__,
            'device': 'cuda',
            'machine': platform.machine(),
            'jetpack_l4t': 'R35.1',
        },
        'software_revision': arguments.software_revision,
        'dino_runtime': dino_report['dino_runtime'],
        'aligned_runtime': clip,
        'candidates': candidates,
        'selection': asdict(selection),
    }
    write_json_atomic(
        output_dir / 'phase1_model_selection_{}.json'.format(
            arguments.run_date),
        model_report)

    frame_selection = {
        'image': [{key: row[key] for key in (
            'index', 'timestamp_ns', 'message_sha256')} for row in image_rows],
        'lidar': [{key: row[key] for key in (
            'index', 'timestamp_ns', 'message_sha256')} for row in lidar_rows],
    }
    probe = {
        'schema_version': '1.0.0',
        'run_id': 'phase1-real-replay-probe-{}'.format(arguments.run_date),
        'software_revision': arguments.software_revision,
        'bag_database': str(arguments.bag_db.resolve()),
        'bag_database_sha256': sha256_file(arguments.bag_db),
        'manifest_sha256': sha256_file(arguments.manifest),
        'frame_selection': frame_selection,
        'frame_selection_sha256': hashlib.sha256(json.dumps(
            frame_selection, sort_keys=True,
            separators=(',', ':')).encode()).hexdigest(),
        'observations_sha256': observation_sha,
        'accuracy_status': 'not_evaluated',
        'accuracy_reason': 'legacy rosbag has no semantic annotations',
        'release_accuracy_claim': False,
        'baselines': {
            'baseline_1_fixed_detector': yolo,
            'baseline_2_lidar_geometry': lidar,
            'baseline_3_language_camera': clip,
        },
        'baseline_statuses': {
            key: value['status'] for key, value in baseline_reports.items()},
    }
    write_json_atomic(
        output_dir / 'phase1_replay_probe_{}.json'.format(arguments.run_date),
        probe)
    print(json.dumps({
        'selection': asdict(selection),
        'baseline_statuses': probe['baseline_statuses'],
        'clip_p95_ms': clip['complete_path_latency']['p95_ms'],
        'yolo_p95_ms': yolo['complete_path_latency']['p95_ms'],
        'lidar_p95_ms': lidar['complete_path_latency']['p95_ms'],
        'observations_sha256': observation_sha,
    }, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
