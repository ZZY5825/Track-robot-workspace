import copy
import hashlib
import json
import math
import os
import re
import sqlite3
import stat
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
from typing import Any, Dict, List, Mapping, Tuple

import yaml


SCHEMA_VERSION = '1.0.0'
SPLITS = {'train', 'validation', 'test', 'extension', 'legacy_replay_only'}
CAPABILITY_KEYS = {
    'camera', 'lidar', 'imu', 'local_pose', 'world_pose',
    'query_events', 'annotations', 'active_motion',
}
PHASE2_SCENARIOS = {
    'static_multi_view',
    'similar_static_objects',
    'moving_human_crossing',
    'camera_occlusion',
    'camera_fov_exit_lidar_visible',
    'both_sensors_exit_reentry',
    'lidar_cluster_split',
    'lidar_cluster_merge',
    'camera_false_positive',
    'lidar_false_cluster',
    'robot_rotation_translation',
    'task_change_without_memory_clear',
}


class ManifestError(ValueError):
    """Raised when a dataset manifest violates the Phase 0 contract."""


def _require_mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ManifestError('{} must be an object'.format(name))
    return value


def _require_keys(value: Mapping[str, Any], keys, name: str) -> None:
    missing = sorted(set(keys) - set(value))
    if missing:
        raise ManifestError('{} missing {}'.format(name, ', '.join(missing)))


def _is_integer(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _is_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float)) and
        not isinstance(value, bool) and
        math.isfinite(value))


def _relative_path(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise ManifestError('{} must be a non-empty string'.format(name))
    path = PurePosixPath(value)
    if (path.is_absolute() or '..' in path.parts or '\\' in value or
            re.match(r'^[A-Za-z]:', value)):
        raise ManifestError('{} must be a safe relative path'.format(name))
    return value


def _inventory_identity(
        path: Path) -> Tuple[bool, int, int, int, int, int]:
    metadata = path.lstat()
    return (
        path.is_symlink(), int(metadata.st_mode), int(metadata.st_dev),
        int(metadata.st_ino), int(metadata.st_size),
        int(metadata.st_mtime_ns))


def _directory_inventory(
        root: Path) -> Dict[str, Tuple[bool, int, int, int, int, int]]:
    return {
        item.relative_to(root).as_posix(): _inventory_identity(item)
        for item in sorted(root.rglob('*'))
    }


def _sha256_verified_files(path: Path, relative_paths: List[str]) -> str:
    digest = hashlib.sha256()
    for relative in sorted(relative_paths):
        encoded = relative.encode('utf-8')
        digest.update(len(encoded).to_bytes(8, 'big'))
        digest.update(encoded)
        file_digest = hashlib.sha256()
        file_size = 0
        with (path / Path(relative)).open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                file_size += len(chunk)
                file_digest.update(chunk)
        digest.update(file_size.to_bytes(8, 'big'))
        digest.update(file_digest.digest())
    return digest.hexdigest()


def _metadata_nonnegative_integer(value: Any, name: str) -> int:
    if not _is_integer(value) or value < 0:
        raise ManifestError(
            '{} must be a non-negative integer'.format(name))
    return value


def _metadata_nonempty_string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise ManifestError('{} must be a non-empty string'.format(name))
    return value


def _check_sqlite(path: Path) -> None:
    uri = path.resolve().as_uri() + '?mode=ro&immutable=1'
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


def read_closed_rosbag(
        bag_dir: Path) -> Tuple[Mapping[str, Any], str]:
    bag_dir = Path(bag_dir)
    metadata_path = bag_dir / 'metadata.yaml'
    if bag_dir.is_symlink() or metadata_path.is_symlink():
        raise ManifestError('closed rosbag cannot contain symlinks')
    if not bag_dir.is_dir() or not metadata_path.is_file():
        raise ManifestError('closed rosbag requires metadata.yaml')
    try:
        before = _directory_inventory(bag_dir)
        metadata_bytes = metadata_path.read_bytes()
        document = _require_mapping(
            yaml.safe_load(metadata_bytes.decode('utf-8')),
            'rosbag metadata document')
        metadata = _require_mapping(
            document['rosbag2_bagfile_information'],
            'rosbag2_bagfile_information')
        storage_id = _metadata_nonempty_string(
            metadata['storage_identifier'], 'storage_identifier')
        if storage_id != 'sqlite3':
            raise ManifestError('closed rosbag storage must be sqlite3')
        relative_values = metadata['relative_file_paths']
        if not isinstance(relative_values, list) or not relative_values:
            raise ManifestError(
                'relative_file_paths must be a non-empty array')
        relative_paths = [
            _relative_path(
                value, 'relative_file_paths[{}]'.format(index))
            for index, value in enumerate(relative_values)
        ]
        if len(set(relative_paths)) != len(relative_paths):
            raise ManifestError('duplicate relative_file_paths')
        if any(not relative.endswith('.db3') for relative in relative_paths):
            raise ManifestError('storage path must end with .db3')

        starting_time = _require_mapping(
            metadata['starting_time'], 'starting_time')
        _metadata_nonnegative_integer(
            starting_time['nanoseconds_since_epoch'],
            'starting_time.nanoseconds_since_epoch')
        duration = _require_mapping(metadata['duration'], 'duration')
        _metadata_nonnegative_integer(
            duration['nanoseconds'], 'duration.nanoseconds')
        topic_items = metadata.get('topics_with_message_count', [])
        if not isinstance(topic_items, list):
            raise ManifestError(
                'topics_with_message_count must be an array')
        for index, item in enumerate(topic_items):
            item = _require_mapping(
                item, 'topics_with_message_count[{}]'.format(index))
            topic = _require_mapping(
                item['topic_metadata'],
                'topics_with_message_count[{}].topic_metadata'.format(
                    index))
            _metadata_nonempty_string(
                topic['name'], 'topic_metadata.name')
            _metadata_nonempty_string(
                topic['type'], 'topic_metadata.type')
            _metadata_nonnegative_integer(
                item['message_count'], 'message_count')

        if any(identity[0] for identity in before.values()):
            raise ManifestError('closed rosbag cannot contain symlinks')
        expected_files = {'metadata.yaml'} | set(relative_paths)
        expected_directories = {
            parent.as_posix()
            for relative in relative_paths
            for parent in PurePosixPath(relative).parents
            if parent != PurePosixPath('.')
        }
        unexpected = sorted(
            relative for relative, identity in before.items()
            if not (
                stat.S_ISREG(identity[1]) and
                relative in expected_files) and
            not (
                stat.S_ISDIR(identity[1]) and
                relative in expected_directories)
        )
        if unexpected:
            raise ManifestError(
                'closed rosbag contains unlisted entries: {}'.format(
                    ', '.join(unexpected)))
        for relative in relative_paths:
            identity = before.get(relative)
            if identity is None or not stat.S_ISREG(identity[1]):
                raise ManifestError(
                    'closed rosbag storage must be a regular '
                    'non-symlink file')

        for relative in relative_paths:
            _check_sqlite(bag_dir / Path(relative))
        if _directory_inventory(bag_dir) != before:
            raise ManifestError(
                'closed rosbag changed while checking SQLite')
        verified_files = ['metadata.yaml'] + relative_paths
        digest = _sha256_verified_files(bag_dir, verified_files)
        if _directory_inventory(bag_dir) != before:
            raise ManifestError('closed rosbag changed while hashing')
    except ManifestError:
        raise
    except (
            OSError, UnicodeError, yaml.YAMLError, KeyError, TypeError,
            AttributeError, ValueError, OverflowError) as error:
        raise ManifestError('invalid rosbag metadata: {}'.format(error)) \
            from error
    return metadata, digest


def sha256_tree(path: Path) -> str:
    path = Path(path)
    relative_paths = [
        item.relative_to(path).as_posix()
        for item in path.rglob('*')
        if item.is_file() and item.name not in ('.DS_Store',)
    ]
    return _sha256_verified_files(path, relative_paths)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def workspace_relative_path(path: Path, workspace_root: Path) -> str:
    path = Path(path).resolve()
    workspace_root = Path(workspace_root).resolve()
    try:
        relative = path.relative_to(workspace_root)
    except ValueError as error:
        raise ManifestError(
            'path must be inside workspace_root') from error
    return _relative_path(relative.as_posix(), 'bag.relative_path')


def validate_manifest(payload: Mapping[str, Any]) -> None:
    payload = _require_mapping(payload, 'manifest')
    manifest_keys = {
        'schema_version', 'dataset_id', 'split', 'bag', 'capabilities',
        'calibration', 'environment', 'queries', 'annotation_files',
        'objects', 'trials', 'provenance',
    }
    _require_keys(payload, manifest_keys, 'manifest')
    if not set(payload) <= manifest_keys | {'phase2'}:
        raise ManifestError('manifest contains unknown fields')
    if payload['schema_version'] != SCHEMA_VERSION:
        raise ManifestError('unsupported schema_version')
    if not isinstance(payload['dataset_id'], str) or not payload['dataset_id']:
        raise ManifestError('dataset_id must be non-empty')
    if payload['split'] not in SPLITS:
        raise ManifestError('unsupported split')

    bag = _require_mapping(payload['bag'], 'bag')
    _require_keys(
        bag,
        {
            'relative_path', 'sha256', 'storage_id', 'start_time_ns',
            'duration_ns', 'topics',
        },
        'bag',
    )
    if set(bag) != {
            'relative_path', 'sha256', 'storage_id', 'start_time_ns',
            'duration_ns', 'topics'}:
        raise ManifestError('bag contains unknown fields')
    _relative_path(bag['relative_path'], 'bag.relative_path')
    checksum = bag['sha256']
    if (not isinstance(checksum, str) or len(checksum) != 64 or
            any(char not in '0123456789abcdef' for char in checksum)):
        raise ManifestError('bag.sha256 must be 64 lowercase hex characters')
    for key in ('start_time_ns', 'duration_ns'):
        if not _is_integer(bag[key]) or bag[key] < 0:
            raise ManifestError('bag.{} must be a non-negative integer'.format(key))
    if not isinstance(bag['topics'], list):
        raise ManifestError('bag.topics must be an array')
    if not isinstance(bag['storage_id'], str) or not bag['storage_id']:
        raise ManifestError('bag.storage_id must be non-empty')
    topic_names = set()
    topic_types = {}
    for topic in bag['topics']:
        topic = _require_mapping(topic, 'bag topic')
        _require_keys(topic, {'name', 'type', 'count'}, 'bag topic')
        if set(topic) != {'name', 'type', 'count'}:
            raise ManifestError('bag topic contains unknown fields')
        if not isinstance(topic['name'], str) or not topic['name']:
            raise ManifestError('topic name must be non-empty')
        if not isinstance(topic['type'], str) or not topic['type']:
            raise ManifestError('topic type must be non-empty')
        if topic['name'] in topic_names:
            raise ManifestError('duplicate topic {}'.format(topic['name']))
        topic_names.add(topic['name'])
        topic_types[topic['name']] = topic['type']
        if not _is_integer(topic['count']) or topic['count'] < 0:
            raise ManifestError('topic count must be non-negative')

    capabilities = _require_mapping(payload['capabilities'], 'capabilities')
    _require_keys(capabilities, CAPABILITY_KEYS, 'capabilities')
    unknown_capabilities = sorted(set(capabilities) - CAPABILITY_KEYS)
    if unknown_capabilities:
        raise ManifestError(
            'unknown capabilities {}'.format(
                ', '.join(unknown_capabilities)))
    if any(not isinstance(capabilities[key], bool) for key in CAPABILITY_KEYS):
        raise ManifestError('capabilities must be boolean')
    if capabilities['world_pose'] and not (
            capabilities['local_pose'] and capabilities['imu']):
        raise ManifestError('world_pose requires local_pose and imu')
    if capabilities['active_motion'] and not (
            capabilities['imu'] and capabilities['local_pose']):
        raise ManifestError('active_motion requires imu and local_pose')
    sensed_capabilities = {
        'camera': any(
            value == 'sensor_msgs/msg/Image'
            for value in topic_types.values()),
        'lidar': any(
            value == 'sensor_msgs/msg/PointCloud2'
            for value in topic_types.values()),
        'imu': any(
            value == 'sensor_msgs/msg/Imu'
            for value in topic_types.values()),
        'local_pose': topic_types.get('/odom') == 'nav_msgs/msg/Odometry',
    }
    for name, present in sensed_capabilities.items():
        if capabilities[name] != present:
            raise ManifestError(
                '{} capability contradicts bag topics'.format(name))
    if capabilities['world_pose'] and (
            topic_types.get('/localization/odometry') !=
            'nav_msgs/msg/Odometry'):
        raise ManifestError(
            'world_pose capability requires localization odometry evidence')
    active_evidence = {
        '/semantic_search/motion_intent':
        'track_robot_interfaces/msg/SearchMotionIntent',
        '/safety/state': 'track_robot_interfaces/msg/SafetyState',
        '/follow/cmd_vel_planned': 'geometry_msgs/msg/Twist',
        '/follow/cmd_vel_safe': 'geometry_msgs/msg/Twist',
    }
    if capabilities['active_motion'] and any(
            topic_types.get(name) != message_type
            for name, message_type in active_evidence.items()):
        raise ManifestError(
            'active_motion requires intent and safety-chain evidence topics')

    calibration_keys = {
        'camera_intrinsics_id',
        'camera_lidar_extrinsics_id',
        'lidar_imu_extrinsics_id',
        'localization_config_id',
    }
    calibration = _require_mapping(
        payload['calibration'], 'calibration')
    _require_keys(calibration, calibration_keys, 'calibration')
    if set(calibration) != calibration_keys:
        raise ManifestError('calibration contains unknown fields')
    if any(
            not isinstance(calibration[key], str) or
            not calibration[key].strip()
            for key in calibration_keys):
        raise ManifestError('calibration IDs must be non-empty strings')
    if payload['split'] != 'legacy_replay_only' and any(
            calibration[key].strip().lower() in {
                'unknown', 'none', 'unverified_legacy_tf'}
            for key in calibration_keys):
        raise ManifestError(
            'field manifests require verified calibration IDs')

    environment_keys = {
        'site_id', 'session_id', 'lighting', 'surface', 'weather'}
    environment = _require_mapping(
        payload['environment'], 'environment')
    _require_keys(environment, environment_keys, 'environment')
    if set(environment) != environment_keys:
        raise ManifestError('environment contains unknown fields')
    if any(
            not isinstance(environment[key], str) or
            not environment[key].strip()
            for key in environment_keys):
        raise ManifestError(
            'environment values must be non-empty strings')
    if payload['split'] != 'legacy_replay_only' and any(
            environment[key].strip().lower() in {
                'unknown', 'legacy_unknown'}
            for key in environment_keys):
        raise ManifestError(
            'field manifests require known environment values')

    for collection in ('queries', 'annotation_files', 'objects', 'trials'):
        if not isinstance(payload[collection], list):
            raise ManifestError('{} must be an array'.format(collection))
    query_ids = set()
    for query in payload['queries']:
        query = _require_mapping(query, 'query event')
        query_keys = {
            'query_id', 'stamp_ns', 'text', 'language', 'client_request_id'}
        _require_keys(query, query_keys, 'query event')
        if set(query) != query_keys:
            raise ManifestError('query event contains unknown fields')
        if not _is_integer(query['query_id']) or query['query_id'] <= 0:
            raise ManifestError('query_id must be positive')
        if query['query_id'] in query_ids:
            raise ManifestError('duplicate query_id')
        query_ids.add(query['query_id'])
        if not _is_integer(query['stamp_ns']) or query['stamp_ns'] < 0:
            raise ManifestError('query stamp_ns must be non-negative')
        if not (
                bag['start_time_ns'] <= query['stamp_ns'] <=
                bag['start_time_ns'] + bag['duration_ns']):
            raise ManifestError('query stamp_ns is outside the bag interval')
        if not isinstance(query['text'], str) or not query['text'].strip():
            raise ManifestError('query text must be non-empty')
        if not isinstance(query['language'], str) or not \
                query['language'].strip():
            raise ManifestError('query language must be non-empty')
        if not isinstance(query['client_request_id'], str):
            raise ManifestError('client_request_id must be a string')
    if bool(payload['queries']) != capabilities['query_events']:
        raise ManifestError('query_events capability must match queries')

    annotation_paths = set()
    annotation_keys = {
        'relative_path', 'sha256', 'format', 'schema_version'}
    for annotation in payload['annotation_files']:
        annotation = _require_mapping(annotation, 'annotation file')
        _require_keys(annotation, annotation_keys, 'annotation file')
        if set(annotation) != annotation_keys:
            raise ManifestError('annotation file contains unknown fields')
        path = _relative_path(
            annotation['relative_path'], 'annotation relative_path')
        if path in annotation_paths:
            raise ManifestError('duplicate annotation relative_path')
        annotation_paths.add(path)
        if (not isinstance(annotation['sha256'], str) or
                not re.fullmatch(r'[0-9a-f]{64}', annotation['sha256'])):
            raise ManifestError('annotation sha256 must be lowercase SHA-256')
        if annotation['format'] != 'jsonl':
            raise ManifestError('annotation format must be jsonl')
        if annotation['schema_version'] != SCHEMA_VERSION:
            raise ManifestError('annotation schema_version is unsupported')
    if bool(payload['annotation_files']) != capabilities['annotations']:
        raise ManifestError(
            'annotations capability must match annotation_files')

    object_ids = set()
    object_keys = {
        'object_id', 'physical_object_id', 'labels', 'site_id',
        'acquisition_date', 'source', 'provenance'}
    for item in payload['objects']:
        item = _require_mapping(item, 'object')
        _require_keys(item, object_keys, 'object')
        if set(item) != object_keys:
            raise ManifestError('object contains unknown fields')
        for key in (
                'object_id', 'physical_object_id', 'site_id',
                'acquisition_date', 'provenance'):
            if not isinstance(item[key], str) or not item[key].strip():
                raise ManifestError('object {} must be non-empty'.format(key))
        if item['object_id'] in object_ids:
            raise ManifestError('duplicate object_id')
        object_ids.add(item['object_id'])
        try:
            datetime.strptime(item['acquisition_date'], '%Y-%m-%d')
        except ValueError as error:
            raise ManifestError(
                'object acquisition_date must be YYYY-MM-DD') from error
        if not isinstance(item['labels'], list) or not item['labels'] or any(
                not isinstance(label, str) or not label.strip()
                for label in item['labels']):
            raise ManifestError('object labels must be non-empty strings')
        if item['source'] not in {'robot', 'public', 'synthetic'}:
            raise ManifestError('object source is unsupported')

    trial_ids = set()
    trial_keys = {
        'trial_id', 'query_id', 'target_object_id', 'is_positive',
        'start_stamp_ns', 'end_stamp_ns', 'nominal_distance_m',
        'observation_stage', 'site_id', 'session_id'}
    for trial in payload['trials']:
        trial = _require_mapping(trial, 'trial')
        _require_keys(trial, trial_keys, 'trial')
        if set(trial) != trial_keys:
            raise ManifestError('trial contains unknown fields')
        if not isinstance(trial['trial_id'], str) or not trial['trial_id']:
            raise ManifestError('trial_id must be non-empty')
        if trial['trial_id'] in trial_ids:
            raise ManifestError('duplicate trial_id')
        trial_ids.add(trial['trial_id'])
        if not _is_integer(trial['query_id']) or trial['query_id'] <= 0:
            raise ManifestError('trial query_id must be positive')
        if trial['query_id'] not in query_ids:
            raise ManifestError('trial query_id is not declared')
        if not isinstance(trial['is_positive'], bool):
            raise ManifestError('trial is_positive must be boolean')
        target = trial['target_object_id']
        if not isinstance(target, str):
            raise ManifestError('trial target_object_id must be a string')
        if trial['is_positive'] and target not in object_ids:
            raise ManifestError('positive trial target is not declared')
        if not trial['is_positive'] and target != '':
            raise ManifestError('negative trial target_object_id must be empty')
        start = trial['start_stamp_ns']
        end = trial['end_stamp_ns']
        if not _is_integer(start) or not _is_integer(end) or not (
                bag['start_time_ns'] <= start <= end <=
                bag['start_time_ns'] + bag['duration_ns']):
            raise ManifestError('trial interval is outside the bag')
        distance = trial['nominal_distance_m']
        if not _is_number(distance) or distance < 0.0:
            raise ManifestError('trial nominal_distance_m is invalid')
        if trial['observation_stage'] not in {
                'passive', 'pre_rotation', 'post_rotation'}:
            raise ManifestError('trial observation_stage is unsupported')
        for key in ('site_id', 'session_id'):
            if not isinstance(trial[key], str) or not trial[key].strip():
                raise ManifestError('trial {} must be non-empty'.format(key))

    provenance = _require_mapping(payload['provenance'], 'provenance')
    _require_keys(
        provenance, {'created_at', 'created_by', 'notes'}, 'provenance')
    if set(provenance) != {'created_at', 'created_by', 'notes'}:
        raise ManifestError('provenance contains unknown fields')
    if not isinstance(provenance['created_at'], str) or not \
            provenance['created_at'].strip():
        raise ManifestError('provenance.created_at must be non-empty')
    if not isinstance(provenance['created_by'], str) or not \
            provenance['created_by'].strip():
        raise ManifestError('provenance.created_by must be non-empty')
    if not isinstance(provenance['notes'], str):
        raise ManifestError('provenance.notes must be a string')

    if 'phase2' in payload:
        phase2 = _require_mapping(payload['phase2'], 'phase2')
        phase2_keys = {
            'memory_frame', 'scenario_ids', 'tf_preflight_passed',
            'resource_profile_available', 'human_regression_evidence'}
        _require_keys(phase2, phase2_keys, 'phase2')
        if set(phase2) != phase2_keys:
            raise ManifestError('phase2 contains unknown fields')
        if phase2['memory_frame'] not in {'base_link', 'odom', 'map'}:
            raise ManifestError('phase2 memory_frame is unsupported')
        scenarios = phase2['scenario_ids']
        if (not isinstance(scenarios, list) or
                any(not isinstance(item, str) or
                    item not in PHASE2_SCENARIOS for item in scenarios) or
                len(set(scenarios)) != len(scenarios)):
            raise ManifestError('phase2 scenario_ids are invalid or duplicated')
        for name in (
                'tf_preflight_passed', 'resource_profile_available',
                'human_regression_evidence'):
            if not isinstance(phase2[name], bool):
                raise ManifestError('phase2 {} must be boolean'.format(name))


def load_manifest(path: Path) -> Dict[str, Any]:
    with Path(path).open('r', encoding='utf-8') as stream:
        payload = json.load(stream)
    validate_manifest(payload)
    return payload


def write_json_atomic(path: Path, payload: Mapping[str, Any]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write('\n')
    os.replace(str(temporary), str(path))


def add_query_event(
        payload: Mapping[str, Any], event: Mapping[str, Any]) -> Dict[str, Any]:
    updated = copy.deepcopy(dict(payload))
    updated['queries'].append(dict(event))
    updated['capabilities']['query_events'] = True
    validate_manifest(updated)
    return updated


def add_object(
        payload: Mapping[str, Any],
        object_record: Mapping[str, Any]) -> Dict[str, Any]:
    updated = copy.deepcopy(dict(payload))
    updated['objects'].append(dict(object_record))
    validate_manifest(updated)
    return updated


def add_trial(
        payload: Mapping[str, Any],
        trial_record: Mapping[str, Any]) -> Dict[str, Any]:
    updated = copy.deepcopy(dict(payload))
    updated['trials'].append(dict(trial_record))
    validate_manifest(updated)
    return updated


def validate_annotation_record(
        record: Mapping[str, Any], manifest: Mapping[str, Any]) -> None:
    record = _require_mapping(record, 'annotation record')
    required = {
        'schema_version', 'dataset_id', 'trial_id', 'stamp_ns', 'query_id',
        'object_id', 'visibility', 'label_source', 'confidence'}
    optional = {
        'bbox_xywh', 'mask_path', 'position_base_m', 'extent_m',
        'public_object_key', 'lidar_source_key', 'support_state',
        'position_3d', 'task_relevant', 'reid_trial_id',
        'reactivation_trial_id', 'ignore', 'ignore_reason'}
    _require_keys(record, required, 'annotation record')
    if not set(record) <= required | optional:
        raise ManifestError('annotation record contains unknown fields')
    if record['schema_version'] != SCHEMA_VERSION:
        raise ManifestError('annotation schema_version is unsupported')
    if record['dataset_id'] != manifest['dataset_id']:
        raise ManifestError('annotation dataset_id does not match manifest')
    if not isinstance(record['trial_id'], str) or not record['trial_id']:
        raise ManifestError('annotation trial_id must be non-empty')
    if not _is_integer(record['query_id']) or record['query_id'] <= 0:
        raise ManifestError('annotation query_id must be positive')
    if not isinstance(record['object_id'], str) or not record['object_id']:
        raise ManifestError('annotation object_id must be non-empty')
    trials = {
        item['trial_id']: item for item in manifest['trials']}
    trial = trials.get(record['trial_id'])
    if trial is None:
        raise ManifestError('annotation trial_id is not declared')
    if record['query_id'] != trial['query_id']:
        raise ManifestError('annotation query_id does not match trial')
    if record['object_id'] not in {
            item['object_id'] for item in manifest['objects']}:
        raise ManifestError('annotation object_id is not declared')
    stamp = record['stamp_ns']
    if not _is_integer(stamp) or not (
            trial['start_stamp_ns'] <= stamp <= trial['end_stamp_ns']):
        raise ManifestError('annotation stamp_ns is outside the trial')
    if record['visibility'] not in {
            'visible', 'partial', 'occluded', 'out_of_fov'}:
        raise ManifestError('annotation visibility is unsupported')
    if record['label_source'] not in {
            'human', 'teacher', 'synthetic', 'temporal_pseudo'}:
        raise ManifestError('annotation label_source is unsupported')
    confidence = record['confidence']
    if not _is_number(confidence) or not 0.0 <= confidence <= 1.0:
        raise ManifestError('annotation confidence is invalid')

    def vector(name, length, non_negative=False):
        value = record.get(name)
        if value is None:
            return
        if not isinstance(value, list) or len(value) != length or any(
                not _is_number(item) for item in value):
            raise ManifestError('{} is invalid'.format(name))
        if non_negative and any(item < 0.0 for item in value):
            raise ManifestError('{} must be non-negative'.format(name))

    vector('bbox_xywh', 4)
    if record.get('bbox_xywh') is not None and any(
            value < 0.0 for value in record['bbox_xywh'][2:]):
        raise ManifestError('bbox width/height must be non-negative')
    vector('position_base_m', 3)
    vector('extent_m', 3, non_negative=True)
    mask_path = record.get('mask_path')
    if mask_path is not None:
        _relative_path(mask_path, 'annotation mask_path')

    public_key = record.get('public_object_key')
    if public_key is not None:
        public_key = _require_mapping(public_key, 'public_object_key')
        if set(public_key) != {'memory_epoch_id', 'global_object_id'} or any(
                not _is_integer(public_key[name]) or public_key[name] <= 0
                for name in ('memory_epoch_id', 'global_object_id')):
            raise ManifestError('public_object_key is invalid')
    lidar_key = record.get('lidar_source_key')
    if lidar_key is not None:
        lidar_key = _require_mapping(lidar_key, 'lidar_source_key')
        if (set(lidar_key) != {'source_epoch_id', 'tracklet_id'} or
                not _is_integer(lidar_key['source_epoch_id']) or
                lidar_key['source_epoch_id'] <= 0 or
                not _is_integer(lidar_key['tracklet_id']) or
                lidar_key['tracklet_id'] < 0):
            raise ManifestError('lidar_source_key is invalid')
    if record.get('support_state') not in {
            None, 'camera_lidar', 'camera_only', 'lidar_only',
            'prediction_only', 'none'}:
        raise ManifestError('annotation support_state is unsupported')
    position_3d = record.get('position_3d')
    if position_3d is not None:
        position_3d = _require_mapping(position_3d, 'position_3d')
        frame_id = position_3d.get('frame_id')
        xyz = position_3d.get('xyz_m')
        if (set(position_3d) != {'frame_id', 'xyz_m'} or
                not isinstance(frame_id, str) or not frame_id or
                len(frame_id) > 128 or not isinstance(xyz, list) or
                len(xyz) != 3 or any(not _is_number(value) for value in xyz)):
            raise ManifestError('annotation position_3d is invalid')
    task_relevant = record.get('task_relevant')
    if task_relevant is not None and not isinstance(task_relevant, bool):
        raise ManifestError('annotation task_relevant must be boolean or null')
    for name in ('reid_trial_id', 'reactivation_trial_id'):
        value = record.get(name)
        if value is not None and (not isinstance(value, str) or not value):
            raise ManifestError('{} must be non-empty or null'.format(name))
    ignore = record.get('ignore', False)
    if not isinstance(ignore, bool):
        raise ManifestError('annotation ignore must be boolean')
    ignore_reason = record.get('ignore_reason', '')
    if not isinstance(ignore_reason, str) or len(ignore_reason) > 256:
        raise ManifestError('annotation ignore_reason is invalid')
    if ignore and not ignore_reason:
        raise ManifestError('ignored annotation requires ignore_reason')


def add_annotation_file(
        payload: Mapping[str, Any], annotation_path: Path,
        workspace_root: Path) -> Dict[str, Any]:
    validate_manifest(payload)
    annotation_path = Path(annotation_path)
    if annotation_path.suffix != '.jsonl':
        raise ManifestError('annotation file must use .jsonl')
    record_count = 0
    with annotation_path.open('r', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as error:
                raise ManifestError(
                    'invalid annotation JSON on line {}'.format(
                        line_number)) from error
            validate_annotation_record(record, payload)
            record_count += 1
    if record_count == 0:
        raise ManifestError('annotation file contains no records')
    updated = copy.deepcopy(dict(payload))
    updated['annotation_files'].append({
        'relative_path': workspace_relative_path(
            annotation_path, workspace_root),
        'sha256': sha256_file(annotation_path),
        'format': 'jsonl',
        'schema_version': SCHEMA_VERSION,
    })
    updated['capabilities']['annotations'] = True
    validate_manifest(updated)
    return updated


def build_legacy_manifest(
        bag_dir: Path,
        dataset_id: str,
        workspace_root: Path) -> Dict[str, Any]:
    bag_dir = Path(bag_dir)
    try:
        metadata, bag_digest = read_closed_rosbag(bag_dir)
        topics = []
        for item in metadata.get('topics_with_message_count', []):
            topic = item['topic_metadata']
            topics.append({
                'name': topic['name'],
                'type': topic['type'],
                'count': item['message_count'],
            })
        storage_id = metadata['storage_identifier']
        start_time_ns = metadata[
            'starting_time']['nanoseconds_since_epoch']
        duration_ns = metadata['duration']['nanoseconds']
    except ManifestError:
        raise
    except (
            OSError, UnicodeError, yaml.YAMLError, KeyError, TypeError,
            AttributeError, ValueError) as error:
        raise ManifestError('invalid rosbag metadata: {}'.format(error)) \
            from error
    payload = {
        'schema_version': SCHEMA_VERSION,
        'dataset_id': dataset_id,
        'split': 'legacy_replay_only',
        'bag': {
            'relative_path': workspace_relative_path(
                bag_dir, workspace_root),
            'sha256': bag_digest,
            'storage_id': storage_id,
            'start_time_ns': start_time_ns,
            'duration_ns': duration_ns,
            'topics': sorted(topics, key=lambda value: value['name']),
        },
        'capabilities': {
            'camera': any(item['type'] == 'sensor_msgs/msg/Image' for item in topics),
            'lidar': any(item['type'] == 'sensor_msgs/msg/PointCloud2' for item in topics),
            'imu': any(item['type'] == 'sensor_msgs/msg/Imu' for item in topics),
            'local_pose': any(
                item['name'] == '/odom' and
                item['type'] == 'nav_msgs/msg/Odometry'
                for item in topics),
            'world_pose': False,
            'query_events': False,
            'annotations': False,
            'active_motion': False,
        },
        'calibration': {
            'camera_intrinsics_id': (
                'recorded_camera_info'
                if any(
                    item['type'] == 'sensor_msgs/msg/CameraInfo'
                    for item in topics)
                else 'unknown'),
            'camera_lidar_extrinsics_id': 'unverified_legacy_tf',
            'lidar_imu_extrinsics_id': 'unknown',
            'localization_config_id': 'none',
        },
        'environment': {
            'site_id': 'legacy_unknown',
            'session_id': dataset_id,
            'lighting': 'unknown',
            'surface': 'unknown',
            'weather': 'unknown',
        },
        'queries': [],
        'annotation_files': [],
        'objects': [],
        'trials': [],
        'provenance': {
            'created_at': datetime.now(
                timezone.utc).isoformat().replace('+00:00', 'Z'),
            'created_by': 'semantic_search_manifest',
            'notes': 'Legacy human-tracking bag; observation-only replay evidence.',
        },
    }
    validate_manifest(payload)
    return payload


def build_field_manifest(
        bag_dir: Path,
        dataset_id: str,
        workspace_root: Path,
        split: str,
        environment: Mapping[str, str],
        calibration: Mapping[str, str],
        world_pose: bool,
        active_motion: bool) -> Dict[str, Any]:
    if split not in SPLITS - {'legacy_replay_only'}:
        raise ManifestError('field split must be train/validation/test/extension')
    payload = build_legacy_manifest(
        bag_dir, dataset_id, workspace_root)
    payload['split'] = split
    payload['environment'] = dict(environment)
    payload['calibration'] = dict(calibration)
    forbidden_ids = {'unknown', 'none', 'unverified_legacy_tf'}
    if any(
            str(value).strip().lower() in forbidden_ids
            for value in payload['calibration'].values()):
        raise ManifestError(
            'field manifests require verified calibration IDs')
    topic_names = {
        topic['name'] for topic in payload['bag']['topics']}
    if world_pose and '/localization/odometry' not in topic_names:
        raise ManifestError(
            'world_pose requires /localization/odometry in the bag')
    payload['capabilities']['world_pose'] = bool(world_pose)
    payload['capabilities']['active_motion'] = bool(active_motion)
    payload['provenance'] = {
        'created_at': datetime.now(
            timezone.utc).isoformat().replace('+00:00', 'Z'),
        'created_by': 'semantic_search_manifest create-field',
        'notes': 'Field dataset bundle generated from closed rosbag metadata.',
    }
    validate_manifest(payload)
    return payload
