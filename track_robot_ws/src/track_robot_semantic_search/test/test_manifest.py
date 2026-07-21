import json
import os
import re
import sqlite3
from pathlib import Path

import pytest
import yaml

import track_robot_semantic_search.manifest as manifest_module
from track_robot_semantic_search.manifest import (
    ManifestError,
    add_annotation_file,
    add_object,
    add_query_event,
    add_trial,
    build_field_manifest,
    build_legacy_manifest,
    load_manifest,
    sha256_tree,
    validate_annotation_record,
    validate_manifest,
    workspace_relative_path,
    write_json_atomic,
)


def write_sqlite_storage(path):
    connection = sqlite3.connect(str(path))
    try:
        connection.execute(
            'CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT, type TEXT)')
        connection.execute(
            'CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER, '
            'timestamp INTEGER, data BLOB)')
        connection.commit()
    finally:
        connection.close()


def write_closed_bag(root, name='bag'):
    bag = root / name
    bag.mkdir(parents=True)
    storage = bag / '{}_0.db3'.format(name)
    write_sqlite_storage(storage)
    metadata = {
        'rosbag2_bagfile_information': {
            'storage_identifier': 'sqlite3',
            'relative_file_paths': [storage.name],
            'starting_time': {'nanoseconds_since_epoch': 12},
            'duration': {'nanoseconds': 34},
            'topics_with_message_count': [],
        },
    }
    (bag / 'metadata.yaml').write_text(
        yaml.safe_dump(metadata), encoding='utf-8')
    return bag, storage


def valid_manifest():
    return {
        'schema_version': '1.0.0',
        'dataset_id': 'unit_test_bag',
        'split': 'legacy_replay_only',
        'bag': {
            'relative_path': 'bags/unit_test_bag',
            'sha256': '0' * 64,
            'storage_id': 'sqlite3',
            'start_time_ns': 1,
            'duration_ns': 10,
            'topics': [
                {'name': '/camera', 'type': 'sensor_msgs/msg/Image', 'count': 2},
            ],
        },
        'capabilities': {
            'camera': True,
            'lidar': False,
            'imu': False,
            'local_pose': False,
            'world_pose': False,
            'query_events': False,
            'annotations': False,
            'active_motion': False,
        },
        'calibration': {
            'camera_intrinsics_id': 'unknown',
            'camera_lidar_extrinsics_id': 'unknown',
            'lidar_imu_extrinsics_id': 'unknown',
            'localization_config_id': 'none',
        },
        'environment': {
            'site_id': 'legacy_unknown',
            'session_id': 'unit',
            'lighting': 'unknown',
            'surface': 'unknown',
            'weather': 'unknown',
        },
        'queries': [],
        'annotation_files': [],
        'objects': [],
        'trials': [],
        'provenance': {
            'created_at': '2026-07-13T00:00:00Z',
            'created_by': 'unit_test',
            'notes': 'legacy replay only',
        },
    }


def test_valid_manifest_round_trip(tmp_path):
    payload = valid_manifest()
    validate_manifest(payload)
    path = tmp_path / 'manifest.json'
    write_json_atomic(path, payload)
    assert load_manifest(path) == payload


def test_phase2_manifest_extension_is_optional_but_strict():
    payload = valid_manifest()
    payload['phase2'] = {
        'memory_frame': 'odom',
        'scenario_ids': [
            'static_multi_view',
            'task_change_without_memory_clear',
        ],
        'tf_preflight_passed': True,
        'resource_profile_available': False,
        'human_regression_evidence': False,
    }
    validate_manifest(payload)

    duplicate = json.loads(json.dumps(payload))
    duplicate['phase2']['scenario_ids'].append('static_multi_view')
    with pytest.raises(ManifestError, match='scenario'):
        validate_manifest(duplicate)

    invalid_frame = json.loads(json.dumps(payload))
    invalid_frame['phase2']['memory_frame'] = 'camera'
    with pytest.raises(ManifestError, match='memory_frame'):
        validate_manifest(invalid_frame)


def test_phase2_annotation_extension_validates_bounded_public_evidence():
    payload = valid_manifest()
    payload['queries'] = [{
        'query_id': 1,
        'stamp_ns': 5,
        'text': 'crate',
        'language': 'en',
        'client_request_id': 'phase2',
    }]
    payload['capabilities']['query_events'] = True
    payload['objects'] = [{
        'object_id': 'object-1',
        'physical_object_id': 'crate-physical',
        'labels': ['crate'],
        'site_id': 'site-a',
        'acquisition_date': '2026-07-16',
        'source': 'robot',
        'provenance': 'human-labelled',
    }]
    payload['trials'] = [{
        'trial_id': 'trial-1',
        'query_id': 1,
        'target_object_id': 'object-1',
        'is_positive': True,
        'start_stamp_ns': 1,
        'end_stamp_ns': 10,
        'nominal_distance_m': 1.0,
        'observation_stage': 'passive',
        'site_id': 'site-a',
        'session_id': 'unit',
    }]
    record = {
        'schema_version': '1.0.0',
        'dataset_id': 'unit_test_bag',
        'trial_id': 'trial-1',
        'stamp_ns': 5,
        'query_id': 1,
        'object_id': 'object-1',
        'visibility': 'visible',
        'label_source': 'human',
        'confidence': 1.0,
        'public_object_key': {
            'memory_epoch_id': 7,
            'global_object_id': 2,
        },
        'lidar_source_key': {'source_epoch_id': 4, 'tracklet_id': 3},
        'support_state': 'camera_lidar',
        'position_3d': {'frame_id': 'odom', 'xyz_m': [1.0, 2.0, 0.0]},
        'task_relevant': True,
        'reid_trial_id': 'reid-1',
        'reactivation_trial_id': None,
        'ignore': False,
        'ignore_reason': '',
    }

    validate_annotation_record(record, payload)

    invalid = dict(record, support_state='magic')
    with pytest.raises(ManifestError, match='support_state'):
        validate_annotation_record(invalid, payload)


@pytest.mark.parametrize(
    'mutate',
    [
        lambda payload: payload.update(schema_version='2.0.0'),
        lambda payload: payload['bag'].update(relative_path='/absolute/path'),
        lambda payload: payload['bag'].update(sha256='bad'),
        lambda payload: payload.update(split='random'),
        lambda payload: payload.update(split='validation'),
        lambda payload: payload['capabilities'].pop('imu'),
        lambda payload: payload['capabilities'].update(annotations=True),
    ],
)
def test_invalid_manifest_is_rejected(mutate):
    payload = valid_manifest()
    mutate(payload)
    with pytest.raises(ManifestError):
        validate_manifest(payload)


def test_query_events_are_positive_unique_and_timestamped():
    payload = valid_manifest()
    event = {
        'query_id': 1,
        'stamp_ns': 5,
        'text': 'fallen branch blocking the path',
        'language': 'en',
        'client_request_id': 'field-001',
    }
    updated = add_query_event(payload, event)
    assert updated['capabilities']['query_events'] is True
    assert updated['queries'] == [event]
    with pytest.raises(ManifestError):
        add_query_event(updated, event)
    outside = dict(event, query_id=2, stamp_ns=100)
    with pytest.raises(ManifestError):
        add_query_event(updated, outside)


def test_annotation_file_is_validated_hashed_and_registered(tmp_path):
    payload = add_query_event(valid_manifest(), {
        'query_id': 1,
        'stamp_ns': 5,
        'text': 'fallen branch blocking the path',
        'language': 'en',
        'client_request_id': 'field-001',
    })
    payload = add_object(payload, {
        'object_id': 'object-1',
        'physical_object_id': 'branch-1',
        'labels': ['fallen branch'],
        'site_id': 'site-a',
        'acquisition_date': '2026-07-13',
        'source': 'robot',
        'provenance': 'human-labelled',
    })
    payload = add_trial(payload, {
        'trial_id': 'trial-1',
        'query_id': 1,
        'target_object_id': 'object-1',
        'is_positive': True,
        'start_stamp_ns': 2,
        'end_stamp_ns': 10,
        'nominal_distance_m': 2.0,
        'observation_stage': 'passive',
        'site_id': 'site-a',
        'session_id': 'unit',
    })
    annotation = tmp_path / 'annotations' / 'trial-1.jsonl'
    annotation.parent.mkdir()
    annotation.write_text(json.dumps({
        'schema_version': '1.0.0',
        'dataset_id': 'unit_test_bag',
        'trial_id': 'trial-1',
        'stamp_ns': 5,
        'query_id': 1,
        'object_id': 'object-1',
        'bbox_xywh': [1.0, 2.0, 3.0, 4.0],
        'mask_path': None,
        'position_base_m': [1.0, 0.0, 0.0],
        'extent_m': [0.5, 0.2, 0.2],
        'visibility': 'visible',
        'label_source': 'human',
        'confidence': 1.0,
    }) + '\n', encoding='utf-8')
    updated = add_annotation_file(payload, annotation, tmp_path)
    assert updated['capabilities']['annotations'] is True
    assert updated['annotation_files'][0]['relative_path'] == (
        'annotations/trial-1.jsonl')
    assert len(updated['annotation_files'][0]['sha256']) == 64


def test_build_legacy_manifest_reads_rosbag_metadata(tmp_path):
    bag, _ = write_closed_bag(tmp_path / 'rosbags', 'legacy_bag')
    metadata_path = bag / 'metadata.yaml'
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    metadata['rosbag2_bagfile_information']['topics_with_message_count'] = [{
        'topic_metadata': {
            'name': '/camera',
            'type': 'sensor_msgs/msg/Image',
        },
        'message_count': 5,
    }]
    metadata_path.write_text(yaml.safe_dump(metadata), encoding='utf-8')
    payload = build_legacy_manifest(bag, 'legacy_bag', tmp_path)
    validate_manifest(payload)
    assert payload['bag']['relative_path'] == 'rosbags/legacy_bag'
    assert payload['bag']['sha256'] == sha256_tree(bag)
    assert payload['capabilities']['imu'] is False
    assert payload['split'] == 'legacy_replay_only'


def test_bag_checksum_includes_sqlite_sidecars(tmp_path):
    bag, storage = write_closed_bag(tmp_path)
    Path(str(storage) + '-wal').write_bytes(b'first')
    Path(str(storage) + '-shm').write_bytes(b'first')
    first = sha256_tree(bag)
    Path(str(storage) + '-wal').write_bytes(b'second')
    Path(str(storage) + '-shm').write_bytes(b'second')
    assert sha256_tree(bag) != first


@pytest.mark.parametrize('suffix', ['-wal', '-shm'])
def test_manifest_builder_rejects_active_sqlite_sidecars(tmp_path, suffix):
    bag, storage = write_closed_bag(tmp_path)
    Path(str(storage) + suffix).write_bytes(b'active')
    with pytest.raises(ManifestError, match='closed rosbag'):
        build_legacy_manifest(bag, 'bag', tmp_path)


def test_manifest_builder_rejects_unlisted_storage(tmp_path):
    bag, _ = write_closed_bag(tmp_path)
    write_sqlite_storage(bag / 'unlisted.db3')
    with pytest.raises(ManifestError, match='unlisted'):
        build_legacy_manifest(bag, 'bag', tmp_path)


def test_manifest_builder_rejects_unlisted_fifo(tmp_path):
    bag, _ = write_closed_bag(tmp_path)
    os.mkfifo(str(bag / 'unlisted.pipe'))
    with pytest.raises(ManifestError, match='unlisted'):
        build_legacy_manifest(bag, 'bag', tmp_path)


def test_manifest_builder_rejects_symlinked_storage(tmp_path):
    bag, storage = write_closed_bag(tmp_path)
    target = tmp_path / 'external.db3'
    storage.rename(target)
    storage.symlink_to(target)
    with pytest.raises(ManifestError, match='symlink'):
        build_legacy_manifest(bag, 'bag', tmp_path)


def test_manifest_builder_rejects_invalid_sqlite(tmp_path):
    bag, storage = write_closed_bag(tmp_path)
    storage.write_bytes(b'not sqlite')
    with pytest.raises(ManifestError, match='SQLite'):
        build_legacy_manifest(bag, 'bag', tmp_path)


@pytest.mark.parametrize(
    'name',
    [
        'literal%25.db3',
        'literal?name.db3',
        'literal#name.db3',
    ],
)
def test_sqlite_check_treats_uri_metacharacters_as_literal(tmp_path, name):
    storage = tmp_path / name
    write_sqlite_storage(storage)
    manifest_module._check_sqlite(storage)


def test_sqlite_check_cannot_decode_percent_path_outside_bag(tmp_path):
    bag = tmp_path / 'bag'
    bag.mkdir()
    (bag / 'nested').mkdir()
    outside = tmp_path / 'outside.db3'
    write_sqlite_storage(outside)
    literal = bag / 'nested%2F..%2F..%2Foutside.db3'
    literal.write_bytes(b'not sqlite')
    with pytest.raises(ManifestError, match='SQLite'):
        manifest_module._check_sqlite(literal)


def test_manifest_builder_rejects_metadata_replaced_after_identity_check(
        tmp_path, monkeypatch):
    bag, _ = write_closed_bag(tmp_path)
    metadata_path = bag / 'metadata.yaml'
    replacement = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    replacement['rosbag2_bagfile_information'][
        'starting_time']['nanoseconds_since_epoch'] = 99
    original = manifest_module._relative_path
    replaced = False

    def replace_during_path_validation(value, name):
        nonlocal replaced
        result = original(value, name)
        if not replaced and name == 'relative_file_paths[0]':
            replacement_path = bag / 'replacement.yaml'
            replacement_path.write_text(
                yaml.safe_dump(replacement), encoding='utf-8')
            replacement_path.replace(metadata_path)
            replaced = True
        return result

    monkeypatch.setattr(
        manifest_module, '_relative_path', replace_during_path_validation)
    with pytest.raises(ManifestError, match='changed'):
        build_legacy_manifest(bag, 'bag', tmp_path)


def test_manifest_builder_rejects_file_changed_during_hash(
        tmp_path, monkeypatch):
    bag, storage = write_closed_bag(tmp_path)
    original = getattr(
        manifest_module, '_sha256_verified_files',
        lambda path, relative_paths: sha256_tree(path))

    def hash_then_change(path, relative_paths):
        digest = original(path, relative_paths)
        with storage.open('ab') as stream:
            stream.write(b'changed')
        return digest

    monkeypatch.setattr(
        manifest_module, '_sha256_verified_files', hash_then_change,
        raising=False)
    with pytest.raises(ManifestError, match='changed while hashing'):
        build_legacy_manifest(bag, 'bag', tmp_path)


def test_manifest_builder_rejects_unlisted_file_added_after_hash(
        tmp_path, monkeypatch):
    bag, _ = write_closed_bag(tmp_path)
    original = getattr(
        manifest_module, '_sha256_verified_files',
        lambda path, relative_paths: sha256_tree(path))

    def hash_then_add(path, relative_paths):
        digest = original(path, relative_paths)
        (bag / 'late-payload.bin').write_bytes(b'late')
        return digest

    monkeypatch.setattr(
        manifest_module, '_sha256_verified_files', hash_then_add,
        raising=False)
    with pytest.raises(ManifestError, match='changed while hashing'):
        build_legacy_manifest(bag, 'bag', tmp_path)


def test_manifest_builder_rejects_storage_deleted_after_hash(
        tmp_path, monkeypatch):
    bag, storage = write_closed_bag(tmp_path)
    original = getattr(
        manifest_module, '_sha256_verified_files',
        lambda path, relative_paths: sha256_tree(path))
    hash_called = False

    def hash_then_delete(path, relative_paths):
        nonlocal hash_called
        hash_called = True
        digest = original(path, relative_paths)
        storage.unlink()
        return digest

    monkeypatch.setattr(
        manifest_module, '_sha256_verified_files', hash_then_delete,
        raising=False)
    with pytest.raises(ManifestError, match='changed while hashing'):
        build_legacy_manifest(bag, 'bag', tmp_path)
    assert hash_called


def test_manifest_builder_rejects_malformed_yaml(tmp_path):
    bag, _ = write_closed_bag(tmp_path)
    (bag / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information: [', encoding='utf-8')
    with pytest.raises(ManifestError):
        build_legacy_manifest(bag, 'bag', tmp_path)


@pytest.mark.parametrize(
    'document',
    [
        None,
        [],
        {},
        {'rosbag2_bagfile_information': []},
        {'rosbag2_bagfile_information': {
            'relative_file_paths': ['bag_0.db3']}},
        {'rosbag2_bagfile_information': {
            'storage_identifier': 'sqlite3'}},
    ],
)
def test_manifest_builder_rejects_missing_metadata_mapping_or_key(
        tmp_path, document):
    bag, _ = write_closed_bag(tmp_path)
    (bag / 'metadata.yaml').write_text(
        yaml.safe_dump(document), encoding='utf-8')
    with pytest.raises(ManifestError):
        build_legacy_manifest(bag, 'bag', tmp_path)


@pytest.mark.parametrize(
    'unsafe',
    [
        '../bag_0.db3',
        '/bag_0.db3',
        'nested/../../bag_0.db3',
        'C:\\bag_0.db3',
        'C:/bag_0.db3',
    ],
)
def test_manifest_builder_rejects_unsafe_relative_file_paths(
        tmp_path, unsafe):
    bag, _ = write_closed_bag(tmp_path)
    metadata_path = bag / 'metadata.yaml'
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    metadata['rosbag2_bagfile_information']['relative_file_paths'] = [unsafe]
    metadata_path.write_text(yaml.safe_dump(metadata), encoding='utf-8')
    with pytest.raises(ManifestError):
        build_legacy_manifest(bag, 'bag', tmp_path)


def test_manifest_builder_rejects_unsupported_storage_identifier(tmp_path):
    bag, _ = write_closed_bag(tmp_path)
    metadata_path = bag / 'metadata.yaml'
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    metadata['rosbag2_bagfile_information']['storage_identifier'] = 'mcap'
    metadata_path.write_text(yaml.safe_dump(metadata), encoding='utf-8')
    with pytest.raises(ManifestError, match='storage must be sqlite3'):
        build_legacy_manifest(bag, 'bag', tmp_path)


def test_manifest_builder_rejects_duplicate_relative_file_paths(tmp_path):
    bag, storage = write_closed_bag(tmp_path)
    metadata_path = bag / 'metadata.yaml'
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    metadata['rosbag2_bagfile_information']['relative_file_paths'] = [
        storage.name, storage.name]
    metadata_path.write_text(yaml.safe_dump(metadata), encoding='utf-8')
    with pytest.raises(ManifestError, match='duplicate'):
        build_legacy_manifest(bag, 'bag', tmp_path)


@pytest.mark.parametrize(
    'section,key,value',
    [
        ('starting_time', 'nanoseconds_since_epoch', True),
        ('starting_time', 'nanoseconds_since_epoch', '12'),
        ('starting_time', 'nanoseconds_since_epoch', 12.5),
        ('starting_time', 'nanoseconds_since_epoch', float('inf')),
        ('starting_time', 'nanoseconds_since_epoch', -1),
        ('duration', 'nanoseconds', True),
        ('duration', 'nanoseconds', '34'),
        ('duration', 'nanoseconds', 34.5),
        ('duration', 'nanoseconds', float('inf')),
        ('duration', 'nanoseconds', -1),
    ],
)
def test_manifest_builder_rejects_invalid_time_scalars(
        tmp_path, section, key, value):
    bag, _ = write_closed_bag(tmp_path)
    metadata_path = bag / 'metadata.yaml'
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    metadata['rosbag2_bagfile_information'][section][key] = value
    metadata_path.write_text(yaml.safe_dump(metadata), encoding='utf-8')
    with pytest.raises(ManifestError):
        build_legacy_manifest(bag, 'bag', tmp_path)


@pytest.mark.parametrize(
    'field,value',
    [
        ('name', 7),
        ('type', ['std_msgs/msg/String']),
        ('message_count', True),
        ('message_count', '1'),
        ('message_count', 1.5),
        ('message_count', float('inf')),
        ('message_count', -1),
    ],
)
def test_manifest_builder_rejects_heterogeneous_topic_scalars(
        tmp_path, field, value):
    bag, _ = write_closed_bag(tmp_path)
    metadata_path = bag / 'metadata.yaml'
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    valid_topic = {
        'topic_metadata': {
            'name': '/camera',
            'type': 'sensor_msgs/msg/Image',
        },
        'message_count': 5,
    }
    invalid_topic = {
        'topic_metadata': {
            'name': '/second',
            'type': 'std_msgs/msg/String',
        },
        'message_count': 1,
    }
    if field == 'message_count':
        invalid_topic[field] = value
    else:
        invalid_topic['topic_metadata'][field] = value
    metadata['rosbag2_bagfile_information'][
        'topics_with_message_count'] = [valid_topic, invalid_topic]
    metadata_path.write_text(yaml.safe_dump(metadata), encoding='utf-8')
    with pytest.raises(ManifestError):
        build_legacy_manifest(bag, 'bag', tmp_path)


def test_build_field_manifest_requires_declared_split_and_calibration(tmp_path):
    bag, _ = write_closed_bag(
        tmp_path / 'rosbags' / 'semantic_search' / 'raw', 'field_bag')
    metadata_path = bag / 'metadata.yaml'
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    metadata['rosbag2_bagfile_information']['topics_with_message_count'] = [{
        'topic_metadata': {
            'name': '/odom',
            'type': 'nav_msgs/msg/Odometry',
        },
        'message_count': 5,
    }, {
        'topic_metadata': {
            'name': '/imu/data_raw',
            'type': 'sensor_msgs/msg/Imu',
        },
        'message_count': 5,
    }]
    metadata_path.write_text(yaml.safe_dump(metadata), encoding='utf-8')
    payload = build_field_manifest(
        bag_dir=bag,
        dataset_id='field_bag',
        workspace_root=tmp_path,
        split='validation',
        environment={
            'site_id': 'site_a',
            'session_id': 'session_a',
            'lighting': 'day',
            'surface': 'path',
            'weather': 'dry',
        },
        calibration={
            'camera_intrinsics_id': 'camera-sha',
            'camera_lidar_extrinsics_id': 'camera-lidar-sha',
            'lidar_imu_extrinsics_id': 'lidar-imu-sha',
            'localization_config_id': 'localization-sha',
        },
        world_pose=False,
        active_motion=False,
    )
    assert payload['split'] == 'validation'
    assert payload['bag']['relative_path'] == (
        'rosbags/semantic_search/raw/field_bag')
    assert payload['capabilities']['imu'] is True
    assert payload['capabilities']['local_pose'] is True


def valid_query():
    return {
        'query_id': 1,
        'stamp_ns': 5,
        'text': 'fallen branch blocking the path',
        'language': 'en',
        'client_request_id': 'field-001',
    }


def valid_object():
    return {
        'object_id': 'object-1',
        'physical_object_id': 'branch-1',
        'labels': ['fallen branch'],
        'site_id': 'site-a',
        'acquisition_date': '2026-07-13',
        'source': 'robot',
        'provenance': 'human-labelled',
    }


def valid_trial():
    return {
        'trial_id': 'trial-1',
        'query_id': 1,
        'target_object_id': 'object-1',
        'is_positive': True,
        'start_stamp_ns': 1,
        'end_stamp_ns': 10,
        'nominal_distance_m': 2.0,
        'observation_stage': 'passive',
        'site_id': 'site-a',
        'session_id': 'unit',
    }


def manifest_with_query_and_object():
    return add_object(
        add_query_event(valid_manifest(), valid_query()), valid_object())


def populated_manifest():
    return add_trial(manifest_with_query_and_object(), valid_trial())


def valid_annotation():
    return {
        'schema_version': '1.0.0',
        'dataset_id': 'unit_test_bag',
        'trial_id': 'trial-1',
        'stamp_ns': 5,
        'query_id': 1,
        'object_id': 'object-1',
        'visibility': 'visible',
        'label_source': 'human',
        'confidence': 1.0,
    }


@pytest.mark.parametrize(
    'mutate',
    [
        lambda payload: payload['bag'].update(start_time_ns=True),
        lambda payload: payload['bag'].update(duration_ns=True),
        lambda payload: payload['bag']['topics'][0].update(count=True),
    ],
)
def test_boolean_values_are_not_bag_integers(mutate):
    payload = valid_manifest()
    mutate(payload)
    with pytest.raises(ManifestError):
        validate_manifest(payload)


@pytest.mark.parametrize('field', ['query_id', 'stamp_ns'])
def test_boolean_values_are_not_query_integers(field):
    event = valid_query()
    event[field] = True
    with pytest.raises(ManifestError):
        add_query_event(valid_manifest(), event)


@pytest.mark.parametrize(
    'mutate',
    [
        lambda trial: trial.update(query_id=True),
        lambda trial: trial.update(start_stamp_ns=True),
        lambda trial: trial.update(end_stamp_ns=True),
        lambda trial: trial.update(nominal_distance_m=True),
        lambda trial: trial.update(nominal_distance_m=float('nan')),
        lambda trial: trial.update(nominal_distance_m=float('inf')),
        lambda trial: trial.update(nominal_distance_m=float('-inf')),
    ],
)
def test_trial_integer_and_number_fields_are_strict(mutate):
    trial = valid_trial()
    mutate(trial)
    with pytest.raises(ManifestError):
        add_trial(manifest_with_query_and_object(), trial)


def test_negative_trial_target_must_be_an_empty_string():
    trial = valid_trial()
    trial.update(is_positive=False, target_object_id=None)
    with pytest.raises(ManifestError):
        add_trial(manifest_with_query_and_object(), trial)


@pytest.mark.parametrize(
    'field,value',
    [
        ('query_id', True),
        ('stamp_ns', True),
        ('confidence', True),
        ('confidence', float('nan')),
        ('confidence', float('inf')),
        ('confidence', float('-inf')),
    ],
)
def test_annotation_integer_and_number_fields_are_strict(field, value):
    record = valid_annotation()
    record[field] = value
    with pytest.raises(ManifestError):
        validate_annotation_record(record, populated_manifest())


def test_annotation_checksum_must_be_a_string():
    payload = valid_manifest()
    payload['annotation_files'].append({
        'relative_path': 'annotations/trial-1.jsonl',
        'sha256': int('1' * 64),
        'format': 'jsonl',
        'schema_version': '1.0.0',
    })
    payload['capabilities']['annotations'] = True
    with pytest.raises(ManifestError):
        validate_manifest(payload)


@pytest.mark.parametrize(
    'unsafe',
    [
        'bags\\field_bag',
        'C:\\bags\\field_bag',
        'C:/bags/field_bag',
    ],
)
def test_manifest_paths_must_be_posix_relative(unsafe):
    payload = valid_manifest()
    payload['bag']['relative_path'] = unsafe
    with pytest.raises(ManifestError):
        validate_manifest(payload)


@pytest.mark.parametrize(
    'unsafe',
    [
        'masks\\frame.png',
        'C:\\masks\\frame.png',
        'C:/masks/frame.png',
        'masks/../frame.png',
    ],
)
def test_annotation_mask_path_must_be_posix_relative(unsafe):
    record = valid_annotation()
    record['mask_path'] = unsafe
    with pytest.raises(ManifestError):
        validate_annotation_record(record, populated_manifest())


def test_schema_paths_share_safe_posix_relative_pattern():
    schema_root = Path(__file__).parents[1] / 'schemas'
    dataset = json.loads(
        (schema_root / 'dataset_manifest.schema.json').read_text())
    annotation = json.loads(
        (schema_root / 'annotation.schema.json').read_text())
    path_schemas = [
        dataset['properties']['bag']['properties']['relative_path'],
        dataset['properties']['annotation_files']['items']['properties'][
            'relative_path'],
        annotation['properties']['mask_path'],
    ]
    patterns = [item.get('pattern') for item in path_schemas]
    assert all(patterns)
    assert len(set(patterns)) == 1
    pattern = patterns[0]
    assert re.search(pattern, 'bags/field_bag/frame.png')
    assert re.search(pattern, 'masks/object..name.png')
    for unsafe in (
            '/bags/field_bag',
            '../field_bag',
            'bags/../field_bag',
            'bags/field_bag/..',
            'bags\\field_bag',
            'C:\\bags\\field_bag',
            'C:/bags/field_bag'):
        assert re.search(pattern, unsafe) is None


def test_tree_checksum_frames_file_boundaries(tmp_path):
    single_file_tree = tmp_path / 'single'
    two_file_tree = tmp_path / 'two'
    single_file_tree.mkdir()
    two_file_tree.mkdir()
    boundary = len(b'b').to_bytes(8, 'big') + b'b'
    (single_file_tree / 'a').write_bytes(b'prefix' + boundary + b'suffix')
    (two_file_tree / 'a').write_bytes(b'prefix')
    (two_file_tree / 'b').write_bytes(b'suffix')
    assert sha256_tree(single_file_tree) != sha256_tree(two_file_tree)


def test_workspace_relative_path_error_is_generic(tmp_path):
    workspace_root = tmp_path / 'workspace'
    outside = tmp_path / 'outside'
    workspace_root.mkdir()
    outside.mkdir()
    with pytest.raises(
            ManifestError, match='path must be inside workspace_root'):
        workspace_relative_path(outside, workspace_root)
