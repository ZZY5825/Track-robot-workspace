import argparse
import json
import sys
from pathlib import Path

from .manifest import (
    ManifestError,
    add_annotation_file,
    add_object,
    add_query_event,
    add_trial,
    build_field_manifest,
    build_legacy_manifest,
    load_manifest,
    write_json_atomic,
)


def parser():
    root = argparse.ArgumentParser()
    commands = root.add_subparsers(dest='command', required=True)
    validate = commands.add_parser('validate')
    validate.add_argument('manifest', type=Path)
    legacy = commands.add_parser('create-legacy')
    legacy.add_argument('bag_dir', type=Path)
    legacy.add_argument('output', type=Path)
    legacy.add_argument('--dataset-id', default='')
    legacy.add_argument('--workspace-root', type=Path, required=True)
    field = commands.add_parser('create-field')
    field.add_argument('bag_dir', type=Path)
    field.add_argument('output', type=Path)
    field.add_argument('--dataset-id', required=True)
    field.add_argument('--workspace-root', type=Path, required=True)
    field.add_argument(
        '--split',
        choices=('train', 'validation', 'test', 'extension'),
        required=True)
    field.add_argument('--site-id', required=True)
    field.add_argument('--session-id', required=True)
    field.add_argument('--lighting', required=True)
    field.add_argument('--surface', required=True)
    field.add_argument('--weather', required=True)
    field.add_argument('--camera-intrinsics-id', required=True)
    field.add_argument('--camera-lidar-extrinsics-id', required=True)
    field.add_argument('--lidar-imu-extrinsics-id', required=True)
    field.add_argument('--localization-config-id', required=True)
    field.add_argument('--world-pose', action='store_true')
    field.add_argument('--active-motion', action='store_true')
    query = commands.add_parser('add-query')
    query.add_argument('manifest', type=Path)
    query.add_argument('--query-id', type=int, required=True)
    query.add_argument('--stamp-ns', type=int, required=True)
    query.add_argument('--text', required=True)
    query.add_argument('--language', default='en')
    query.add_argument('--client-request-id', default='')
    object_parser = commands.add_parser('add-object')
    object_parser.add_argument('manifest', type=Path)
    object_parser.add_argument('--object-id', required=True)
    object_parser.add_argument('--physical-object-id', required=True)
    object_parser.add_argument('--label', action='append', required=True)
    object_parser.add_argument('--site-id', required=True)
    object_parser.add_argument('--acquisition-date', required=True)
    object_parser.add_argument(
        '--source', choices=('robot', 'public', 'synthetic'), required=True)
    object_parser.add_argument('--provenance', required=True)
    trial = commands.add_parser('add-trial')
    trial.add_argument('manifest', type=Path)
    trial.add_argument('--trial-id', required=True)
    trial.add_argument('--query-id', type=int, required=True)
    trial.add_argument('--target-object-id', default='')
    trial.add_argument('--positive', action='store_true')
    trial.add_argument('--start-stamp-ns', type=int, required=True)
    trial.add_argument('--end-stamp-ns', type=int, required=True)
    trial.add_argument('--nominal-distance-m', type=float, required=True)
    trial.add_argument(
        '--observation-stage',
        choices=('passive', 'pre_rotation', 'post_rotation'),
        required=True)
    trial.add_argument('--site-id', required=True)
    trial.add_argument('--session-id', required=True)
    annotations = commands.add_parser('add-annotations')
    annotations.add_argument('manifest', type=Path)
    annotations.add_argument('annotation_file', type=Path)
    annotations.add_argument('--workspace-root', type=Path, required=True)
    return root


def run(arguments) -> int:
    if arguments.command == 'validate':
        payload = load_manifest(arguments.manifest)
        print(json.dumps({
            'dataset_id': payload['dataset_id'],
            'schema_version': payload['schema_version'],
            'valid': True,
        }, sort_keys=True))
        return 0
    if arguments.command == 'create-legacy':
        dataset_id = arguments.dataset_id or arguments.bag_dir.name
        payload = build_legacy_manifest(
            arguments.bag_dir, dataset_id, arguments.workspace_root)
        write_json_atomic(arguments.output, payload)
        print(str(arguments.output))
        return 0
    if arguments.command == 'create-field':
        payload = build_field_manifest(
            bag_dir=arguments.bag_dir,
            dataset_id=arguments.dataset_id,
            workspace_root=arguments.workspace_root,
            split=arguments.split,
            environment={
                'site_id': arguments.site_id,
                'session_id': arguments.session_id,
                'lighting': arguments.lighting,
                'surface': arguments.surface,
                'weather': arguments.weather,
            },
            calibration={
                'camera_intrinsics_id': arguments.camera_intrinsics_id,
                'camera_lidar_extrinsics_id':
                arguments.camera_lidar_extrinsics_id,
                'lidar_imu_extrinsics_id':
                arguments.lidar_imu_extrinsics_id,
                'localization_config_id':
                arguments.localization_config_id,
            },
            world_pose=arguments.world_pose,
            active_motion=arguments.active_motion,
        )
        write_json_atomic(arguments.output, payload)
        print(str(arguments.output))
        return 0
    payload = load_manifest(arguments.manifest)
    if arguments.command == 'add-query':
        event = {
            'query_id': arguments.query_id,
            'stamp_ns': arguments.stamp_ns,
            'text': arguments.text,
            'language': arguments.language,
            'client_request_id': arguments.client_request_id,
        }
        updated = add_query_event(payload, event)
    elif arguments.command == 'add-object':
        updated = add_object(payload, {
            'object_id': arguments.object_id,
            'physical_object_id': arguments.physical_object_id,
            'labels': arguments.label,
            'site_id': arguments.site_id,
            'acquisition_date': arguments.acquisition_date,
            'source': arguments.source,
            'provenance': arguments.provenance,
        })
    elif arguments.command == 'add-trial':
        updated = add_trial(payload, {
            'trial_id': arguments.trial_id,
            'query_id': arguments.query_id,
            'target_object_id': arguments.target_object_id,
            'is_positive': arguments.positive,
            'start_stamp_ns': arguments.start_stamp_ns,
            'end_stamp_ns': arguments.end_stamp_ns,
            'nominal_distance_m': arguments.nominal_distance_m,
            'observation_stage': arguments.observation_stage,
            'site_id': arguments.site_id,
            'session_id': arguments.session_id,
        })
    else:
        updated = add_annotation_file(
            payload, arguments.annotation_file, arguments.workspace_root)
    write_json_atomic(arguments.manifest, updated)
    print(str(arguments.manifest))
    return 0


def main(argv=None):
    try:
        return run(parser().parse_args(argv))
    except (ManifestError, OSError, ValueError) as error:
        print('manifest error: {}'.format(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
