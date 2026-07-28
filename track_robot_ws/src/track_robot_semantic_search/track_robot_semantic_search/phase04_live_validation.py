"""Bounded Phase 0-4 live evidence collector and consistency gates."""

import argparse
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import tempfile
import time


def _stamp_ns(message):
    header = getattr(message, 'header', None)
    stamp = getattr(header, 'stamp', None)
    if stamp is None:
        return None
    try:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return None


def _time_ns(value):
    try:
        return int(value.sec) * 1_000_000_000 + int(value.nanosec)
    except (AttributeError, TypeError, ValueError):
        return None


def _percentile(values, fraction):
    finite = sorted(
        float(value) for value in values
        if math.isfinite(float(value)))
    if not finite:
        return None
    index = int(math.ceil(fraction * len(finite))) - 1
    return finite[max(0, min(index, len(finite) - 1))]


def _phase(status, failures, evidence, latency_ms=None):
    return {
        'status': status,
        'failures': list(failures),
        'evidence': dict(evidence),
        'latency_ms': dict(latency_ms or {}),
    }


@dataclass
class Phase04LiveEvidence:
    expected_query_id: int
    expected_query_version: int
    collection_start_ns: int
    localization_messages: int = 0
    localization_epochs: set = field(default_factory=set)
    localization_frames: set = field(default_factory=set)
    localization_healthy_count: int = 0
    localization_reasons: set = field(default_factory=set)
    localization_stamps: list = field(default_factory=list)
    region_messages: int = 0
    nonempty_region_messages: int = 0
    region_query_ids: set = field(default_factory=set)
    region_query_versions: set = field(default_factory=set)
    region_frames: set = field(default_factory=set)
    region_scores: list = field(default_factory=list)
    region_stamps: list = field(default_factory=list)
    observation_messages: int = 0
    observation_count: int = 0
    observation_query_ids: set = field(default_factory=set)
    observation_query_versions: set = field(default_factory=set)
    observation_frames: set = field(default_factory=set)
    observation_localization_epochs: set = field(default_factory=set)
    observation_position_valid_count: int = 0
    observation_stamps: list = field(default_factory=list)
    observation_contract_failures: dict = field(default_factory=dict)
    lidar_tracklet_messages: int = 0
    lidar_tracklet_stamps: list = field(default_factory=list)
    camera_to_available_lidar_deltas_ms: list = field(
        default_factory=list)
    active_object_messages: int = 0
    active_object_count: int = 0
    active_top_global_ids: set = field(default_factory=set)
    active_memory_epochs: set = field(default_factory=set)
    active_localization_epochs: set = field(default_factory=set)
    active_query_ids: set = field(default_factory=set)
    active_query_versions: set = field(default_factory=set)
    active_position_frames: set = field(default_factory=set)
    active_valid_position_count: int = 0
    active_top_sample_count: int = 0
    active_stamps: list = field(default_factory=list)
    best_candidate_messages: int = 0
    best_candidate_count: int = 0
    best_global_ids: set = field(default_factory=set)
    best_memory_epochs: set = field(default_factory=set)
    best_localization_epochs: set = field(default_factory=set)
    best_query_ids: set = field(default_factory=set)
    best_query_versions: set = field(default_factory=set)
    best_position_frames: set = field(default_factory=set)
    best_relevance: list = field(default_factory=list)
    best_uncertainty: list = field(default_factory=list)
    best_stamps: list = field(default_factory=list)
    costmap_messages: int = 0
    costmap_frames: set = field(default_factory=set)
    costmap_stamps: list = field(default_factory=list)
    costmap_shapes: set = field(default_factory=set)
    phase4_diagnostic_messages: int = 0
    phase4_pass_messages: int = 0
    phase4_reasons: list = field(default_factory=list)
    phase4_latencies_ms: list = field(default_factory=list)
    phase4_memory_epochs: set = field(default_factory=set)
    phase4_global_ids: set = field(default_factory=set)
    phase4_localization_epochs: set = field(default_factory=set)
    phase4_query_ids: set = field(default_factory=set)
    phase4_query_versions: set = field(default_factory=set)
    path_messages: int = 0
    nonempty_path_messages: int = 0
    maximum_path_poses: int = 0
    path_frames: set = field(default_factory=set)
    path_stamps: list = field(default_factory=list)
    cmd_vel_publishers: int = 0
    motion_publishers: dict = field(default_factory=dict)
    advice_messages: int = 0
    ready_advice_messages: int = 0
    latest_advice: str = ''
    advisor_diagnostic_messages: int = 0
    advisor_ready_messages: int = 0
    advisor_reasons: list = field(default_factory=list)
    advisor_memory_epochs: set = field(default_factory=set)
    advisor_global_ids: set = field(default_factory=set)
    advisor_localization_epochs: set = field(default_factory=set)
    advisor_query_ids: set = field(default_factory=set)
    advisor_query_versions: set = field(default_factory=set)
    memory_diagnostic_messages: int = 0
    latest_memory_diagnostics: dict = field(default_factory=dict)

    def localization(self, message):
        self.localization_messages += 1
        self.localization_epochs.add(int(message.localization_epoch_id))
        self.localization_frames.add(str(message.canonical_frame_id))
        self.localization_reasons.add(str(message.reason))
        if bool(message.local_healthy):
            self.localization_healthy_count += 1
        stamp = _stamp_ns(message)
        if stamp is not None:
            self.localization_stamps.append(stamp)

    def regions(self, message):
        self.region_messages += 1
        self.region_query_ids.add(int(message.query_id))
        self.region_query_versions.add(int(message.query_version))
        self.region_frames.add(str(message.header.frame_id))
        regions = tuple(message.regions)
        if regions:
            self.nonempty_region_messages += 1
        stamp = _stamp_ns(message)
        if stamp is not None:
            self.region_stamps.append(stamp)
        for item in regions:
            self.region_query_ids.add(int(item.query_id))
            self.region_query_versions.add(int(item.query_version))
            self.region_scores.append(float(item.fused_score))

    def observations(self, message):
        self.observation_messages += 1
        self.observation_frames.add(str(message.header.frame_id))
        stamp = _stamp_ns(message)
        if stamp is not None:
            self.observation_stamps.append(stamp)
            if self.lidar_tracklet_stamps:
                nearest = min(
                    self.lidar_tracklet_stamps,
                    key=lambda value: abs(int(value) - stamp))
                self.camera_to_available_lidar_deltas_ms.append(
                    (stamp - int(nearest)) / 1_000_000.0)
        for item in message.observations:
            self.observation_count += 1
            self.observation_query_ids.add(int(item.query_id))
            self.observation_query_versions.add(int(item.query_version))
            self.observation_localization_epochs.add(
                int(item.localization_epoch_id))
            if bool(item.position_valid):
                self.observation_position_valid_count += 1
            batch_epoch = getattr(message, 'producer_epoch_id', None)
            reason = (
                self._observation_contract_failure(item, int(batch_epoch))
                if batch_epoch is not None
                and hasattr(item, 'producer_epoch_id')
                else None)
            if reason is not None:
                self.observation_contract_failures[reason] = (
                    self.observation_contract_failures.get(reason, 0) + 1)

    @staticmethod
    def _observation_contract_failure(item, producer_epoch_id):
        if int(item.producer_epoch_id) != producer_epoch_id:
            return 'producer_epoch_mismatch'
        if int(item.visual_candidate_id) == 0:
            return 'invalid_visual_candidate_id'
        if not bool(item.camera_stamp_valid):
            return 'camera_stamp_invalid'
        width = int(item.image_width)
        height = int(item.image_height)
        if width <= 0 or height <= 0:
            return 'invalid_image_dimensions'
        roi = item.roi
        if int(roi.width) <= 0 or int(roi.height) <= 0:
            return 'empty_roi'
        if (
                int(roi.x_offset) < 0
                or int(roi.y_offset) < 0
                or int(roi.x_offset) + int(roi.width) > width
                or int(roi.y_offset) + int(roi.height) > height):
            return 'roi_out_of_bounds'
        return None

    def active_objects(self, message):
        self.active_object_messages += 1
        self.active_object_count += len(message.objects)
        self.active_memory_epochs.add(int(message.memory_epoch_id))
        stamp = _stamp_ns(message)
        if stamp is not None:
            self.active_stamps.append(stamp)
        if not message.objects:
            return
        query_objects = [
            item for item in message.objects
            if (
                int(item.active_query_id) == self.expected_query_id
                and int(item.active_query_version)
                == self.expected_query_version)
        ]
        if not query_objects:
            return
        top = max(
            query_objects,
            key=lambda item: (
                float(item.task_relevance),
                -int(item.global_object_id)))
        self.active_top_sample_count += 1
        self.active_top_global_ids.add(int(top.global_object_id))
        self.active_localization_epochs.add(
            int(top.localization_epoch_id))
        self.active_query_ids.add(int(top.active_query_id))
        self.active_query_versions.add(int(top.active_query_version))
        self.active_position_frames.add(str(top.position_frame_id))
        if bool(top.position_valid):
            self.active_valid_position_count += 1

    def lidar_tracklets(self, message):
        self.lidar_tracklet_messages += 1
        stamp = _stamp_ns(message)
        if stamp is not None:
            self.lidar_tracklet_stamps.append(stamp)

    def best_candidate(self, message):
        self.best_candidate_messages += 1
        self.best_candidate_count += len(message.objects)
        self.best_memory_epochs.add(int(message.memory_epoch_id))
        stamp = _stamp_ns(message)
        if stamp is not None:
            self.best_stamps.append(stamp)
        for item in message.objects:
            self.best_global_ids.add(int(item.global_object_id))
            self.best_localization_epochs.add(
                int(item.localization_epoch_id))
            self.best_query_ids.add(int(item.active_query_id))
            self.best_query_versions.add(
                int(item.active_query_version))
            self.best_position_frames.add(str(item.position_frame_id))
            self.best_relevance.append(float(item.task_relevance))
            self.best_uncertainty.append(float(item.uncertainty))

    def costmap(self, message):
        self.costmap_messages += 1
        self.costmap_frames.add(str(message.header.frame_id))
        self.costmap_shapes.add((
            int(message.info.width),
            int(message.info.height),
            float(message.info.resolution)))
        stamp = _stamp_ns(message)
        if stamp is not None:
            self.costmap_stamps.append(stamp)

    def phase4_diagnostics(self, message):
        for status in message.status:
            values = {
                str(item.key): str(item.value)
                for item in status.values}
            self.phase4_diagnostic_messages += 1
            reason = values.get('reason', str(status.message))
            self.phase4_reasons.append(reason)
            if values.get('status') == 'PASS' and reason == 'planned':
                self.phase4_pass_messages += 1
            self._integer_value(
                values, 'memory_epoch_id', self.phase4_memory_epochs)
            self._integer_value(
                values, 'global_object_id', self.phase4_global_ids)
            self._integer_value(
                values,
                'localization_epoch_id',
                self.phase4_localization_epochs)
            self._integer_value(
                values, 'query_id', self.phase4_query_ids)
            self._integer_value(
                values, 'query_version', self.phase4_query_versions)
            try:
                self.phase4_latencies_ms.append(
                    float(values['latency_ms']))
            except (KeyError, TypeError, ValueError):
                pass

    @staticmethod
    def _integer_value(values, key, destination):
        try:
            value = int(values[key])
        except (KeyError, TypeError, ValueError):
            return
        if value > 0:
            destination.add(value)

    def path(self, message):
        self.path_messages += 1
        count = len(message.poses)
        if count:
            self.nonempty_path_messages += 1
        self.maximum_path_poses = max(self.maximum_path_poses, count)
        self.path_frames.add(str(message.header.frame_id))
        stamp = _stamp_ns(message)
        if stamp is not None:
            self.path_stamps.append(stamp)

    def advice(self, message):
        text = str(message.data).strip()
        self.advice_messages += 1
        self.latest_advice = text
        if text.startswith('READY ') and text.endswith(' ADVISORY_ONLY'):
            self.ready_advice_messages += 1

    def phase4a_diagnostics(self, message):
        for status in message.status:
            values = {
                str(item.key): str(item.value)
                for item in status.values}
            self.advisor_diagnostic_messages += 1
            reason = values.get('reason', str(status.message))
            self.advisor_reasons.append(reason)
            if values.get('status') == 'READY' and reason == 'ready':
                self.advisor_ready_messages += 1
            self._integer_value(
                values, 'memory_epoch_id', self.advisor_memory_epochs)
            self._integer_value(
                values, 'global_object_id', self.advisor_global_ids)
            self._integer_value(
                values,
                'localization_epoch_id',
                self.advisor_localization_epochs)
            self._integer_value(
                values, 'query_id', self.advisor_query_ids)
            self._integer_value(
                values, 'query_version', self.advisor_query_versions)

    def memory_diagnostics(self, message):
        for status in message.status:
            self.memory_diagnostic_messages += 1
            values = {
                str(item.key): str(item.value)
                for item in status.values}
            values['status_message'] = str(status.message)
            self.latest_memory_diagnostics = values


def _monotonic(values):
    return all(right >= left for left, right in zip(values, values[1:]))


def _phase0(evidence):
    if evidence.localization_messages == 0:
        return _phase('NOT EVALUATED', [], {
            'localization_messages': 0})
    failures = []
    if evidence.localization_healthy_count != evidence.localization_messages:
        failures.append('local localization was not healthy for every sample')
    if len(evidence.localization_epochs) != 1:
        failures.append('localization epoch was not stable')
    if len(evidence.localization_frames) != 1:
        failures.append('canonical localization frame was not stable')
    if not _monotonic(evidence.localization_stamps):
        failures.append('localization timestamps were not monotonic')
    return _phase('FAIL' if failures else 'PASS', failures, {
        'localization_messages': evidence.localization_messages,
        'healthy_messages': evidence.localization_healthy_count,
        'localization_epoch_ids': sorted(evidence.localization_epochs),
        'canonical_frames': sorted(evidence.localization_frames),
        'reasons': sorted(evidence.localization_reasons),
    })


def _phase1(evidence):
    if evidence.region_messages == 0 and evidence.observation_messages == 0:
        return _phase('NOT EVALUATED', [], {
            'region_messages': 0,
            'observation_messages': 0})
    failures = []
    if evidence.region_messages == 0:
        failures.append('no semantic region messages')
    if evidence.nonempty_region_messages == 0:
        failures.append('no valid Phase 1 semantic regions')
    if evidence.observation_messages == 0 or evidence.observation_count == 0:
        failures.append('no Phase 1 semantic observations')
    query_ids = evidence.region_query_ids | evidence.observation_query_ids
    versions = (
        evidence.region_query_versions
        | evidence.observation_query_versions)
    if query_ids != {evidence.expected_query_id}:
        failures.append('Phase 1 query ID mismatch')
    if versions != {evidence.expected_query_version}:
        failures.append('Phase 1 query version mismatch')
    if not _monotonic(evidence.region_stamps):
        failures.append('region timestamps were not monotonic')
    if not _monotonic(evidence.observation_stamps):
        failures.append('observation timestamps were not monotonic')
    return _phase('FAIL' if failures else 'PASS', failures, {
        'region_messages': evidence.region_messages,
        'nonempty_region_messages': evidence.nonempty_region_messages,
        'observation_messages': evidence.observation_messages,
        'observation_count': evidence.observation_count,
        'query_ids': sorted(query_ids),
        'query_versions': sorted(versions),
        'region_frames': sorted(evidence.region_frames),
        'observation_frames': sorted(evidence.observation_frames),
        'observation_contract_failures':
            dict(evidence.observation_contract_failures),
        'score_minimum': (
            min(evidence.region_scores)
            if evidence.region_scores else None),
        'score_mean': (
            sum(evidence.region_scores) / len(evidence.region_scores)
            if evidence.region_scores else None),
        'score_maximum': (
            max(evidence.region_scores)
            if evidence.region_scores else None),
    })


def _nearest_source_delta_ms(camera_stamps, lidar_stamps):
    if not camera_stamps or not lidar_stamps:
        return {
            'sample_count': 0,
            'minimum': None,
            'p50': None,
            'p95': None,
            'maximum': None,
        }
    lidar = sorted(int(value) for value in lidar_stamps)
    deltas = []
    index = 0
    for camera_stamp in sorted(int(value) for value in camera_stamps):
        while (
                index + 1 < len(lidar)
                and abs(lidar[index + 1] - camera_stamp)
                <= abs(lidar[index] - camera_stamp)):
            index += 1
        deltas.append(abs(lidar[index] - camera_stamp) / 1_000_000.0)
    return {
        'sample_count': len(deltas),
        'minimum': min(deltas),
        'p50': _percentile(deltas, 0.50),
        'p95': _percentile(deltas, 0.95),
        'maximum': max(deltas),
    }


def _timing_summary_ms(values):
    finite = [
        float(value) for value in values
        if math.isfinite(float(value))]
    if not finite:
        return {
            'sample_count': 0,
            'minimum': None,
            'p50': None,
            'p95': None,
            'maximum': None,
        }
    return {
        'sample_count': len(finite),
        'minimum': min(finite),
        'p50': _percentile(finite, 0.50),
        'p95': _percentile(finite, 0.95),
        'maximum': max(finite),
    }


def _phase2(evidence):
    source_delta = _nearest_source_delta_ms(
        evidence.observation_stamps,
        evidence.lidar_tracklet_stamps)
    available_delta = _timing_summary_ms(
        evidence.camera_to_available_lidar_deltas_ms)
    if evidence.active_object_messages == 0:
        return _phase('NOT EVALUATED', [], {
            'active_object_messages': 0,
            'lidar_tracklet_messages': evidence.lidar_tracklet_messages,
            'camera_lidar_nearest_delta_ms': source_delta,
            'camera_to_available_lidar_delta_ms': available_delta,
            'runtime_diagnostic_messages':
                evidence.memory_diagnostic_messages,
            'latest_runtime_diagnostics':
                dict(evidence.latest_memory_diagnostics),
        })
    failures = []
    if evidence.active_top_sample_count == 0:
        failures.append('no active semantic object')
    if len(evidence.active_top_global_ids) != 1:
        failures.append('global object ID was not stable')
    if (
            evidence.active_top_sample_count > 0
            and evidence.active_valid_position_count
            != evidence.active_top_sample_count):
        failures.append('stable object did not have a valid 3D position')
    if len(evidence.active_memory_epochs) != 1:
        failures.append('memory epoch was not stable')
    if len(evidence.active_localization_epochs) != 1:
        failures.append('object localization epoch was not stable')
    if (
            evidence.active_query_ids
            and evidence.active_query_ids != {evidence.expected_query_id}):
        failures.append('Phase 2 target query ID mismatch')
    if (
            evidence.active_query_versions
            and evidence.active_query_versions
            != {evidence.expected_query_version}):
        failures.append('Phase 2 target query version mismatch')
    if not _monotonic(evidence.active_stamps):
        failures.append('active object timestamps were not monotonic')
    return _phase('FAIL' if failures else 'PASS', failures, {
        'active_object_messages': evidence.active_object_messages,
        'object_count_total': evidence.active_object_count,
        'top_object_samples': evidence.active_top_sample_count,
        'valid_position_samples': evidence.active_valid_position_count,
        'global_object_ids': sorted(evidence.active_top_global_ids),
        'memory_epoch_ids': sorted(evidence.active_memory_epochs),
        'localization_epoch_ids': sorted(
            evidence.active_localization_epochs),
        'position_frames': sorted(evidence.active_position_frames),
        'query_ids': sorted(evidence.active_query_ids),
        'query_versions': sorted(evidence.active_query_versions),
        'lidar_tracklet_messages': evidence.lidar_tracklet_messages,
        'camera_lidar_nearest_delta_ms': source_delta,
        'camera_to_available_lidar_delta_ms': available_delta,
        'runtime_diagnostic_messages':
            evidence.memory_diagnostic_messages,
        'latest_runtime_diagnostics':
            dict(evidence.latest_memory_diagnostics),
    })


def _phase3(evidence):
    if evidence.best_candidate_messages == 0:
        return _phase('NOT EVALUATED', [], {
            'best_candidate_messages': 0})
    failures = []
    if evidence.best_candidate_count == 0:
        failures.append('no selected target')
    if len(evidence.best_global_ids) > 1:
        failures.append('selected target ID was not stable')
    if (
            evidence.best_global_ids
            and evidence.active_top_global_ids
            and evidence.best_global_ids != evidence.active_top_global_ids):
        failures.append('selected target does not reference Phase 2 object')
    if (
            evidence.best_relevance
            and min(evidence.best_relevance) < 0.5):
        failures.append('selected target confidence below 0.5')
    if (
            evidence.best_uncertainty
            and max(evidence.best_uncertainty) > 0.5):
        failures.append('selected target uncertainty above 0.5')
    if not _monotonic(evidence.best_stamps):
        failures.append('selected target timestamps were not monotonic')
    return _phase('FAIL' if failures else 'PASS', failures, {
        'best_candidate_messages': evidence.best_candidate_messages,
        'selected_target_count_total': evidence.best_candidate_count,
        'global_object_ids': sorted(evidence.best_global_ids),
        'memory_epoch_ids': sorted(evidence.best_memory_epochs),
        'localization_epoch_ids': sorted(
            evidence.best_localization_epochs),
        'position_frames': sorted(evidence.best_position_frames),
        'query_ids': sorted(evidence.best_query_ids),
        'query_versions': sorted(evidence.best_query_versions),
        'confidence_minimum': (
            min(evidence.best_relevance)
            if evidence.best_relevance else None),
        'uncertainty_maximum': (
            max(evidence.best_uncertainty)
            if evidence.best_uncertainty else None),
    })


def _phase4(evidence):
    if (
            evidence.phase4_diagnostic_messages == 0
            and evidence.path_messages == 0):
        return _phase('NOT EVALUATED', [], {
            'diagnostic_messages': 0,
            'path_messages': 0})
    failures = []
    if evidence.phase4_pass_messages == 0:
        reason = (
            evidence.phase4_reasons[-1]
            if evidence.phase4_reasons else 'no successful plan')
        failures.append(reason)
    if evidence.nonempty_path_messages == 0:
        failures.append('no collision-free planned path')
    if evidence.costmap_messages == 0:
        failures.append('no costmap messages')
    if not _monotonic(evidence.costmap_stamps):
        failures.append('costmap timestamps were not monotonic')
    if not _monotonic(evidence.path_stamps):
        failures.append('path timestamps were not monotonic')
    return _phase(
        'FAIL' if failures else 'PASS',
        failures,
        {
            'diagnostic_messages': evidence.phase4_diagnostic_messages,
            'planned_messages': evidence.phase4_pass_messages,
            'reasons': sorted(set(evidence.phase4_reasons)),
            'costmap_messages': evidence.costmap_messages,
            'costmap_frames': sorted(evidence.costmap_frames),
            'costmap_shapes': [
                list(item) for item in sorted(evidence.costmap_shapes)],
            'path_messages': evidence.path_messages,
            'nonempty_path_messages': evidence.nonempty_path_messages,
            'maximum_path_poses': evidence.maximum_path_poses,
            'path_frames': sorted(evidence.path_frames),
        },
        {
            'planner_p50': _percentile(
                evidence.phase4_latencies_ms, 0.50),
            'planner_p95': _percentile(
                evidence.phase4_latencies_ms, 0.95),
        })


def _phase4a_advisory(evidence):
    if (
            evidence.advice_messages == 0
            and evidence.advisor_diagnostic_messages == 0):
        return _phase('NOT EVALUATED', [], {
            'advice_messages': 0,
            'diagnostic_messages': 0,
        })
    failures = []
    if evidence.ready_advice_messages == 0:
        reason = (
            evidence.advisor_reasons[-1]
            if evidence.advisor_reasons else 'no READY approach advice')
        failures.append(reason)
    if evidence.advisor_ready_messages == 0:
        failures.append('no correlated READY advisory diagnostic')
    if (
            evidence.latest_advice
            and not evidence.latest_advice.endswith('ADVISORY_ONLY')):
        failures.append('advice was not marked ADVISORY_ONLY')
    return _phase('FAIL' if failures else 'PASS', failures, {
        'advice_messages': evidence.advice_messages,
        'ready_advice_messages': evidence.ready_advice_messages,
        'diagnostic_messages': evidence.advisor_diagnostic_messages,
        'ready_diagnostic_messages': evidence.advisor_ready_messages,
        'reasons': sorted(set(evidence.advisor_reasons)),
        'latest_advice': evidence.latest_advice,
        'memory_epoch_ids': sorted(evidence.advisor_memory_epochs),
        'global_object_ids': sorted(evidence.advisor_global_ids),
        'localization_epoch_ids': sorted(
            evidence.advisor_localization_epochs),
        'query_ids': sorted(evidence.advisor_query_ids),
        'query_versions': sorted(evidence.advisor_query_versions),
    })


def _consistency(evidence):
    memory_epochs = (
        evidence.active_memory_epochs
        | evidence.best_memory_epochs
        | evidence.phase4_memory_epochs
        | evidence.advisor_memory_epochs)
    global_ids = (
        evidence.active_top_global_ids
        | evidence.best_global_ids
        | evidence.phase4_global_ids
        | evidence.advisor_global_ids)
    localization_epochs = (
        evidence.localization_epochs
        | evidence.active_localization_epochs
        | evidence.best_localization_epochs
        | evidence.phase4_localization_epochs
        | evidence.advisor_localization_epochs)
    query_ids = (
        evidence.region_query_ids
        | evidence.observation_query_ids
        | evidence.active_query_ids
        | evidence.best_query_ids
        | evidence.phase4_query_ids
        | evidence.advisor_query_ids)
    query_versions = (
        evidence.region_query_versions
        | evidence.observation_query_versions
        | evidence.active_query_versions
        | evidence.best_query_versions
        | evidence.phase4_query_versions
        | evidence.advisor_query_versions)
    frames = (
        evidence.active_position_frames
        | evidence.best_position_frames
        | evidence.costmap_frames
        | evidence.path_frames)
    failures = []
    for label, values in (
            ('memory epoch', memory_epochs),
            ('global object ID', global_ids),
            ('localization epoch', localization_epochs),
            ('query ID', query_ids),
            ('query version', query_versions),
            ('planning frame', frames)):
        if len(values) > 1:
            failures.append('{} mismatch: {}'.format(
                label, sorted(values)))
    if query_ids and query_ids != {evidence.expected_query_id}:
        failures.append('query reference differs from accepted query ID')
    if (
            query_versions
            and query_versions != {evidence.expected_query_version}):
        failures.append(
            'query reference differs from accepted query version')
    evaluated = any((
        memory_epochs,
        global_ids,
        localization_epochs,
        query_ids,
        frames,
    ))
    status = (
        'NOT EVALUATED'
        if not evaluated else ('FAIL' if failures else 'PASS'))
    return {
        'status': status,
        'failures': failures,
        'memory_epoch_ids': sorted(memory_epochs),
        'global_object_ids': sorted(global_ids),
        'localization_epoch_ids': sorted(localization_epochs),
        'query_ids': sorted(query_ids),
        'query_versions': sorted(query_versions),
        'planning_frames': sorted(frames),
    }


def build_live_report(
        evidence, query_text, duration_sec, require_advisory=False):
    phases = {
        'phase0': _phase0(evidence),
        'phase1': _phase1(evidence),
        'phase2': _phase2(evidence),
        'phase3': _phase3(evidence),
        'phase4': _phase4(evidence),
    }
    if require_advisory:
        phases['phase4a_advisory'] = _phase4a_advisory(evidence)
    motion_publishers = dict(evidence.motion_publishers)
    if not motion_publishers and evidence.cmd_vel_publishers:
        motion_publishers['/cmd_vel'] = int(evidence.cmd_vel_publishers)
    motion_publisher_count = sum(motion_publishers.values())
    return {
        'schema_version': 'phase0_4_live_validation/1.0.0',
        'generated_at': datetime.now(timezone.utc).replace(
            microsecond=0).isoformat().replace('+00:00', 'Z'),
        'query': {
            'text': str(query_text),
            'query_id': int(evidence.expected_query_id),
            'query_version': int(evidence.expected_query_version),
        },
        'collection': {
            'duration_sec': float(duration_sec),
            'start_ns': int(evidence.collection_start_ns),
        },
        'phases': phases,
        'cross_phase_consistency': _consistency(evidence),
        'safety': {
            'planning_only': True,
            'advisory_only': bool(require_advisory),
            'cmd_vel_publishers': int(evidence.cmd_vel_publishers),
            'motion_publishers': motion_publishers,
            'status': (
                'PASS'
                if motion_publisher_count == 0 else 'FAIL'),
        },
    }


def collect_live(
        duration_sec,
        query_id,
        query_version,
        localization_topic='/semantic_memory/localization_state',
        active_objects_topic='/semantic_memory/active_objects',
        selected_target_topic='/semantic_memory/best_candidate',
        collect_phase4a=False):
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import OccupancyGrid, Path
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import String
    from track_robot_interfaces.msg import (
        SemanticLidarTrackletArray,
        SemanticLocalizationState,
        SemanticObjectArray,
        SemanticObservationArray,
        SemanticRegionArray,
    )

    evidence = Phase04LiveEvidence(
        int(query_id), int(query_version), time.time_ns())
    rclpy.init()
    node = rclpy.create_node('phase04_live_validation_collector')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    selected_qos = QoSProfile(depth=1)
    selected_qos.reliability = ReliabilityPolicy.RELIABLE
    selected_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    subscriptions = [
        node.create_subscription(
            SemanticLidarTrackletArray,
            '/semantic_memory/lidar_tracklets',
            evidence.lidar_tracklets,
            10),
        node.create_subscription(
            SemanticLocalizationState,
            localization_topic,
            evidence.localization,
            10),
        node.create_subscription(
            SemanticRegionArray,
            '/semantic_search/regions',
            evidence.regions,
            10),
        node.create_subscription(
            SemanticObservationArray,
            '/semantic_memory/observations',
            evidence.observations,
            10),
        node.create_subscription(
            SemanticObjectArray,
            active_objects_topic,
            evidence.active_objects,
            10),
        node.create_subscription(
            DiagnosticArray,
            '/semantic_memory/diagnostics',
            evidence.memory_diagnostics,
            10),
        node.create_subscription(
            SemanticObjectArray,
            selected_target_topic,
            evidence.best_candidate,
            selected_qos),
        node.create_subscription(
            OccupancyGrid,
            '/safety/local_obstacle_grid',
            evidence.costmap,
            5),
        node.create_subscription(
            DiagnosticArray,
            '/semantic_search/phase4/diagnostics',
            evidence.phase4_diagnostics,
            10),
        node.create_subscription(
            Path,
            '/semantic_search/phase4/planned_path',
            evidence.path,
            5),
    ]
    if collect_phase4a:
        subscriptions.extend([
            node.create_subscription(
                String,
                '/semantic_search/phase4a/advice',
                evidence.advice,
                10),
            node.create_subscription(
                DiagnosticArray,
                '/semantic_search/phase4a/diagnostics',
                evidence.phase4a_diagnostics,
                10),
        ])
    try:
        deadline = time.monotonic() + float(duration_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        motion_topics = (
            '/cmd_vel',
            '/cmd_vel_raw',
            '/cmd_vel_out',
            '/move_base_simple/goal',
            '/goal_pose',
            '/navigate_to_pose/_action/goal',
        )
        evidence.motion_publishers = {
            topic: count
            for topic in motion_topics
            for count in (len(node.get_publishers_info_by_topic(topic)),)
            if count > 0
        }
        evidence.cmd_vel_publishers = evidence.motion_publishers.get(
            '/cmd_vel', 0)
    finally:
        del subscriptions
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return evidence


def _write(path, payload):
    encoded = json.dumps(
        payload, allow_nan=False, indent=2, sort_keys=True) + '\n'
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=path.name + '.', suffix='.tmp', dir=str(path.parent))
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, str(path))
    finally:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass


def _parser():
    parser = argparse.ArgumentParser(
        description='Collect one bounded live Phase 0-4 validation report')
    parser.add_argument('--query', required=True)
    parser.add_argument('--query-id', required=True, type=int)
    parser.add_argument('--query-version', required=True, type=int)
    parser.add_argument('--duration-sec', type=float, default=20.0)
    parser.add_argument('--output', required=True)
    return parser


def main(argv=None):
    args = _parser().parse_args(argv)
    if (
            args.query_id <= 0
            or args.query_version <= 0
            or not math.isfinite(args.duration_sec)
            or args.duration_sec <= 0.0):
        return 4
    evidence = collect_live(
        args.duration_sec, args.query_id, args.query_version)
    report = build_live_report(
        evidence, args.query, args.duration_sec)
    _write(Path(args.output).expanduser(), report)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
