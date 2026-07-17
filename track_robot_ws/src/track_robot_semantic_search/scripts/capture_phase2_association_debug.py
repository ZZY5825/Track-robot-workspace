#!/usr/bin/env python3

import argparse
from collections import deque
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from track_robot_interfaces.msg import (
    AssociationDebug,
    SemanticLidarTrackletArray,
    SemanticObservationArray,
)


DECISION_NAMES = {
    AssociationDebug.DECISION_REJECTED_GATE: 'rejected_gate',
    AssociationDebug.DECISION_UNMATCHED: 'unmatched',
    AssociationDebug.DECISION_TENTATIVE: 'tentative',
    AssociationDebug.DECISION_MATCHED: 'matched',
    AssociationDebug.DECISION_AMBIGUOUS: 'ambiguous',
}


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def finite_or_none(value):
    value = float(value)
    return value if math.isfinite(value) else None


class CaptureNode(Node):
    def __init__(self, debug_path, context_path):
        super().__init__('phase2_association_debug_capture')
        debug_path.parent.mkdir(parents=True, exist_ok=True)
        context_path.parent.mkdir(parents=True, exist_ok=True)
        self._debug_stream = debug_path.open('w', encoding='utf-8')
        self._context_stream = context_path.open('w', encoding='utf-8')
        self._tracklet_stamps = {}
        self._pair_ids = set()

        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        best_effort = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            SemanticLidarTrackletArray,
            '/semantic_memory/lidar_tracklets',
            self._on_tracklets,
            best_effort)
        self.create_subscription(
            SemanticObservationArray,
            '/semantic_memory/observations',
            self._on_observations,
            reliable)
        self.create_subscription(
            AssociationDebug,
            '/semantic_memory/association_debug',
            self._on_debug,
            reliable)

    @staticmethod
    def _write(stream, row):
        stream.write(json.dumps(row, sort_keys=True, allow_nan=False) + '\n')
        stream.flush()

    def _on_tracklets(self, message):
        batch_stamp_ns = stamp_ns(message.header.stamp)
        tracklets = []
        for tracklet in message.tracklets:
            measurement_stamp_ns = stamp_ns(tracklet.last_measurement_stamp)
            key = (int(message.source_epoch_id), int(tracklet.tracklet_id))
            history = self._tracklet_stamps.setdefault(key, deque(maxlen=256))
            if not history or history[-1] != measurement_stamp_ns:
                history.append(measurement_stamp_ns)
            tracklets.append({
                'tracklet_id': int(tracklet.tracklet_id),
                'measurement_stamp_ns': measurement_stamp_ns,
                'position': [
                    float(tracklet.position.x),
                    float(tracklet.position.y),
                    float(tracklet.position.z),
                ],
                'size': [
                    float(tracklet.size.x),
                    float(tracklet.size.y),
                    float(tracklet.size.z),
                ],
                'confidence': float(tracklet.confidence),
                'active': bool(tracklet.active),
            })
        self._write(self._context_stream, {
            'kind': 'lidar_batch',
            'batch_stamp_ns': batch_stamp_ns,
            'frame_id': message.header.frame_id,
            'lidar_source_epoch_id': int(message.source_epoch_id),
            'tracklets': tracklets,
        })

    def _on_observations(self, message):
        visual_stamp_ns = stamp_ns(message.header.stamp)
        for observation in message.observations:
            self._write(self._context_stream, {
                'kind': 'visual_observation',
                'visual_stamp_ns': visual_stamp_ns,
                'frame_id': observation.header.frame_id,
                'observation_producer_epoch_id': int(
                    observation.producer_epoch_id),
                'visual_candidate_id': int(observation.visual_candidate_id),
                'roi': [
                    int(observation.roi.x_offset),
                    int(observation.roi.y_offset),
                    int(observation.roi.width),
                    int(observation.roi.height),
                ],
                'image_size': [
                    int(observation.image_width),
                    int(observation.image_height),
                ],
                'labels': [item.label for item in observation.semantic_labels],
            })

    def _on_debug(self, message):
        key = (int(message.lidar_source_epoch_id), int(message.lidar_tracklet_id))
        visual_stamp_ns = stamp_ns(message.header.stamp)
        lidar_stamp_history = self._tracklet_stamps.get(key)
        if not lidar_stamp_history:
            self.get_logger().warning(
                'Dropping debug pair without captured LiDAR source stamp')
            return
        lidar_stamp_ns = min(
            lidar_stamp_history,
            key=lambda value: (abs(value - visual_stamp_ns), value))
        pair_id = '{}:{}:{}:{}:{}'.format(
            int(message.memory_epoch_id),
            int(message.observation_producer_epoch_id),
            int(message.visual_candidate_id),
            int(message.lidar_source_epoch_id),
            int(message.lidar_tracklet_id),
        )
        if pair_id in self._pair_ids:
            self.get_logger().warning('Dropping duplicate pair {}'.format(pair_id))
            return
        self._pair_ids.add(pair_id)
        terms = []
        for term in message.terms:
            terms.append({
                'name': term.name,
                'valid': bool(term.valid),
                'hard_gate': bool(term.hard_gate),
                'gate_passed': bool(term.gate_passed),
                'raw_value': finite_or_none(term.raw_value),
                'normalized_value': finite_or_none(term.normalized_value),
                'weight': float(term.weight),
                'contribution': finite_or_none(term.contribution),
            })
        self._write(self._debug_stream, {
            'schema_version': '1.0.0',
            'pair_id': pair_id,
            'visual_stamp_ns': visual_stamp_ns,
            'lidar_stamp_ns': lidar_stamp_ns,
            'memory_epoch_id': int(message.memory_epoch_id),
            'observation_producer_epoch_id': int(
                message.observation_producer_epoch_id),
            'visual_candidate_id': int(message.visual_candidate_id),
            'lidar_source_epoch_id': int(message.lidar_source_epoch_id),
            'lidar_tracklet_id': int(message.lidar_tracklet_id),
            'decision': DECISION_NAMES[int(message.decision)],
            'total_score': float(message.total_score),
            'terms': terms,
        })

    def close(self):
        self._debug_stream.close()
        self._context_stream.close()


def main():
    parser = argparse.ArgumentParser(
        description='Capture Phase 2 association shadow evidence as JSONL.')
    parser.add_argument('--debug-jsonl', required=True, type=Path)
    parser.add_argument('--context-jsonl', required=True, type=Path)
    arguments = parser.parse_args()

    rclpy.init()
    node = CaptureNode(arguments.debug_jsonl, arguments.context_jsonl)
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
