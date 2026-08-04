"""Passive ROS 2 worker for Phase 1R YOLO-World perception."""

import json
import math
from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, RegionOfInterest
from std_msgs.msg import String
from track_robot_interfaces.msg import (
    SemanticLabelEvidence,
    SemanticObservation,
    SemanticObservationArray,
    SemanticRegion,
    SemanticRegionArray,
    SemanticTask,
)

from .camera_tracking import CameraTrackManager, CameraTrackingConfig
from .dino_crop_descriptors import (
    DinoCropConfig,
    DinoCropDescriptorBackend,
)
from .query_transport import parse_query_payload
from .visual_candidates import build_visual_descriptor_message
from .yolo_world_backend import YoloWorldBackend
from .yolo_world_perception_core import YoloWorldPerceptionCore


class YoloWorldPerceptionNode(Node):
    """Convert an English text query and camera frames into passive evidence."""

    def __init__(self):
        super().__init__('semantic_search_yolo_world_perception')
        self._bridge = CvBridge()
        self._latest_image = None
        self._latest_image_key = None
        self._last_processed_image_key = None
        self._processing = False
        self._core = None
        self._published_task_key = None
        self._model_error = ''

        self._calibration_id = str(self.declare_parameter(
            'calibration_id', 'zed_left_rectified_v1').value)
        if not 1 <= len(self._calibration_id) <= 128:
            raise ValueError('calibration_id must contain 1 to 128 characters')
        self._model_input_size = int(self.declare_parameter(
            'input_size', 640).value)
        self._max_detections = int(self.declare_parameter(
            'max_detections', 64).value)
        self._descriptor_version = int(self.declare_parameter(
            'task_descriptor_version', 1).value)
        self._target_rate_hz = float(self.declare_parameter(
            'target_rate_hz', 2.0).value)
        if (
                not math.isfinite(self._target_rate_hz) or
                not 0.1 <= self._target_rate_hz <= 30.0):
            raise ValueError('target_rate_hz must be in [0.1, 30]')

        self._regions_publisher = self.create_publisher(
            SemanticRegionArray,
            str(self.declare_parameter(
                'regions_topic', '/semantic_search/regions').value),
            5)
        self._observations_publisher = self.create_publisher(
            SemanticObservationArray,
            str(self.declare_parameter(
                'observations_topic',
                '/semantic_memory/observations').value),
            5)
        self._task_publisher = self.create_publisher(
            SemanticTask,
            str(self.declare_parameter(
                'task_topic', '/semantic_memory/tasks').value),
            5)
        self._diagnostics_publisher = self.create_publisher(
            String,
            str(self.declare_parameter(
                'diagnostics_topic',
                '/semantic_search/perception_diagnostics').value),
            5)

        self._image_subscription = self.create_subscription(
            Image,
            str(self.declare_parameter(
                'image_topic',
                '/zed/zed_node/left/image_rect_color').value),
            self._image_callback,
            qos_profile_sensor_data)
        self._query_subscription = self.create_subscription(
            String,
            str(self.declare_parameter(
                'query_topic', '/semantic_search/query').value),
            self._query_callback,
            5)

        self._load_core()
        self._processing_timer = self.create_timer(
            1.0 / self._target_rate_hz, self._process_latest)

    @staticmethod
    def _required_local_path(value, name, directory=False):
        path = Path(str(value)).expanduser()
        valid = path.is_dir() if directory else path.is_file()
        if not str(value) or path.is_symlink() or not valid:
            kind = 'directory' if directory else 'file'
            raise ValueError('{} must be a local {}'.format(name, kind))
        return path

    def _load_core(self):
        try:
            runtime_path = self._required_local_path(
                self.declare_parameter('runtime_path', '').value,
                'runtime_path',
                directory=True)
            clip_runtime_path = self._required_local_path(
                self.declare_parameter('clip_runtime_path', '').value,
                'clip_runtime_path',
                directory=True)
            world_checkpoint = self._required_local_path(
                self.declare_parameter('world_checkpoint', '').value,
                'world_checkpoint')
            clip_checkpoint = self._required_local_path(
                self.declare_parameter('clip_checkpoint', '').value,
                'clip_checkpoint')
            device = int(self.declare_parameter('device', 0).value)
            backend = YoloWorldBackend.from_local_model(
                runtime_path=runtime_path,
                clip_runtime_path=clip_runtime_path,
                world_checkpoint=world_checkpoint,
                clip_checkpoint=clip_checkpoint,
                confidence_floor=float(self.declare_parameter(
                    'confidence_floor', 0.05).value),
                iou_threshold=float(self.declare_parameter(
                    'iou_threshold', 0.70).value),
                input_size=self._model_input_size,
                max_detections=self._max_detections,
                device=device,
                half=bool(self.declare_parameter('half', True).value),
            )
            dino = self._load_dino(device)
            tracker = CameraTrackManager(CameraTrackingConfig(
                minimum_iou=float(self.declare_parameter(
                    'tracking_minimum_iou', 0.20).value),
                maximum_normalized_center_distance=float(
                    self.declare_parameter(
                        'tracking_maximum_center_distance', 0.30).value),
                ambiguity_margin=float(self.declare_parameter(
                    'tracking_ambiguity_margin', 0.05).value),
                minimum_appearance_similarity=float(self.declare_parameter(
                    'tracking_minimum_appearance_similarity', 0.80).value),
                maximum_missed_frames=int(self.declare_parameter(
                    'tracking_maximum_missed_frames', 8).value),
                maximum_tracks=self._max_detections,
            ))
            self._core = YoloWorldPerceptionCore(
                backend=backend,
                dino_backend=dino,
                tracker=tracker,
            )
            self._publish_diagnostics(
                'ready',
                'local_pretrained_models_loaded',
                appearance_available=dino.available,
                appearance_reason=dino.unavailable_reason,
            )
        except (ImportError, OSError, RuntimeError, TypeError, ValueError) \
                as error:
            self._model_error = '{}: {}'.format(
                type(error).__name__, str(error))
            self._publish_diagnostics(
                'not_ready', 'model_initialization_failed',
                error=self._model_error)

    def _load_dino(self, device):
        enabled = bool(self.declare_parameter('dino_enabled', True).value)
        if not enabled:
            return DinoCropDescriptorBackend.disabled(
                'disabled_by_configuration')
        try:
            local_repo = self._required_local_path(
                self.declare_parameter('dino_local_repo', '').value,
                'dino_local_repo',
                directory=True)
            checkpoint = self._required_local_path(
                self.declare_parameter('dino_checkpoint', '').value,
                'dino_checkpoint')
            config = DinoCropConfig(
                input_size=int(self.declare_parameter(
                    'dino_input_size', 224).value),
                context_margin=float(self.declare_parameter(
                    'dino_context_margin', 0.10).value),
                maximum_crops=int(self.declare_parameter(
                    'dino_maximum_crops', 3).value),
                encoder_id=str(self.declare_parameter(
                    'dino_encoder_id', 'dinov3:vits16plus').value),
                checkpoint_id=checkpoint.name,
                descriptor_version=int(self.declare_parameter(
                    'dino_descriptor_version', 1).value),
            )
            return DinoCropDescriptorBackend.from_local_model(
                local_repo=local_repo,
                weights_path=checkpoint,
                device='cuda:{}'.format(device),
                config=config,
            )
        except (ImportError, OSError, RuntimeError, TypeError, ValueError) \
                as error:
            reason = 'dino_initialization_failed:{}:{}'.format(
                type(error).__name__, str(error))
            self.get_logger().warning(
                'DINO appearance evidence disabled: %s' % reason)
            return DinoCropDescriptorBackend.disabled(reason)

    @staticmethod
    def _image_key(message):
        return (
            int(message.header.stamp.sec),
            int(message.header.stamp.nanosec),
        )

    @staticmethod
    def _stamp_ns(message):
        return (
            int(message.header.stamp.sec) * 1_000_000_000 +
            int(message.header.stamp.nanosec)
        )

    def _image_callback(self, message):
        self._latest_image = message
        self._latest_image_key = self._image_key(message)

    def _query_callback(self, message):
        if self._core is None:
            self._publish_diagnostics(
                'not_ready', 'query_rejected_model_unavailable',
                error=self._model_error)
            return
        try:
            query = parse_query_payload(message.data)
            self._core.accept_query(query)
            self._published_task_key = None
            self._publish_diagnostics(
                'ready',
                'query_accepted',
                query_id=query.query_id,
                query_version=query.query_version,
                query_text=query.query_text,
            )
        except (TypeError, ValueError) as error:
            self._publish_diagnostics(
                'degraded', 'query_rejected', error=str(error))

    def _process_latest(self):
        if (
                self._processing or self._core is None or
                self._latest_image is None or
                self._latest_image_key == self._last_processed_image_key):
            return
        self._processing = True
        message = self._latest_image
        key = self._latest_image_key
        try:
            image_bgr = self._bridge.imgmsg_to_cv2(
                message, desired_encoding='bgr8')
            result = self._core.process(image_bgr, self._stamp_ns(message))
            self._last_processed_image_key = key
            if result is None:
                self._publish_diagnostics(
                    'waiting', 'no_active_query')
                return
            self._publish_regions(message, result, image_bgr.shape)
            self._publish_observations(message, result, image_bgr.shape)
            self._publish_task(message, result)
            self._publish_diagnostics(
                'active',
                'frame_processed',
                query_id=result.query_id,
                query_version=result.query_version,
                candidate_count=len(result.candidates),
                producer_epoch_id=result.producer_epoch_id,
                model_latency_ms=result.model_latency_ms,
                appearance_latency_ms=result.appearance_latency_ms,
                appearance_available=result.appearance_available,
                appearance_reason=result.appearance_reason,
                rollback_count=result.rollback_count,
            )
        except (ImportError, OSError, RuntimeError, TypeError, ValueError) \
                as error:
            self._last_processed_image_key = key
            self._publish_diagnostics(
                'degraded',
                'frame_processing_failed',
                error='{}: {}'.format(type(error).__name__, str(error)))
        finally:
            self._processing = False

    def _publish_regions(self, image_message, result, image_shape):
        output = SemanticRegionArray()
        output.header = image_message.header
        output.query_id = result.query_id
        output.query_version = result.query_version
        height, width = image_shape[:2]
        for candidate in result.candidates:
            detection = candidate.detection
            region = SemanticRegion()
            region.header = image_message.header
            region.image_width = width
            region.image_height = height
            region.query_id = result.query_id
            region.query_version = result.query_version
            region.observation_id = candidate.candidate_id
            region.roi = self._roi(detection)
            region.mask_encoding = SemanticRegion.MASK_NONE
            region.compressed_mask = []
            region.appearance_score = (
                candidate.descriptor.quality
                if candidate.descriptor is not None else 0.0)
            region.language_score = detection.score
            region.localization_score = detection.score
            region.fused_score = detection.score
            region.preprocessing_scale = 1.0
            region.padding_left = 0
            region.padding_top = 0
            region.model_input_width = self._model_input_size
            region.model_input_height = self._model_input_size
            region.image_encoder_id = 'yolov8s-worldv2'
            region.text_encoder_id = 'openai_clip:ViT-B/32'
            region.checkpoint_id = 'yolov8s-worldv2.pt'
            output.regions.append(region)
        self._regions_publisher.publish(output)

    def _publish_observations(self, image_message, result, image_shape):
        output = SemanticObservationArray()
        output.header = image_message.header
        output.producer_epoch_id = result.producer_epoch_id
        height, width = image_shape[:2]
        for candidate in result.candidates:
            detection = candidate.detection
            observation = SemanticObservation()
            observation.header = image_message.header
            observation.producer_epoch_id = result.producer_epoch_id
            observation.observation_id = candidate.candidate_id
            observation.visual_candidate_id = candidate.candidate_id
            observation.camera_track_id_valid = True
            observation.camera_track_id = candidate.camera_track_id
            observation.query_id = result.query_id
            observation.query_version = result.query_version
            observation.camera_stamp_valid = True
            observation.camera_stamp = image_message.header.stamp
            observation.image_width = width
            observation.image_height = height
            observation.roi = self._roi(detection)
            observation.mask_encoding = SemanticObservation.MASK_NONE
            observation.compressed_mask = []
            observation.proposal_source = (
                SemanticObservation.PROPOSAL_OPEN_VOCABULARY)
            observation.detector_confidence = detection.score
            observation.appearance_confidence = (
                candidate.descriptor.quality
                if candidate.descriptor is not None else 0.0)
            observation.language_relevance = detection.score
            observation.geometry_confidence = 0.0
            observation.overall_confidence = detection.score
            observation.position_valid = False
            observation.velocity_valid = False
            observation.extent_valid = False
            observation.memory_mode = (
                SemanticObservation.MEMORY_OBSERVATION_ONLY)
            observation.calibration_id = self._calibration_id
            observation.evidence_flags = SemanticObservation.EVIDENCE_CAMERA
            if candidate.descriptor is not None:
                descriptor = candidate.descriptor
                observation.appearance_descriptor = (
                    build_visual_descriptor_message(
                        descriptor.values,
                        descriptor.quality,
                        descriptor.encoder_id,
                        descriptor.checkpoint_id,
                        descriptor.version,
                    ))
            observation.semantic_labels.append(SemanticLabelEvidence(
                label=result.query_text,
                confidence=detection.score,
                provenance='yolov8s-worldv2',
                evidence_kind=(
                    SemanticLabelEvidence.EVIDENCE_TASK_CONDITIONED),
                source_observation_id=candidate.candidate_id,
            ))
            output.observations.append(observation)
        self._observations_publisher.publish(output)

    def _publish_task(self, image_message, result):
        key = (
            result.producer_epoch_id,
            result.query_id,
            result.query_version,
        )
        if key == self._published_task_key:
            return
        output = SemanticTask()
        output.header = image_message.header
        output.producer_epoch_id = result.producer_epoch_id
        output.query_id = result.query_id
        output.query_version = result.query_version
        output.query_text = result.query_text
        output.task_descriptor = build_visual_descriptor_message(
            result.task_descriptor,
            1.0,
            'openai_clip:ViT-B/32',
            'ViT-B-32.pt',
            self._descriptor_version,
        )
        self._task_publisher.publish(output)
        self._published_task_key = key

    @staticmethod
    def _roi(detection):
        left = max(0, int(math.floor(detection.x1)))
        top = max(0, int(math.floor(detection.y1)))
        right = max(left + 1, int(math.ceil(detection.x2)))
        bottom = max(top + 1, int(math.ceil(detection.y2)))
        return RegionOfInterest(
            x_offset=left,
            y_offset=top,
            width=right - left,
            height=bottom - top,
            do_rectify=False,
        )

    def _publish_diagnostics(self, state, reason, **metrics):
        payload = {
            'schema_version': 'phase1r_runtime/1.0.0',
            'state': state,
            'reason': reason,
            'model_ready': self._core is not None,
            'motion_output': False,
        }
        payload.update(metrics)
        self._diagnostics_publisher.publish(String(
            data=json.dumps(payload, sort_keys=True)))


def main(args=None):
    rclpy.init(args=args)
    node = YoloWorldPerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
