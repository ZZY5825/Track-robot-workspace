import json

import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, RegionOfInterest
from std_msgs.msg import String
from track_robot_interfaces.msg import (
    SemanticLabelEvidence,
    SemanticObservation,
    SemanticObservationArray,
    SemanticRegion,
    SemanticRegionArray,
    SemanticTask,
    VisualProposalArray,
)

from .model_adapters import ModelUnavailableError, create_aligned_encoder
from .perception_core import PassivePerceptionCore, SourceTimestampScheduler
from .query_transport import parse_query_payload
from .visual_candidates import (
    CandidateLabel,
    CandidateProposal,
    ProducerIdentity,
    build_visual_descriptor_message,
)


class SemanticPerceptionNode(Node):
    def __init__(self):
        super().__init__('semantic_search_perception')
        self._bridge = CvBridge()
        self._latest_image = None
        self._active_query = None
        self._latest_proposals = None
        self._processing = False
        self._core = None
        self._published_task_key = None
        self._image_message_count = 0
        self._processing_timer_started = False

        self._producer_identity = ProducerIdentity(int(
            self.declare_parameter('producer_epoch_seed', 0).value))
        self._descriptor_version = int(self.declare_parameter(
            'descriptor_version', 1).value)
        self._calibration_id = str(self.declare_parameter(
            'calibration_id', 'zed_left_rectified_v1').value)
        if not self._calibration_id or len(self._calibration_id) > 128:
            raise ValueError(
                'calibration_id must contain between 1 and 128 characters')

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
                'image_topic', '/zed/zed_node/left/image_rect_color').value),
            self._image_callback,
            1)
        self._query_subscription = self.create_subscription(
            String,
            str(self.declare_parameter(
                'query_topic', '/semantic_search/query').value),
            self._query_callback,
            1)
        self._proposal_subscription = self.create_subscription(
            VisualProposalArray,
            str(self.declare_parameter(
                'proposal_topic',
                '/semantic_memory/visual_proposals').value),
            self._proposal_callback,
            1)

        target_rate_hz = float(self.declare_parameter(
            'target_rate_hz', 5.0).value)
        self._scheduler = SourceTimestampScheduler(target_rate_hz)
        self._load_model()
        self._processing_timer = self.create_timer(0.01, self._process_latest)

    def _load_model(self):
        implementation = str(self.declare_parameter(
            'adapter_implementation', 'openai_clip').value)
        model_name = str(self.declare_parameter(
            'model_name', 'ViT-B/32').value)
        checkpoint_path = str(self.declare_parameter(
            'checkpoint_path', '').value)
        runtime_path = str(self.declare_parameter(
            'runtime_path', '').value)
        requested_device = str(self.declare_parameter(
            'device', 'auto').value)
        device = (
            'cuda' if requested_device == 'auto' and torch.cuda.is_available()
            else 'cpu' if requested_device == 'auto'
            else requested_device)
        grid_size = int(self.declare_parameter('grid_size', 4).value)
        threshold = float(self.declare_parameter('threshold', 0.25).value)
        threshold_mode = str(self.declare_parameter(
            'threshold_mode', 'absolute').value)
        quantile = float(self.declare_parameter('quantile', 0.90).value)
        min_area = int(self.declare_parameter('min_area', 1).value)
        max_regions = int(self.declare_parameter('max_regions', 10).value)
        max_visual_candidates = int(self.declare_parameter(
            'max_visual_candidates', 64).value)
        try:
            encoder = create_aligned_encoder(
                implementation=implementation,
                model_name=model_name,
                checkpoint_path=checkpoint_path,
                runtime_path=runtime_path,
                device=device,
                grid_size=grid_size)
            encoder.encode_text('object')
            encoder.encode_image_grid(np.zeros((224, 224, 3), dtype=np.uint8))
            self._core = PassivePerceptionCore(
                encoder,
                threshold=threshold,
                threshold_mode=threshold_mode,
                quantile=quantile,
                min_area=min_area,
                max_regions=max_regions,
                max_visual_candidates=max_visual_candidates)
        except (ModelUnavailableError, RuntimeError, ValueError) as exc:
            self._core = None
            self._publish_diagnostics('not_ready', str(exc))
            self.get_logger().error('Semantic model unavailable: {}'.format(exc))
            return
        self._publish_diagnostics('ready', 'model loaded and warmed')
        self.get_logger().info(
            'Passive semantic perception ready: {} on {}'.format(
                encoder.encoder_id, device))

    def _image_callback(self, message):
        self._image_message_count += 1
        self._latest_image = message
        if self._image_message_count == 1 or self._image_message_count % 50 == 0:
            self._publish_diagnostics(
                'image_received',
                'image stream active',
                image_message_count=self._image_message_count,
                image_stamp_ns=self._stamp_ns(message.header.stamp),
                image_frame=message.header.frame_id)

    def _query_callback(self, message):
        try:
            query = parse_query_payload(message.data)
            if len(query.query_text) > 512:
                raise ValueError('query_text exceeds the 512 character bound')
            self._active_query = query
            self._publish_diagnostics(
                'query_accepted',
                'semantic query activated',
                query_id=query.query_id,
                query_version=query.query_version,
                image_message_count=self._image_message_count,
                processing_timer_started=self._processing_timer_started)
        except ValueError as exc:
            self._publish_diagnostics('query_rejected', str(exc))
            self.get_logger().warning('Rejected semantic query: {}'.format(exc))

    def _proposal_callback(self, message):
        self._latest_proposals = message

    @staticmethod
    def _stamp_ns(stamp):
        return int(stamp.sec) * 1000000000 + int(stamp.nanosec)

    def _matching_proposals(self, image_stamp_ns):
        message = self._latest_proposals
        if message is None:
            return []
        proposal_stamp_ns = self._stamp_ns(message.header.stamp)
        if proposal_stamp_ns < image_stamp_ns:
            self._latest_proposals = None
            return []
        if proposal_stamp_ns != image_stamp_ns:
            return []
        self._latest_proposals = None
        output = []
        for proposal in message.proposals:
            labels = tuple(CandidateLabel(
                label=item.label,
                confidence=float(item.confidence),
                provenance=item.provenance,
                evidence_kind=int(item.evidence_kind),
            ) for item in proposal.semantic_labels)
            output.append(CandidateProposal(
                producer_epoch_id=int(proposal.producer_epoch_id),
                proposal_id=int(proposal.proposal_id),
                roi=(
                    int(proposal.roi.x_offset),
                    int(proposal.roi.y_offset),
                    int(proposal.roi.width),
                    int(proposal.roi.height),
                ),
                proposal_source=int(proposal.proposal_source),
                detector_confidence=float(proposal.detector_confidence),
                mask_encoding=int(proposal.mask_encoding),
                compressed_mask=bytes(proposal.compressed_mask),
                labels=labels,
            ))
        return output

    def _process_latest(self):
        if not self._processing_timer_started:
            self._processing_timer_started = True
            self._publish_diagnostics(
                'processing_timer_started',
                'perception processing timer active')
        if self._processing or self._core is None:
            return
        image_message = self._latest_image
        query = self._active_query
        if image_message is None:
            return
        stamp_ns = self._stamp_ns(image_message.header.stamp)
        proposals = self._matching_proposals(stamp_ns)
        if query is None and not proposals:
            return
        previous_rollback_count = self._scheduler.rollback_count
        if not self._scheduler.should_process(stamp_ns):
            if self._scheduler.rollback_count != previous_rollback_count:
                self._producer_identity.advance_epoch()
                self._core.start_producer_epoch()
                self._published_task_key = None
            return
        self._latest_image = None
        self._processing = True
        try:
            image_bgr = self._bridge.imgmsg_to_cv2(
                image_message, desired_encoding='bgr8')
            result = self._core.process_frame(
                image_bgr=image_bgr,
                stamp_ns=stamp_ns,
                query_text=(query.query_text if query is not None else None),
                query_id=(query.query_id if query is not None else 0),
                query_version=(query.query_version if query is not None else 0),
                proposals=proposals)
            self._publish_result(image_message, result)
            self._publish_diagnostics(
                'ready',
                'observation published',
                observation_id=result.observation_id,
                region_count=len(result.regions),
                visual_candidate_count=len(result.visual_candidates),
                inference_ms=result.inference_ms)
        except Exception as exc:
            self._core = None
            self._publish_diagnostics('fault', str(exc))
            self.get_logger().error(
                'Semantic inference stopped after fault: {}'.format(exc))
        finally:
            self._processing = False

    def _publish_result(self, image_message, result):
        output = SemanticRegionArray()
        output.header = image_message.header
        output.query_id = result.query_id
        output.query_version = result.query_version
        for candidate in result.regions:
            region = SemanticRegion()
            region.header = image_message.header
            region.image_width = result.image_width
            region.image_height = result.image_height
            region.query_id = result.query_id
            region.query_version = result.query_version
            region.observation_id = result.observation_id
            region.roi = RegionOfInterest(
                x_offset=candidate.x,
                y_offset=candidate.y,
                height=candidate.height,
                width=candidate.width,
                do_rectify=False)
            region.mask_encoding = SemanticRegion.MASK_NONE
            region.compressed_mask = []
            region.appearance_score = 0.0
            region.language_score = candidate.score
            region.localization_score = candidate.peak_score
            region.fused_score = candidate.score
            region.preprocessing_scale = result.preprocessing_scale
            region.padding_left = result.padding_left
            region.padding_top = result.padding_top
            region.model_input_width = result.model_input_width
            region.model_input_height = result.model_input_height
            region.image_encoder_id = result.image_encoder_id
            region.text_encoder_id = result.text_encoder_id
            region.checkpoint_id = result.checkpoint_id
            output.regions.append(region)
        self._regions_publisher.publish(output)
        self._publish_observations(image_message, result)
        self._publish_task(image_message, result)

    def _publish_observations(self, image_message, result):
        output = SemanticObservationArray()
        output.header = image_message.header
        output.producer_epoch_id = self._producer_identity.epoch_id
        for candidate in result.visual_candidates:
            observation = SemanticObservation()
            observation.header = image_message.header
            observation.producer_epoch_id = self._producer_identity.epoch_id
            observation.observation_id = result.observation_id
            observation.visual_candidate_id = candidate.visual_candidate_id
            observation.upstream_proposal_id_valid = (
                candidate.upstream_proposal_id_valid)
            observation.upstream_producer_epoch_id = (
                candidate.upstream_producer_epoch_id)
            observation.upstream_proposal_id = candidate.upstream_proposal_id
            observation.query_id = result.query_id
            observation.query_version = result.query_version
            observation.camera_stamp_valid = True
            observation.camera_stamp = image_message.header.stamp
            observation.image_width = result.image_width
            observation.image_height = result.image_height
            observation.roi = RegionOfInterest(
                x_offset=candidate.roi[0],
                y_offset=candidate.roi[1],
                width=candidate.roi[2],
                height=candidate.roi[3],
                do_rectify=False)
            observation.mask_encoding = candidate.mask_encoding
            observation.compressed_mask = list(candidate.compressed_mask)
            observation.proposal_source = candidate.proposal_source
            observation.detector_confidence = candidate.detector_confidence
            observation.appearance_confidence = candidate.descriptor_quality
            observation.language_relevance = candidate.language_score
            observation.geometry_confidence = 0.0
            observation.overall_confidence = max(
                candidate.detector_confidence,
                candidate.language_score)
            observation.position_valid = False
            observation.velocity_valid = False
            observation.extent_valid = False
            observation.memory_mode = SemanticObservation.MEMORY_OBSERVATION_ONLY
            observation.calibration_id = self._calibration_id
            observation.evidence_flags = SemanticObservation.EVIDENCE_CAMERA
            observation.appearance_descriptor = build_visual_descriptor_message(
                candidate.descriptor,
                candidate.descriptor_quality,
                result.image_encoder_id,
                result.checkpoint_id,
                self._descriptor_version)
            for item in candidate.labels:
                observation.semantic_labels.append(SemanticLabelEvidence(
                    label=item.label,
                    confidence=item.confidence,
                    provenance=item.provenance,
                    evidence_kind=item.evidence_kind,
                    source_observation_id=result.observation_id,
                ))
            output.observations.append(observation)
        self._observations_publisher.publish(output)

    def _publish_task(self, image_message, result):
        if not result.normalized_query_text or result.task_descriptor.size == 0:
            return
        key = (
            self._producer_identity.epoch_id,
            result.query_id,
            result.query_version,
        )
        if key == self._published_task_key:
            return
        output = SemanticTask()
        output.header = image_message.header
        output.producer_epoch_id = self._producer_identity.epoch_id
        output.query_id = result.query_id
        output.query_version = result.query_version
        output.query_text = result.normalized_query_text
        output.task_descriptor = build_visual_descriptor_message(
            result.task_descriptor,
            1.0,
            result.text_encoder_id,
            result.checkpoint_id,
            self._descriptor_version)
        self._task_publisher.publish(output)
        self._published_task_key = key

    def _publish_diagnostics(self, state, reason, **metrics):
        payload = {
            'state': state,
            'reason': reason,
            'model_ready': self._core is not None,
            'rollback_count': getattr(self, '_scheduler', None).rollback_count
            if hasattr(self, '_scheduler') else 0,
        }
        payload.update(metrics)
        self._diagnostics_publisher.publish(
            String(data=json.dumps(payload, sort_keys=True)))


def main(args=None):
    rclpy.init(args=args)
    node = SemanticPerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
