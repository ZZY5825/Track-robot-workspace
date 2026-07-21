from track_robot_interfaces.msg import (
    AssociationDebug,
    AssociationTerm,
    LidarTracklet,
    SemanticLabelEvidence,
    SemanticLidarTrackletArray,
    SemanticLocalizationState,
    SemanticMemoryEvent,
    SemanticObject,
    SemanticObjectArray,
    SemanticObjectHistorySample,
    SemanticObservation,
    SemanticObservationArray,
    SemanticTask,
    VisualDescriptor,
    VisualProposal,
    VisualProposalArray,
)
from track_robot_interfaces.srv import (
    GetSemanticObject,
    MarkSemanticObjectInspected,
    QuerySemanticObjects,
    ResetSemanticMemory,
)


def descriptor(dimension=512):
    return VisualDescriptor(
        encoder_id='openai_clip:ViT-B/32',
        checkpoint_id='clip-vit-b32.pt',
        version=1,
        dimension=dimension,
        l2_normalized=True,
        quality=0.8,
        values=[0.0] * dimension,
    )


def test_phase2_messages_expose_separate_ids_and_bounded_payload_shapes():
    proposal = VisualProposal(
        producer_epoch_id=7,
        proposal_id=11,
        proposal_source=VisualProposal.PROPOSAL_EXTERNAL_DETECTOR,
        detector_confidence=0.9,
    )
    proposals = VisualProposalArray(
        producer_epoch_id=7,
        proposals=[proposal] * 64,
    )
    observation = SemanticObservation(
        producer_epoch_id=13,
        observation_id=17,
        visual_candidate_id=19,
        camera_track_id_valid=True,
        camera_track_id=23,
        lidar_tracklet_id_valid=True,
        lidar_source_epoch_id=29,
        lidar_tracklet_id=31,
        appearance_descriptor=descriptor(),
    )
    observations = SemanticObservationArray(
        producer_epoch_id=13,
        observations=[observation] * 64,
    )
    lidar = SemanticLidarTrackletArray(
        source_epoch_id=37,
        tracklets=[LidarTracklet(tracklet_id=index) for index in range(256)],
    )

    assert len(proposals.proposals) == 64
    assert len(observations.observations) == 64
    assert len(lidar.tracklets) == 256
    assert observation.visual_candidate_id != observation.camera_track_id
    assert observation.camera_track_id != observation.lidar_tracklet_id
    assert not hasattr(observation, 'global_object_id')
    assert len(observation.appearance_descriptor.values) == 512


def test_semantic_object_uses_public_key_and_orthogonal_states():
    sample = SemanticObjectHistorySample(
        uncertainty=0.2,
        lifecycle_state=SemanticObject.LIFECYCLE_CONFIRMED,
        support_state=SemanticObject.SUPPORT_CAMERA_LIDAR,
    )
    obj = SemanticObject(
        memory_epoch_id=41,
        global_object_id=43,
        memory_mode=SemanticObject.MEMORY_LOCAL_SESSION,
        lifecycle_state=SemanticObject.LIFECYCLE_CONFIRMED,
        support_state=SemanticObject.SUPPORT_LIDAR_ONLY,
        visibility_state=SemanticObject.VISIBILITY_OUT_OF_FOV,
        motion_class=SemanticObject.MOTION_STATIC,
        short_history=[sample] * 16,
        semantic_labels=[SemanticLabelEvidence(label='branch')] * 16,
    )
    snapshot = SemanticObjectArray(
        memory_epoch_id=41,
        objects=[obj] * 256,
    )

    assert (obj.memory_epoch_id, obj.global_object_id) == (41, 43)
    assert obj.lifecycle_state == SemanticObject.LIFECYCLE_CONFIRMED
    assert obj.support_state == SemanticObject.SUPPORT_LIDAR_ONLY
    assert obj.visibility_state == SemanticObject.VISIBILITY_OUT_OF_FOV
    assert obj.motion_class == SemanticObject.MOTION_STATIC
    assert len(obj.short_history) == 16
    assert len(snapshot.objects) == 256


def test_phase2_task_localization_debug_events_and_services_construct():
    localization = SemanticLocalizationState(
        memory_mode=SemanticLocalizationState.MEMORY_WORLD,
        localization_epoch_id=47,
        canonical_frame_id='map',
        world_healthy=True,
    )
    task = SemanticTask(
        producer_epoch_id=53,
        query_id=59,
        query_version=61,
        query_text='find a fallen branch blocking the path',
        task_descriptor=descriptor(),
    )
    term = AssociationTerm(
        name='position_nis',
        valid=True,
        hard_gate=True,
        gate_passed=True,
        normalized_value=0.9,
    )
    debug = AssociationDebug(
        decision=AssociationDebug.DECISION_MATCHED,
        terms=[term] * 24,
    )
    event = SemanticMemoryEvent(
        event_type=SemanticMemoryEvent.EVENT_INSPECTION_CHANGED,
        memory_epoch_id=67,
        global_object_id=71,
    )

    assert localization.memory_mode == SemanticLocalizationState.MEMORY_WORLD
    assert task.query_id != event.global_object_id
    assert event.event_type == SemanticMemoryEvent.EVENT_INSPECTION_CHANGED
    assert len(debug.terms) == 24
    assert GetSemanticObject.Request(memory_epoch_id=67, global_object_id=71)
    assert QuerySemanticObjects.Request(page_size=64)
    assert MarkSemanticObjectInspected.Request(
        memory_epoch_id=67, global_object_id=71)
    assert ResetSemanticMemory.Request(
        expected_memory_epoch_id=67, require_epoch_match=True)
