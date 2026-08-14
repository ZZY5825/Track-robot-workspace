from pathlib import Path

from rclpy.serialization import deserialize_message, serialize_message

from track_robot_interfaces.action import SearchForObject
from track_robot_interfaces.msg import (
    ObjectObservation3D,
    ObjectObservation3DArray,
    SearchMotionIntent,
    SemanticRegion,
    SemanticRegionArray,
    TrackedSemanticObject,
    TrackedSemanticObjectArray,
)


def round_trip(message):
    return deserialize_message(serialize_message(message), type(message))


def test_action_constants_and_safe_defaults():
    goal = SearchForObject.Goal()
    result = SearchForObject.Result()
    feedback = SearchForObject.Feedback()
    assert goal.allow_rotation is False
    assert goal.maximum_rotation_angle == 0.0
    assert result.CONFIRMED == 0
    assert result.TIMEOUT == 7
    assert feedback.INITIALISING == 0
    assert feedback.TERMINAL == 4


def test_semantic_messages_round_trip_without_tensor_fields():
    region = SemanticRegion(query_id=1, query_version=1, observation_id=1)
    observation = ObjectObservation3D(
        query_id=1, query_version=1, observation_id=1, region=region)
    tracked = TrackedSemanticObject(global_object_id=1)
    intent = SearchMotionIntent(query_id=1)
    for message in (
            region,
            SemanticRegionArray(query_id=1, query_version=1, regions=[region]),
            observation,
            ObjectObservation3DArray(
                query_id=1, query_version=1, observations=[observation]),
            tracked,
            TrackedSemanticObjectArray(objects=[tracked]),
            intent):
        restored = round_trip(message)
        fields = restored.get_fields_and_field_types()
        assert not any(
            forbidden in name
            for name in fields
            for forbidden in ('tensor', 'embedding', 'feature_grid'))
    assert round_trip(intent).forward_permitted is False


def test_identifier_and_provenance_constants_are_distinct():
    assert ObjectObservation3D.EVIDENCE_CAMERA == 1
    assert ObjectObservation3D.EVIDENCE_LIDAR == 2
    assert ObjectObservation3D.EVIDENCE_STEREO_DEPTH == 4
    assert TrackedSemanticObject.MEMORY_OBSERVATION_ONLY == 0
    assert TrackedSemanticObject.MEMORY_LOCAL_SESSION == 1
    assert TrackedSemanticObject.MEMORY_WORLD == 2
    assert SearchMotionIntent.INTENT_ROTATE_VERIFY == 1
    assert SearchMotionIntent.INTENT_STOP == 2


def test_semantic_approach_authorization_carries_the_exact_target_reference():
    package_root = Path(__file__).resolve().parents[1]
    service = (
        package_root / 'srv' / 'AuthorizeSemanticApproach.srv'
    ).read_text()
    cmake = (package_root / 'CMakeLists.txt').read_text()

    for field in (
            'uint64 memory_epoch_id',
            'uint64 global_object_id',
            'uint64 localization_epoch_id',
            'uint64 query_id',
            'uint64 query_version',
            'uint64 snapshot_sequence'):
        assert field in service
    assert 'bool accepted' in service
    assert 'string<=256 reason' in service
    assert '"srv/AuthorizeSemanticApproach.srv"' in cmake


from track_robot_interfaces.msg import HumanFollowingSession


def test_human_following_session_exposes_motion_authorization_state():
    session = HumanFollowingSession(
        runtime_mode=HumanFollowingSession.MODE_ACTIVE,
        state=HumanFollowingSession.STATE_FOLLOWING,
        logical_target_id=17,
        motion_session_enabled=True,
        target_authorized=True,
        arm_request_pending=False,
        safety_armed=True,
        rc_override_active=False,
        target_confidence=0.82,
        reason='confirmed_camera_lidar',
    )

    assert HumanFollowingSession.MODE_SHADOW == 0
    assert HumanFollowingSession.MODE_ACTIVE == 1
    assert HumanFollowingSession.STATE_WAITING_FOR_GESTURE == 1
    assert HumanFollowingSession.STATE_RC_OVERRIDE == 6
    assert session.logical_target_id == 17
    assert session.target_authorized is True
