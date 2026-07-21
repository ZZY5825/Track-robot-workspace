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
