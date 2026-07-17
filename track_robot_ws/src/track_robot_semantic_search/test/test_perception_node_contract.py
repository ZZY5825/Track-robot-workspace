import ast
from pathlib import Path

import pytest

from track_robot_semantic_search.query_transport import parse_query_payload


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE = PACKAGE_ROOT / 'track_robot_semantic_search' / 'perception_node.py'


def test_query_payload_parses_typed_identifiers_and_normalized_text():
    query = parse_query_payload(
        '{"query_text":" fallen   branch ","query_id":8,"query_version":3}')

    assert query.query_text == 'fallen branch'
    assert query.query_id == 8
    assert query.query_version == 3


@pytest.mark.parametrize('payload,reason', [
    ('not json', 'valid JSON'),
    ('{}', 'query_text'),
    ('{"query_text":"x","query_id":-1,"query_version":1}', 'query_id'),
    ('{"query_text":"x","query_id":1,"query_version":true}', 'query_version'),
])
def test_query_payload_rejects_invalid_transport(payload, reason):
    with pytest.raises(ValueError, match=reason):
        parse_query_payload(payload)


def test_perception_node_has_only_passive_publishers():
    source = NODE.read_text(encoding='utf-8')
    tree = ast.parse(source)
    publisher_types = []
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        function = call.func
        if not (
                isinstance(function, ast.Attribute) and
                function.attr == 'create_publisher' and call.args):
            continue
        message_type = call.args[0]
        if isinstance(message_type, ast.Name):
            publisher_types.append(message_type.id)

    assert sorted(publisher_types) == [
        'SemanticObservationArray',
        'SemanticRegionArray',
        'SemanticTask',
        'String',
    ]
    for forbidden in (
            'cmd_vel', 'SearchMotionIntent', 'FollowDecision',
            'feature_tensor', 'patch_tokens'):
        assert forbidden not in source


def test_perception_node_does_not_reach_into_core_private_encoder_state():
    source = NODE.read_text(encoding='utf-8')

    assert '._encoder' not in source


def test_perception_node_resets_candidate_sequence_after_source_time_rollback():
    source = NODE.read_text(encoding='utf-8')

    assert 'self._core.start_producer_epoch()' in source


def test_perception_observations_carry_explicit_camera_calibration_identity():
    source = NODE.read_text(encoding='utf-8')

    assert "'calibration_id', 'zed_left_rectified_v1'" in source
    assert 'observation.calibration_id = self._calibration_id' in source


def test_perception_retains_ros_callback_entities_for_foxy_lifetime_safety():
    source = NODE.read_text(encoding='utf-8')
    assert 'self._image_subscription = self.create_subscription(' in source
    assert 'self._query_subscription = self.create_subscription(' in source
    assert 'self._proposal_subscription = self.create_subscription(' in source
    assert 'self._processing_timer = self.create_timer(' in source
    assert "'query_accepted'" in source
    assert "'image_received'" in source
    assert "'processing_timer_started'" in source


def test_perception_node_is_packaged_as_console_script():
    setup_source = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')

    assert 'semantic_search_perception' in setup_source
    assert 'track_robot_semantic_search.perception_node:main' in setup_source
