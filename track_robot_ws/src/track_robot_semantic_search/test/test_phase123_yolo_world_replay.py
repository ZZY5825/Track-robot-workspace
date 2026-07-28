import json
from pathlib import Path

import numpy as np

from track_robot_semantic_search.camera_tracking import (
    AppearanceDescriptor,
    CameraTrackManager,
    CameraTrackingConfig,
)
from track_robot_semantic_search.phase123_replay import run_phase123_replay_twice
from track_robot_semantic_search.query_transport import ActiveQuery
from track_robot_semantic_search.yolo_world_backend import GroundedDetection
from track_robot_semantic_search.yolo_world_perception_core import (
    YoloWorldPerceptionCore,
)


FIXTURE = Path(__file__).parent / 'data' / 'phase123_yolo_world_replay.json'
WORKSPACE = Path(__file__).resolve().parents[3]
REPLAY = (
    WORKSPACE / 'build' / 'track_robot_semantic_memory'
    / 'semantic_memory_replay')


class FixtureWorld:
    def __init__(self, frames, query):
        self.frames = iter(frames)
        self.query = query
        self.current = None

    def predict(self, _image, query):
        assert query == self.query
        self.current = next(self.frames)
        return tuple(
            GroundedDetection(
                *candidate['box'],
                candidate['confidence'],
                candidate['label'],
            )
            for candidate in self.current['camera_candidates']
        )

    @staticmethod
    def active_text_descriptor():
        result = np.zeros((512,), dtype=np.float32)
        result[0] = 1.0
        return result


class FixtureDino:
    available = True
    unavailable_reason = ''

    def __init__(self, world):
        self.world = world

    def encode(self, _image, detections):
        candidates = self.world.current['camera_candidates']
        assert len(detections) == min(3, len(candidates))
        return tuple(
            AppearanceDescriptor(
                values=np.asarray(
                    candidate['appearance']['values'], dtype=np.float32),
                quality=candidate['appearance']['quality'],
                encoder_id=candidate['appearance']['encoder_id'],
                checkpoint_id=candidate['appearance']['checkpoint_id'],
                version=candidate['appearance']['version'],
            )
            for candidate in candidates[:len(detections)]
        )


def test_phase123_fixture_uses_real_tracking_and_memory_contracts(tmp_path):
    fixture = json.loads(FIXTURE.read_text(encoding='utf-8'))
    query = fixture['query']
    world = FixtureWorld(fixture['frames'], query['text'])
    core = YoloWorldPerceptionCore(
        backend=world,
        dino_backend=FixtureDino(world),
        tracker=CameraTrackManager(CameraTrackingConfig()),
        clock_ns=iter(range(1000)).__next__,
    )
    core.accept_query(ActiveQuery(
        query['text'], query['query_id'], query['query_version']))

    target_track_ids = []
    distractor_track_id = None
    for frame in fixture['frames']:
        result = core.process(
            np.zeros((
                fixture['image_height'],
                fixture['image_width'],
                3,
            ), dtype=np.uint8),
            frame['source_stamp_ns'],
        )
        assert result.query_id == query['query_id']
        assert result.query_version == query['query_version']
        expected = frame['camera_candidates']
        assert [item.camera_track_id for item in result.candidates] == [
            item['camera_track_id'] for item in expected]
        target_track_ids.append(result.candidates[0].camera_track_id)
        if len(result.candidates) == 2:
            distractor_track_id = result.candidates[1].camera_track_id

    assert target_track_ids == [1, 1, 1]
    assert distractor_track_id == 2

    output = tmp_path / 'phase123_output.json'
    report = tmp_path / 'phase123_report.json'
    assert run_phase123_replay_twice(
        REPLAY, FIXTURE, output, report) == 0

    result = json.loads(output.read_text(encoding='utf-8'))
    evidence = json.loads(report.read_text(encoding='utf-8'))
    target_id = result['frames'][0]['objects'][0]['global_object_id']
    assert result['query']['text'] == 'blue toothpaste container'
    assert result['frames'][0]['objects'][0]['support'] == 'CAMERA_ONLY'
    assert result['frames'][1]['objects'][0]['global_object_id'] == target_id
    assert result['frames'][1]['objects'][0]['support'] == 'CAMERA_LIDAR'
    assert result['frames'][2]['objects'][0]['global_object_id'] == target_id
    assert result['frames'][2]['objects'][1]['global_object_id'] != target_id
    assert result['diagnostic_ranking'][0]['global_object_id'] == target_id
    assert result['diagnostic_ranking'][0]['evidence_mode'] == (
        'YOLO_WORLD_GROUNDING')
    assert result['calibration_state'] == 'UNCALIBRATED'
    assert result['best_candidate'] == []
    assert evidence['deterministic_replay_passed'] is True
    assert evidence['production_best_candidate_empty'] is True
