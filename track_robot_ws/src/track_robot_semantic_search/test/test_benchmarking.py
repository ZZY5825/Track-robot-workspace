import math

import pytest

from track_robot_semantic_search.benchmarking import (
    available_candidate,
    benchmark_aligned_encoder,
    latency_summary,
    unavailable_candidate,
)


def test_latency_summary_is_deterministic_and_complete():
    summary = latency_summary([10.0, 20.0, 30.0, 40.0])

    assert summary == {
        'count': 4,
        'mean_ms': 25.0,
        'p50_ms': 25.0,
        'p95_ms': pytest.approx(38.5),
        'maximum_ms': 40.0,
    }


@pytest.mark.parametrize('values', [[], [math.nan], [-1.0]])
def test_latency_summary_rejects_invalid_input(values):
    with pytest.raises(ValueError, match='latencies'):
        latency_summary(values)


def test_unavailable_candidate_fails_every_unmeasured_gate():
    candidate = unavailable_candidate(
        'siglip2_b', 'runtime and checkpoint absent',
        python_compatible=False, licence_approved=False)

    assert candidate['candidate_id'] == 'siglip2_b'
    assert candidate['available'] is False
    assert candidate['python_compatible'] is False
    assert candidate['licence_approved'] is False
    assert candidate['memory_pass'] is False
    assert candidate['latency_pass'] is False
    assert candidate['phrase_region_recall'] is None
    assert candidate['p95_latency_ms'] == 0.0
    assert candidate['reason'] == 'runtime and checkpoint absent'


def test_available_candidate_preserves_not_evaluated_accuracy():
    candidate = available_candidate(
        'openai_clip_vit_b32',
        p95_latency_ms=42.0,
        peak_memory_mb=390.0,
        memory_limit_mb=1024.0,
        phrase_region_recall=None)

    assert candidate['available'] is True
    assert candidate['latency_pass'] is True
    assert candidate['memory_pass'] is True
    assert candidate['phrase_region_recall'] is None
    assert candidate['accuracy_status'] == 'not_evaluated'


def test_aligned_benchmark_uses_real_adapter_interface_and_images():
    class Encoding:
        inference_ms = 20.0
        embeddings = __import__('numpy').ones((2, 2, 2), dtype='float32')
        valid_patch_mask = __import__('numpy').ones((2, 2), dtype=bool)

    class Adapter:
        encoder_id = 'real:test'
        checkpoint_id = 'checkpoint.pt'

        def encode_text(self, text):
            assert text == 'a person'
            return __import__('numpy').array([1.0, 0.0], dtype='float32')

        def encode_image_grid(self, image):
            assert image.shape == (4, 8, 3)
            return Encoding()

    result = benchmark_aligned_encoder(
        Adapter(),
        [__import__('numpy').zeros((4, 8, 3), dtype='uint8')] * 3,
        query_text='a person')

    assert result['observation_count'] == 3
    assert result['query_text'] == 'a person'
    assert result['image_latency']['p95_ms'] == 20.0
    assert result['score_summary']['maximum'] == pytest.approx(2.0 ** -0.5)
