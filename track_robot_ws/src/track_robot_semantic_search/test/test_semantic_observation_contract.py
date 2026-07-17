import numpy as np
import pytest

from track_robot_semantic_search.visual_candidates import (
    build_visual_descriptor_message,
)


def test_visual_descriptor_message_carries_explicit_model_identity():
    message = build_visual_descriptor_message(
        np.asarray([0.6, 0.8], dtype=np.float32),
        quality=0.75,
        encoder_id='clip-vit-b32',
        checkpoint_id='sha256:abc',
        version=3,
    )

    assert message.encoder_id == 'clip-vit-b32'
    assert message.checkpoint_id == 'sha256:abc'
    assert message.version == 3
    assert message.dimension == 2
    assert message.l2_normalized is True
    assert message.quality == pytest.approx(0.75)
    assert message.values == pytest.approx([0.6, 0.8])


def test_visual_descriptor_message_rejects_unbounded_or_invalid_values():
    with pytest.raises(ValueError, match='1024'):
        build_visual_descriptor_message(
            np.ones(1025, dtype=np.float32), 1.0, 'encoder', 'checkpoint', 1)
    with pytest.raises(ValueError, match='unit normalized'):
        build_visual_descriptor_message(
            np.asarray([1.0, 1.0]), 1.0, 'encoder', 'checkpoint', 1)
    with pytest.raises(ValueError, match='quality'):
        build_visual_descriptor_message(
            np.asarray([1.0, 0.0]), 1.1, 'encoder', 'checkpoint', 1)
