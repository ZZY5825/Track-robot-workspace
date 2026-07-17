import pytest
import numpy as np
import torch
import types

from track_robot_semantic_search.model_adapters import (
    ModelUnavailableError,
    create_aligned_encoder,
)


def test_open_clip_adapter_requires_real_checkpoint(tmp_path):
    missing = tmp_path / 'missing.pt'

    with pytest.raises(ModelUnavailableError, match='checkpoint does not exist'):
        create_aligned_encoder(
            'open_clip',
            model_name='ViT-B-32',
            checkpoint_path=str(missing),
            runtime_path='')


def test_open_clip_adapter_rejects_missing_external_runtime(tmp_path):
    checkpoint = tmp_path / 'model.pt'
    checkpoint.write_bytes(b'not loaded because runtime is missing')
    missing_runtime = tmp_path / 'no-runtime'

    with pytest.raises(ModelUnavailableError, match='runtime path does not exist'):
        create_aligned_encoder(
            'open_clip',
            model_name='ViT-B-32',
            checkpoint_path=str(checkpoint),
            runtime_path=str(missing_runtime))


def test_test_double_is_not_a_production_adapter():
    with pytest.raises(ModelUnavailableError, match='unknown aligned encoder'):
        create_aligned_encoder(
            'fake', model_name='fake', checkpoint_path='', runtime_path='')


def test_empty_implementation_is_rejected():
    with pytest.raises(ModelUnavailableError, match='unknown aligned encoder'):
        create_aligned_encoder(
            '', model_name='', checkpoint_path='', runtime_path='')


def test_openai_clip_adapter_uses_local_checkpoint_and_aligned_space(
        tmp_path, monkeypatch):
    checkpoint = tmp_path / 'ViT-B-32.pt'
    checkpoint.write_bytes(b'fixture')
    calls = {}

    class FakeModel:
        def eval(self):
            return self

        def encode_text(self, tokens):
            return torch.tensor([[3.0, 4.0]], dtype=torch.float32)

        def encode_image(self, batch):
            return torch.tensor(
                [[1.0, 0.0]] * int(batch.shape[0]), dtype=torch.float32)

    def load(path, device, jit):
        calls['load'] = (path, device, jit)
        return FakeModel(), lambda _: torch.zeros((3, 224, 224))

    fake_clip = types.SimpleNamespace(
        load=load,
        tokenize=lambda texts: torch.ones((len(texts), 77), dtype=torch.long),
    )
    monkeypatch.setitem(__import__('sys').modules, 'clip', fake_clip)

    adapter = create_aligned_encoder(
        'openai_clip',
        model_name='ViT-B/32',
        checkpoint_path=str(checkpoint),
        runtime_path=str(tmp_path),
        device='cpu',
        grid_size=2)
    text = adapter.encode_text('fallen branch')
    image = adapter.encode_image_grid(
        np.zeros((32, 64, 3), dtype=np.uint8))

    assert calls['load'] == (str(checkpoint), 'cpu', False)
    assert adapter.encoder_id == 'openai_clip:ViT-B/32'
    assert text.shape == (2,)
    assert image.embeddings.shape == (2, 2, 2)
    assert image.geometry.source_width == 64
    assert image.geometry.source_height == 32
    assert image.valid_patch_mask.shape == (2, 2)


def test_openai_clip_adapter_excludes_crops_that_are_mostly_padding(
        tmp_path, monkeypatch):
    checkpoint = tmp_path / 'ViT-B-32.pt'
    checkpoint.write_bytes(b'fixture')

    class FakeModel:
        def eval(self):
            return self

        def encode_image(self, batch):
            return torch.ones((int(batch.shape[0]), 2), dtype=torch.float32)

    fake_clip = types.SimpleNamespace(
        load=lambda *args, **kwargs: (
            FakeModel(), lambda _: torch.zeros((3, 224, 224))),
        tokenize=lambda texts: torch.ones((len(texts), 77), dtype=torch.long),
    )
    monkeypatch.setitem(__import__('sys').modules, 'clip', fake_clip)

    adapter = create_aligned_encoder(
        'openai_clip',
        model_name='ViT-B/32',
        checkpoint_path=str(checkpoint),
        runtime_path=str(tmp_path),
        device='cpu',
        grid_size=4)
    image = adapter.encode_image_grid(
        np.zeros((720, 1280, 3), dtype=np.uint8))

    assert image.valid_patch_mask.tolist() == [
        [False, False, False, False],
        [True, True, True, True],
        [True, True, True, True],
        [False, False, False, False],
    ]
