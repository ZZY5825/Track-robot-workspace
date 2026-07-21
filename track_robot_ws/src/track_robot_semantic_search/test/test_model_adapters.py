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
        encode_image_call_count = 0
        last_batch = None

        def eval(self):
            return self

        def encode_text(self, tokens):
            return torch.tensor([[3.0, 4.0]], dtype=torch.float32)

        def encode_image(self, batch):
            self.encode_image_call_count += 1
            self.last_batch = batch
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
    assert image.extra_windows == ()
    assert adapter._model.encode_image_call_count == 1
    assert adapter._model.last_batch.shape[0] == 4
    assert image.geometry.source_width == 64
    assert image.geometry.source_height == 32
    assert image.valid_patch_mask.shape == (2, 2)


def test_openai_clip_multiscale_encodes_six_windows_in_one_batch(
        tmp_path, monkeypatch):
    checkpoint = tmp_path / 'ViT-B-32.pt'
    checkpoint.write_bytes(b'fixture')

    class FakeModel:
        def __init__(self):
            self.encode_image_call_count = 0
            self.last_batch = None

        def eval(self):
            return self

        def encode_image(self, batch):
            self.encode_image_call_count += 1
            self.last_batch = batch
            return torch.tensor(
                [[float(index + 1), 1.0]
                 for index in range(int(batch.shape[0]))],
                dtype=torch.float32)

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
        grid_size=2,
        window_strategy='multiscale_v1',
        center_window_scale=0.60)
    result = adapter.encode_image_grid(
        np.zeros((720, 1280, 3), dtype=np.uint8))

    assert adapter._model.encode_image_call_count == 1
    assert adapter._model.last_batch.shape[0] == 6
    assert result.embeddings.shape == (2, 2, 2)
    assert [item.kind for item in result.extra_windows] == [
        'global', 'center']
    assert result.extra_windows[0].roi == (0, 0, 1280, 720)
    assert result.extra_windows[1].roi == (256, 144, 768, 432)


def test_open_clip_multiscale_encodes_six_windows_in_one_batch(
        tmp_path, monkeypatch):
    checkpoint = tmp_path / 'ViT-B-32.pt'
    checkpoint.write_bytes(b'fixture')

    class FakeModel:
        def __init__(self):
            self.encode_image_call_count = 0
            self.last_batch = None

        def eval(self):
            return self

        def encode_image(self, batch):
            self.encode_image_call_count += 1
            self.last_batch = batch
            return torch.ones((int(batch.shape[0]), 2), dtype=torch.float32)

    model = FakeModel()
    fake_open_clip = types.SimpleNamespace(
        create_model_and_transforms=lambda *args, **kwargs: (
            model, None, lambda _: torch.zeros((3, 224, 224))),
        get_tokenizer=lambda _: (
            lambda texts: torch.ones((len(texts), 77), dtype=torch.long)),
    )
    monkeypatch.setitem(__import__('sys').modules, 'open_clip', fake_open_clip)

    adapter = create_aligned_encoder(
        'open_clip',
        model_name='ViT-B-32',
        checkpoint_path=str(checkpoint),
        runtime_path=str(tmp_path),
        device='cpu',
        grid_size=2,
        window_strategy='multiscale_v1',
        center_window_scale=0.60)
    result = adapter.encode_image_grid(
        np.zeros((720, 1280, 3), dtype=np.uint8))

    assert model.encode_image_call_count == 1
    assert model.last_batch.shape[0] == 6
    assert [item.kind for item in result.extra_windows] == [
        'global', 'center']


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
