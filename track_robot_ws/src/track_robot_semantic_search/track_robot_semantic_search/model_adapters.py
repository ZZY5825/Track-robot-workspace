from dataclasses import dataclass
import importlib
import os
import sys
import time
from typing import Any

import cv2
import numpy as np

from .region_scoring import ImageGeometry


class ModelUnavailableError(RuntimeError):
    pass


@dataclass(frozen=True)
class ImageGridEncoding:
    embeddings: np.ndarray
    valid_patch_mask: np.ndarray
    geometry: ImageGeometry
    inference_ms: float


def _valid_crop_mask(
        grid_size: int,
        model_side: int,
        resized_width: int,
        resized_height: int,
        padding_left: int,
        padding_top: int) -> np.ndarray:
    """Keep cells with at least half of their area inside real image data."""
    edges = np.rint(np.linspace(0, model_side, grid_size + 1)).astype(np.int32)
    valid_left = padding_left
    valid_top = padding_top
    valid_right = padding_left + resized_width
    valid_bottom = padding_top + resized_height

    def overlap_fraction(start, end, valid_start, valid_end):
        overlap = max(0, min(end, valid_end) - max(start, valid_start))
        return float(overlap) / float(end - start)

    valid_x = np.asarray([
        overlap_fraction(edges[index], edges[index + 1],
                         valid_left, valid_right) >= 0.5
        for index in range(grid_size)
    ], dtype=bool)
    valid_y = np.asarray([
        overlap_fraction(edges[index], edges[index + 1],
                         valid_top, valid_bottom) >= 0.5
        for index in range(grid_size)
    ], dtype=bool)
    return np.logical_and(valid_y[:, None], valid_x[None, :])


class OpenClipAdapter:
    def __init__(
            self,
            model_name: str,
            checkpoint_path: str,
            runtime_path: str = '',
            device: str = 'cuda',
            grid_size: int = 4):
        checkpoint_path = os.path.abspath(os.path.expanduser(checkpoint_path))
        runtime_path = os.path.abspath(os.path.expanduser(runtime_path)) \
            if runtime_path else ''
        if not os.path.isfile(checkpoint_path):
            raise ModelUnavailableError(
                'OpenCLIP checkpoint does not exist: {}'.format(checkpoint_path))
        if runtime_path and not os.path.isdir(runtime_path):
            raise ModelUnavailableError(
                'OpenCLIP runtime path does not exist: {}'.format(runtime_path))
        if not model_name.strip():
            raise ModelUnavailableError('OpenCLIP model_name must not be empty')
        if grid_size <= 0:
            raise ModelUnavailableError('OpenCLIP grid_size must be positive')
        if runtime_path and runtime_path not in sys.path:
            sys.path.insert(0, runtime_path)
        try:
            open_clip = importlib.import_module('open_clip')
            torch = importlib.import_module('torch')
        except ImportError as exc:
            raise ModelUnavailableError(
                'OpenCLIP runtime is not importable; use an isolated external '
                'runtime compatible with Python 3.8 and PyTorch 1.13') from exc
        try:
            model, _, preprocess = open_clip.create_model_and_transforms(
                model_name.strip(), pretrained=checkpoint_path, device=device)
            tokenizer = open_clip.get_tokenizer(model_name.strip())
        except Exception as exc:
            raise ModelUnavailableError(
                'OpenCLIP model could not be loaded: {}'.format(exc)) from exc
        model.eval()
        self._torch = torch
        self._model = model
        self._preprocess = preprocess
        self._tokenizer = tokenizer
        self._device = device
        self._grid_size = grid_size
        self.encoder_id = 'open_clip:{}'.format(model_name.strip())
        self.checkpoint_id = os.path.basename(checkpoint_path)

    def encode_text(self, text: str) -> np.ndarray:
        tokens = self._tokenizer([text]).to(self._device)
        with self._torch.no_grad():
            vector = self._model.encode_text(tokens)
        return vector[0].detach().float().cpu().numpy()

    def encode_image_grid(self, image_bgr: np.ndarray) -> ImageGridEncoding:
        from PIL import Image

        source_height, source_width = image_bgr.shape[:2]
        side = max(source_width, source_height)
        padding_left = (side - source_width) // 2
        padding_top = (side - source_height) // 2
        canvas = np.zeros((side, side, 3), dtype=np.uint8)
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        canvas[
            padding_top:padding_top + source_height,
            padding_left:padding_left + source_width] = image_rgb
        edges = np.rint(
            np.linspace(0, side, self._grid_size + 1)).astype(np.int32)
        crops = []
        for row in range(self._grid_size):
            for column in range(self._grid_size):
                crop = canvas[
                    edges[row]:edges[row + 1],
                    edges[column]:edges[column + 1]]
                crops.append(self._preprocess(Image.fromarray(crop)))
        batch = self._torch.stack(crops).to(self._device)
        start = time.monotonic()
        with self._torch.no_grad():
            vectors = self._model.encode_image(batch)
        if str(self._device).startswith('cuda'):
            self._torch.cuda.synchronize()
        inference_ms = (time.monotonic() - start) * 1000.0
        embeddings = vectors.detach().float().cpu().numpy().reshape(
            self._grid_size, self._grid_size, -1)

        model_side = self._grid_size * 16
        scale = float(model_side) / float(side)
        resized_width = max(1, int(round(source_width * scale)))
        resized_height = max(1, int(round(source_height * scale)))
        model_padding_left = (model_side - resized_width) // 2
        model_padding_top = (model_side - resized_height) // 2
        valid = _valid_crop_mask(
            self._grid_size, model_side,
            resized_width, resized_height,
            model_padding_left, model_padding_top)
        return ImageGridEncoding(
            embeddings=embeddings,
            valid_patch_mask=valid,
            geometry=ImageGeometry(
                source_width=source_width,
                source_height=source_height,
                model_width=model_side,
                model_height=model_side,
                resized_width=resized_width,
                resized_height=resized_height,
                scale=scale,
                padding_left=model_padding_left,
                padding_top=model_padding_top,
                patch_size=16),
            inference_ms=inference_ms)


class OpenAIClipAdapter:
    def __init__(
            self,
            model_name: str,
            checkpoint_path: str,
            runtime_path: str = '',
            device: str = 'cuda',
            grid_size: int = 4):
        checkpoint_path = os.path.abspath(os.path.expanduser(checkpoint_path))
        runtime_path = os.path.abspath(os.path.expanduser(runtime_path)) \
            if runtime_path else ''
        if not os.path.isfile(checkpoint_path):
            raise ModelUnavailableError(
                'OpenAI CLIP checkpoint does not exist: {}'.format(
                    checkpoint_path))
        if runtime_path and not os.path.isdir(runtime_path):
            raise ModelUnavailableError(
                'OpenAI CLIP runtime path does not exist: {}'.format(
                    runtime_path))
        if not model_name.strip():
            raise ModelUnavailableError(
                'OpenAI CLIP model_name must not be empty')
        if grid_size <= 0:
            raise ModelUnavailableError(
                'OpenAI CLIP grid_size must be positive')
        if runtime_path and runtime_path not in sys.path:
            sys.path.insert(0, runtime_path)
        try:
            clip = importlib.import_module('clip')
            torch = importlib.import_module('torch')
        except ImportError as exc:
            raise ModelUnavailableError(
                'OpenAI CLIP runtime is not importable; use the isolated '
                'Python-3.8 runtime recorded by the Phase 1 manifest') from exc
        try:
            model, preprocess = clip.load(
                checkpoint_path, device=device, jit=False)
        except Exception as exc:
            raise ModelUnavailableError(
                'OpenAI CLIP model could not be loaded: {}'.format(exc)) \
                from exc
        model.eval()
        self._clip = clip
        self._torch = torch
        self._model = model
        self._preprocess = preprocess
        self._device = device
        self._grid_size = grid_size
        self.encoder_id = 'openai_clip:{}'.format(model_name.strip())
        self.checkpoint_id = os.path.basename(checkpoint_path)

    def encode_text(self, text: str) -> np.ndarray:
        tokens = self._clip.tokenize([text]).to(self._device)
        with self._torch.no_grad():
            vector = self._model.encode_text(tokens)
        return vector[0].detach().float().cpu().numpy()

    def encode_image_grid(self, image_bgr: np.ndarray) -> ImageGridEncoding:
        from PIL import Image

        source_height, source_width = image_bgr.shape[:2]
        side = max(source_width, source_height)
        padding_left = (side - source_width) // 2
        padding_top = (side - source_height) // 2
        canvas = np.zeros((side, side, 3), dtype=np.uint8)
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        canvas[
            padding_top:padding_top + source_height,
            padding_left:padding_left + source_width] = image_rgb
        edges = np.rint(
            np.linspace(0, side, self._grid_size + 1)).astype(np.int32)
        crops = []
        for row in range(self._grid_size):
            for column in range(self._grid_size):
                crop = canvas[
                    edges[row]:edges[row + 1],
                    edges[column]:edges[column + 1]]
                crops.append(self._preprocess(Image.fromarray(crop)))
        batch = self._torch.stack(crops).to(self._device)
        start = time.monotonic()
        with self._torch.no_grad():
            vectors = self._model.encode_image(batch)
        if str(self._device).startswith('cuda'):
            self._torch.cuda.synchronize()
        inference_ms = (time.monotonic() - start) * 1000.0
        embeddings = vectors.detach().float().cpu().numpy().reshape(
            self._grid_size, self._grid_size, -1)

        model_side = self._grid_size * 16
        scale = float(model_side) / float(side)
        resized_width = max(1, int(round(source_width * scale)))
        resized_height = max(1, int(round(source_height * scale)))
        model_padding_left = (model_side - resized_width) // 2
        model_padding_top = (model_side - resized_height) // 2
        valid = _valid_crop_mask(
            self._grid_size, model_side,
            resized_width, resized_height,
            model_padding_left, model_padding_top)
        return ImageGridEncoding(
            embeddings=embeddings,
            valid_patch_mask=valid,
            geometry=ImageGeometry(
                source_width=source_width,
                source_height=source_height,
                model_width=model_side,
                model_height=model_side,
                resized_width=resized_width,
                resized_height=resized_height,
                scale=scale,
                padding_left=model_padding_left,
                padding_top=model_padding_top,
                patch_size=16),
            inference_ms=inference_ms)


def create_aligned_encoder(
        implementation: str,
        model_name: str,
        checkpoint_path: str,
        runtime_path: str,
        device: str = 'cuda',
        grid_size: int = 4) -> Any:
    selected = implementation.strip().lower()
    if selected == 'open_clip':
        return OpenClipAdapter(
            model_name=model_name,
            checkpoint_path=checkpoint_path,
            runtime_path=runtime_path,
            device=device,
            grid_size=grid_size)
    if selected == 'openai_clip':
        return OpenAIClipAdapter(
            model_name=model_name,
            checkpoint_path=checkpoint_path,
            runtime_path=runtime_path,
            device=device,
            grid_size=grid_size)
    raise ModelUnavailableError(
        'unknown aligned encoder implementation: {}'.format(
            implementation))
