#!/usr/bin/env python3

import os
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import torch


IMAGENET_MEAN = np.asarray([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.asarray([0.229, 0.224, 0.225], dtype=np.float32)
DEFAULT_HF_MODEL_ID = 'facebook/dinov3-vits16plus-pretrain-lvd1689m'


@dataclass(frozen=True)
class PreprocessTransform:
    source_width: int
    source_height: int
    input_width: int
    input_height: int
    resized_width: int
    resized_height: int
    scale: float
    padding_left: int
    padding_top: int
    padding_right: int
    padding_bottom: int
    patch_size: int
    valid_patch_mask: np.ndarray

    @property
    def grid_width(self) -> int:
        return self.input_width // self.patch_size

    @property
    def grid_height(self) -> int:
        return self.input_height // self.patch_size


def _validate_image_and_size(
        image_bgr: np.ndarray, input_size: int, patch_size: int) -> None:
    if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError('image_bgr must be shaped H,W,3')
    if image_bgr.shape[0] <= 0 or image_bgr.shape[1] <= 0:
        raise ValueError('image_bgr dimensions must be positive')
    if input_size <= 0 or input_size % patch_size != 0:
        raise ValueError(
            'input_size must be a positive multiple of patch_size')
    if patch_size <= 0:
        raise ValueError('patch_size must be positive')


def _normalize_rgb(image_rgb: np.ndarray) -> torch.Tensor:
    image = image_rgb.astype(np.float32) / 255.0
    image = (image - IMAGENET_MEAN) / IMAGENET_STD
    return torch.from_numpy(image.transpose(2, 0, 1)).unsqueeze(0)


def preprocess_bgr(image_bgr: np.ndarray, input_size: int) -> torch.Tensor:
    if input_size <= 0 or input_size % 16 != 0:
        raise ValueError('input_size must be a positive multiple of 16')
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    image_rgb = cv2.resize(
        image_rgb, (input_size, input_size), interpolation=cv2.INTER_AREA)
    return _normalize_rgb(image_rgb)


def preprocess_bgr_aspect_preserving(
        image_bgr: np.ndarray,
        input_size: int,
        patch_size: int = 16) -> Tuple[torch.Tensor, PreprocessTransform]:
    _validate_image_and_size(image_bgr, input_size, patch_size)
    source_height, source_width = image_bgr.shape[:2]
    scale = min(
        float(input_size) / float(source_width),
        float(input_size) / float(source_height))
    resized_width = min(input_size, max(1, int(round(source_width * scale))))
    resized_height = min(input_size, max(1, int(round(source_height * scale))))

    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    interpolation = cv2.INTER_AREA if scale <= 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(
        image_rgb, (resized_width, resized_height), interpolation=interpolation)

    horizontal_padding = input_size - resized_width
    vertical_padding = input_size - resized_height
    padding_left = horizontal_padding // 2
    padding_right = horizontal_padding - padding_left
    padding_top = vertical_padding // 2
    padding_bottom = vertical_padding - padding_top
    canvas = cv2.copyMakeBorder(
        resized,
        padding_top,
        padding_bottom,
        padding_left,
        padding_right,
        cv2.BORDER_CONSTANT,
        value=(0, 0, 0))

    grid_size = input_size // patch_size
    patch_x0 = np.arange(grid_size) * patch_size
    patch_y0 = np.arange(grid_size) * patch_size
    valid_x = np.logical_and(
        patch_x0 < padding_left + resized_width,
        patch_x0 + patch_size > padding_left)
    valid_y = np.logical_and(
        patch_y0 < padding_top + resized_height,
        patch_y0 + patch_size > padding_top)
    valid_patch_mask = np.logical_and(valid_y[:, None], valid_x[None, :])

    transform = PreprocessTransform(
        source_width=source_width,
        source_height=source_height,
        input_width=input_size,
        input_height=input_size,
        resized_width=resized_width,
        resized_height=resized_height,
        scale=scale,
        padding_left=padding_left,
        padding_top=padding_top,
        padding_right=padding_right,
        padding_bottom=padding_bottom,
        patch_size=patch_size,
        valid_patch_mask=valid_patch_mask)
    return _normalize_rgb(canvas), transform


def map_model_roi_to_source(
        transform: PreprocessTransform,
        x: float,
        y: float,
        width: float,
        height: float) -> Tuple[int, int, int, int]:
    if width <= 0.0 or height <= 0.0:
        return 0, 0, 0, 0
    valid_left = float(transform.padding_left)
    valid_top = float(transform.padding_top)
    valid_right = valid_left + float(transform.resized_width)
    valid_bottom = valid_top + float(transform.resized_height)
    clipped_left = max(valid_left, min(valid_right, float(x)))
    clipped_top = max(valid_top, min(valid_bottom, float(y)))
    clipped_right = max(valid_left, min(valid_right, float(x) + float(width)))
    clipped_bottom = max(valid_top, min(valid_bottom, float(y) + float(height)))
    if clipped_right <= clipped_left or clipped_bottom <= clipped_top:
        return 0, 0, 0, 0

    source_left = int(np.floor((clipped_left - valid_left) / transform.scale))
    source_top = int(np.floor((clipped_top - valid_top) / transform.scale))
    source_right = int(np.ceil((clipped_right - valid_left) / transform.scale))
    source_bottom = int(np.ceil((clipped_bottom - valid_top) / transform.scale))
    source_left = max(0, min(transform.source_width, source_left))
    source_top = max(0, min(transform.source_height, source_top))
    source_right = max(source_left, min(transform.source_width, source_right))
    source_bottom = max(source_top, min(transform.source_height, source_bottom))
    return (
        source_left,
        source_top,
        source_right - source_left,
        source_bottom - source_top)


def load_model(
        model_source: str,
        model_name: str,
        device: str,
        local_repo: str = '',
        weights_path: str = '',
        hf_model_id: str = DEFAULT_HF_MODEL_ID):
    source = model_source.strip().lower()
    local_repo = os.path.expanduser(local_repo)
    weights_path = os.path.expanduser(weights_path)

    if local_repo and not os.path.isfile(os.path.join(local_repo, 'hubconf.py')):
        raise FileNotFoundError(
            'local_repo must be a real DINOv3 repository containing hubconf.py; '
            'received: {}'.format(local_repo))
    if weights_path and not (
            os.path.isfile(weights_path) or
            weights_path.startswith(('http://', 'https://', 'file://'))):
        raise FileNotFoundError(
            'weights_path must be a real checkpoint file or URL; received: {}'.format(
                weights_path))

    if source == 'torch_hub':
        repo = local_repo if local_repo else 'facebookresearch/dinov3'
        kwargs = {}
        if weights_path:
            kwargs['weights'] = weights_path
        try:
            model = torch.hub.load(
                repo, model_name, source='local' if local_repo else 'github', **kwargs)
        except TypeError as exc:
            if 'unsupported operand type(s) for |' in str(exc):
                raise RuntimeError(
                    'The current official DINOv3 repository requires Python 3.10+ '
                    'syntax, but ROS Foxy is using Python 3.8. Use a separately '
                    'prepared compatible runtime or exported model; do not upgrade '
                    'the working ROS environment in place.') from exc
            raise
        backend = 'official_torch_hub'
    elif source == 'local_repo':
        if not local_repo:
            raise ValueError('local_repo must be set when model_source=local_repo')
        if not weights_path:
            raise FileNotFoundError(
                'No pretrained DINOv3 checkpoint was supplied. Request access to '
                'facebook/dinov3-vits16plus-pretrain-lvd1689m, download the .pth '
                'file, and set weights_path to that real file.')
        kwargs = {}
        kwargs['weights'] = weights_path
        try:
            model = torch.hub.load(local_repo, model_name, source='local', **kwargs)
        except TypeError as exc:
            if 'unsupported operand type(s) for |' in str(exc):
                raise RuntimeError(
                    'This DINOv3 checkout is not Python 3.8-compatible. Use the '
                    'workspace compatibility copy documented in the README.') from exc
            raise
        backend = 'official_local_repo'
    elif source == 'huggingface':
        try:
            from transformers import AutoModel
        except ImportError as exc:
            raise RuntimeError(
                'transformers is not installed. Do not install it into the ROS environment '
                'without checking its Python/PyTorch requirements.') from exc
        model = AutoModel.from_pretrained(hf_model_id)
        backend = 'huggingface'
    else:
        raise ValueError(
            'model_source must be torch_hub, huggingface, or local_repo')

    model.eval()
    model.to(device)
    return model, backend


def extract_features(
        model,
        input_tensor: torch.Tensor,
        backend: str) -> Tuple[torch.Tensor, torch.Tensor, Dict[str, str]]:
    details = {}
    with torch.no_grad():
        if backend.startswith('official_'):
            if not hasattr(model, 'forward_features'):
                raise RuntimeError('Official model does not expose forward_features()')
            output = model.forward_features(input_tensor)
            if not isinstance(output, dict):
                raise RuntimeError(
                    'Unexpected official forward_features output: {}'.format(type(output)))
            details['output_keys'] = ','.join(sorted(output.keys()))
            cls_token = output['x_norm_clstoken']
            patch_tokens = output['x_norm_patchtokens']
        else:
            output = model(pixel_values=input_tensor)
            if not hasattr(output, 'last_hidden_state'):
                raise RuntimeError('Hugging Face model did not return last_hidden_state')
            tokens = output.last_hidden_state
            register_count = int(getattr(model.config, 'num_register_tokens', 0))
            cls_token = (
                output.pooler_output if getattr(output, 'pooler_output', None) is not None
                else tokens[:, 0])
            patch_tokens = tokens[:, 1 + register_count:]
            details['register_tokens'] = str(register_count)

    return cls_token, patch_tokens, details


def patch_grid(
        patch_tokens: torch.Tensor,
        grid_height: int,
        grid_width: Optional[int] = None) -> np.ndarray:
    if grid_width is None:
        if grid_height <= 0 or grid_height % 16 != 0:
            raise ValueError('legacy input_size must be a positive multiple of 16')
        grid_height = grid_height // 16
        grid_width = grid_height
    if grid_height <= 0 or grid_width <= 0:
        raise ValueError('grid dimensions must be positive')
    expected = grid_height * grid_width
    if patch_tokens.ndim != 3 or patch_tokens.shape[0] != 1:
        raise ValueError(
            'Expected patch tokens shaped [1,N,C], got {}'.format(
                tuple(patch_tokens.shape)))
    if patch_tokens.shape[1] != expected:
        raise ValueError(
            'Expected {} patch tokens for {}x{} grid, got {}'.format(
                expected, grid_height, grid_width, patch_tokens.shape[1]))
    return patch_tokens[0].detach().float().cpu().numpy().reshape(
        grid_height, grid_width, patch_tokens.shape[-1])


def make_feature_heatmap(image_bgr: np.ndarray, feature_grid: np.ndarray) -> np.ndarray:
    norms = np.linalg.norm(feature_grid, axis=2)
    low, high = np.percentile(norms, [2.0, 98.0])
    if high <= low:
        normalized = np.zeros(norms.shape, dtype=np.uint8)
    else:
        normalized = np.clip((norms - low) / (high - low), 0.0, 1.0)
        normalized = np.rint(normalized * 255.0).astype(np.uint8)
    heatmap = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    heatmap = cv2.resize(
        heatmap, (image_bgr.shape[1], image_bgr.shape[0]),
        interpolation=cv2.INTER_CUBIC)
    return cv2.addWeighted(image_bgr, 0.55, heatmap, 0.45, 0.0)


def normalize_feature_rows(features: np.ndarray) -> np.ndarray:
    norms = np.linalg.norm(features, axis=-1, keepdims=True)
    return features / np.maximum(norms, 1e-12)
