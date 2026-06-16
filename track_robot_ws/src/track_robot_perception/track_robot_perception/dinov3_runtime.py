#!/usr/bin/env python3

import os
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import torch


IMAGENET_MEAN = np.asarray([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.asarray([0.229, 0.224, 0.225], dtype=np.float32)
DEFAULT_HF_MODEL_ID = 'facebook/dinov3-vits16plus-pretrain-lvd1689m'


def preprocess_bgr(image_bgr: np.ndarray, input_size: int) -> torch.Tensor:
    if input_size <= 0 or input_size % 16 != 0:
        raise ValueError('input_size must be a positive multiple of 16')
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    image_rgb = cv2.resize(
        image_rgb, (input_size, input_size), interpolation=cv2.INTER_AREA)
    image = image_rgb.astype(np.float32) / 255.0
    image = (image - IMAGENET_MEAN) / IMAGENET_STD
    return torch.from_numpy(image.transpose(2, 0, 1)).unsqueeze(0)


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


def patch_grid(patch_tokens: torch.Tensor, input_size: int) -> np.ndarray:
    grid_size = input_size // 16
    expected = grid_size * grid_size
    if patch_tokens.ndim != 3 or patch_tokens.shape[0] != 1:
        raise ValueError(
            'Expected patch tokens shaped [1,N,C], got {}'.format(
                tuple(patch_tokens.shape)))
    if patch_tokens.shape[1] != expected:
        raise ValueError(
            'Expected {} patch tokens for {}x{} input, got {}'.format(
                expected, input_size, input_size, patch_tokens.shape[1]))
    return patch_tokens[0].detach().float().cpu().numpy().reshape(
        grid_size, grid_size, patch_tokens.shape[-1])


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
