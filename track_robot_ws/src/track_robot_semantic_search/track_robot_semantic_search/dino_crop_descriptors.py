"""Bounded DINOv3 crop descriptors for visual identity evidence."""

import math
from dataclasses import dataclass

import cv2
import numpy as np

from .camera_tracking import AppearanceDescriptor
from .yolo_world_backend import GroundedDetection


@dataclass(frozen=True)
class DinoCropConfig:
    input_size: int = 224
    context_margin: float = 0.10
    maximum_crops: int = 3
    encoder_id: str = 'dinov3:vits16plus'
    checkpoint_id: str = 'dinov3_vits16plus_pretrain_lvd1689m.pth'
    descriptor_version: int = 1

    def __post_init__(self):
        if (
                isinstance(self.input_size, bool) or
                not isinstance(self.input_size, int) or
                self.input_size <= 0 or self.input_size % 16 != 0):
            raise ValueError('DINO input size must be a positive multiple of 16')
        if (
                not isinstance(self.context_margin, (int, float)) or
                isinstance(self.context_margin, bool) or
                not math.isfinite(self.context_margin) or
                not 0.0 <= float(self.context_margin) <= 0.5):
            raise ValueError('DINO context margin is invalid')
        if (
                isinstance(self.maximum_crops, bool) or
                not isinstance(self.maximum_crops, int) or
                not 1 <= self.maximum_crops <= 3):
            raise ValueError('DINO maximum crops must be in [1, 3]')
        if (
                not isinstance(self.encoder_id, str) or
                not self.encoder_id or len(self.encoder_id) > 128 or
                not isinstance(self.checkpoint_id, str) or
                not self.checkpoint_id or len(self.checkpoint_id) > 128 or
                isinstance(self.descriptor_version, bool) or
                not isinstance(self.descriptor_version, int) or
                self.descriptor_version <= 0):
            raise ValueError('DINO descriptor identity is invalid')


def extract_context_crops(
        image_bgr, detections, context_margin, maximum_crops):
    if (
            not isinstance(image_bgr, np.ndarray) or
            image_bgr.ndim != 3 or image_bgr.shape[2] != 3 or
            image_bgr.shape[0] <= 0 or image_bgr.shape[1] <= 0):
        raise ValueError('image_bgr must be a non-empty H,W,3 array')
    if (
            not isinstance(context_margin, (int, float)) or
            isinstance(context_margin, bool) or
            not math.isfinite(context_margin) or
            not 0.0 <= float(context_margin) <= 0.5 or
            isinstance(maximum_crops, bool) or
            not isinstance(maximum_crops, int) or
            not 1 <= maximum_crops <= 3):
        raise ValueError('crop bounds are invalid')
    height, width = image_bgr.shape[:2]
    crops = []
    for detection in tuple(detections)[:maximum_crops]:
        if not isinstance(detection, GroundedDetection):
            raise ValueError('crop detection is invalid')
        box_width = detection.x2 - detection.x1
        box_height = detection.y2 - detection.y1
        margin_x = box_width * float(context_margin)
        margin_y = box_height * float(context_margin)
        left = max(0, min(width, int(math.floor(detection.x1 - margin_x))))
        top = max(0, min(height, int(math.floor(detection.y1 - margin_y))))
        right = max(0, min(width, int(math.ceil(detection.x2 + margin_x))))
        bottom = max(0, min(height, int(math.ceil(detection.y2 + margin_y))))
        if right <= left or bottom <= top:
            raise ValueError('DINO crop is empty after clipping')
        crops.append(image_bgr[top:bottom, left:right].copy())
    return tuple(crops)


def _default_preprocess(crop, input_size):
    from track_robot_perception.dinov3_runtime import (
        preprocess_bgr_aspect_preserving,
    )
    return preprocess_bgr_aspect_preserving(crop, input_size)[0]


def _default_stack(values):
    import torch
    return torch.cat(tuple(values), dim=0)


def _default_extract(model, batch, backend):
    from track_robot_perception.dinov3_runtime import extract_cls_batch
    values, _ = extract_cls_batch(model, batch, backend)
    return values.detach().float().cpu().numpy()


class DinoCropDescriptorBackend:
    def __init__(
            self, model, backend, config,
            preprocess_fn=None, stack_fn=None, extract_fn=None,
            available=True, unavailable_reason='', device=None):
        if available and not isinstance(config, DinoCropConfig):
            raise ValueError('DINO crop config is required')
        self.model = model
        self.backend = backend
        self.config = config
        self.device = device
        self._preprocess = preprocess_fn or _default_preprocess
        self._stack = stack_fn or _default_stack
        self._extract = extract_fn or _default_extract
        self.available = bool(available)
        self.unavailable_reason = str(unavailable_reason)

    @classmethod
    def from_local_model(
            cls, local_repo, weights_path, device='cuda',
            config=None):
        from track_robot_perception.dinov3_runtime import load_model
        config = config or DinoCropConfig()
        model, backend = load_model(
            model_source='local_repo',
            model_name='dinov3_vits16plus',
            device=device,
            local_repo=str(local_repo),
            weights_path=str(weights_path),
        )
        return cls(
            model=model,
            backend=backend,
            config=config,
            device=device,
        )

    @classmethod
    def disabled(cls, reason):
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError('DINO unavailable reason must be non-empty')
        return cls(
            model=None,
            backend='disabled',
            config=None,
            available=False,
            unavailable_reason=reason.strip(),
        )

    def encode(self, image_bgr, detections):
        if not self.available:
            return ()
        crops = extract_context_crops(
            image_bgr,
            detections,
            self.config.context_margin,
            self.config.maximum_crops,
        )
        if not crops:
            return ()
        tensors = [
            self._preprocess(crop, self.config.input_size)
            for crop in crops
        ]
        batch = self._stack(tensors)
        if self.device is not None:
            move = getattr(batch, 'to', None)
            if move is None:
                raise ValueError('DINO input batch cannot move to model device')
            batch = move(self.device)
        features = np.asarray(
            self._extract(
                self.model,
                batch,
                self.backend,
            ),
            dtype=np.float32,
        )
        if (
                features.ndim != 2 or
                features.shape[0] != len(crops) or
                not 1 <= features.shape[1] <= 1024 or
                not np.isfinite(features).all()):
            raise ValueError('DINO descriptor batch is invalid')
        norms = np.linalg.norm(features, axis=1)
        if not np.isfinite(norms).all() or np.any(norms <= 1e-12):
            raise ValueError('DINO descriptor norm is invalid')
        normalized = features / norms[:, None]
        return tuple(
            AppearanceDescriptor(
                values=normalized[index],
                quality=1.0,
                encoder_id=self.config.encoder_id,
                checkpoint_id=self.config.checkpoint_id,
                version=self.config.descriptor_version,
            )
            for index in range(len(crops))
        )
