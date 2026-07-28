"""Isolated local YOLO-World runtime shared by offline and ROS callers."""

import importlib
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import numpy as np


ULTRALYTICS_VERSION = '8.2.103'
_MIB = 1024.0 * 1024.0
_MAX_DETECTIONS = 256


@dataclass(frozen=True)
class GroundedDetection:
    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    label: str


@dataclass(frozen=True)
class YoloWorldDependencies:
    torch: object
    clip: object
    yolo_world_class: object
    torch_load_original: object


def _inside(path_value, root):
    try:
        path = Path(path_value).resolve()
        root = Path(root).resolve()
        return os.path.commonpath((str(path), str(root))) == str(root)
    except (OSError, TypeError, ValueError):
        return False


def load_yolo_world_dependencies(
        runtime_path, clip_runtime_path) -> YoloWorldDependencies:
    runtime_path = Path(runtime_path)
    clip_runtime_path = Path(clip_runtime_path)
    for value in reversed((str(runtime_path), str(clip_runtime_path))):
        if value in sys.path:
            sys.path.remove(value)
        sys.path.insert(0, value)
    importlib.invalidate_caches()
    try:
        torch = importlib.import_module('torch')
        clip = importlib.import_module('clip')
        ultralytics = importlib.import_module('ultralytics')
        yolo_world_class = getattr(ultralytics, 'YOLOWorld')
        patches = importlib.import_module('ultralytics.utils.patches')
        torch_load_original = getattr(patches, '_torch_load')
    except (ImportError, AttributeError, OSError) as error:
        raise RuntimeError(
            'isolated YOLO-World runtime dependencies are unavailable: '
            '{}: {}'.format(type(error).__name__, str(error))
        ) from error
    if str(getattr(ultralytics, '__version__', '')) != ULTRALYTICS_VERSION:
        raise RuntimeError('isolated Ultralytics version is incompatible')
    if not _inside(getattr(ultralytics, '__file__', ''), runtime_path):
        raise RuntimeError('Ultralytics did not load from isolated runtime')
    if not _inside(getattr(clip, '__file__', ''), clip_runtime_path):
        raise RuntimeError('CLIP did not load from isolated runtime')
    return YoloWorldDependencies(
        torch=torch,
        clip=clip,
        yolo_world_class=yolo_world_class,
        torch_load_original=torch_load_original,
    )


def _to_list(value):
    for method_name in ('detach', 'cpu'):
        method = getattr(value, method_name, None)
        if method is not None:
            value = method()
    method = getattr(value, 'tolist', None)
    return method() if method is not None else list(value)


def _finite_number(value):
    return (
        isinstance(value, (int, float)) and
        not isinstance(value, bool) and
        math.isfinite(value)
    )


def _validate_probability(value, name):
    if not _finite_number(value) or not 0.0 <= float(value) <= 1.0:
        raise ValueError('{} must be in [0, 1]'.format(name))


def _validate_limit(value):
    if (
            isinstance(value, bool) or not isinstance(value, int) or
            not 1 <= value <= _MAX_DETECTIONS):
        raise ValueError('max_detections must be in [1, 256]')


def _label_for_class(names, class_id):
    if isinstance(names, dict):
        if class_id not in names:
            raise ValueError('YOLO-World class ID is unavailable')
        return names[class_id]
    if isinstance(names, (list, tuple)) and 0 <= class_id < len(names):
        return names[class_id]
    raise ValueError('YOLO-World class names are invalid')


def normalize_yolo_world_result(
        result, query, width, height,
        max_detections) -> Tuple[GroundedDetection, ...]:
    _validate_limit(max_detections)
    if not isinstance(query, str) or not query:
        raise ValueError('YOLO-World query must be non-empty')
    if (
            isinstance(width, bool) or not isinstance(width, int) or width <= 0
            or isinstance(height, bool) or not isinstance(height, int)
            or height <= 0):
        raise ValueError('image dimensions must be positive integers')
    boxes_value = getattr(result, 'boxes', None)
    if boxes_value is None:
        return ()
    try:
        boxes = _to_list(boxes_value.xyxy)
        scores = _to_list(boxes_value.conf)
        classes = _to_list(boxes_value.cls)
        names = result.names
    except (AttributeError, TypeError, ValueError) as error:
        raise ValueError('YOLO-World result fields are invalid') from error
    if not len(boxes) == len(scores) == len(classes):
        raise ValueError('YOLO-World result lengths do not match')

    detections = []
    for box, score, class_value in zip(boxes, scores, classes):
        if not isinstance(box, (list, tuple)) or len(box) != 4:
            raise ValueError('YOLO-World box is invalid')
        if not all(_finite_number(item) for item in box):
            raise ValueError('YOLO-World box must be finite')
        if not _finite_number(score) or not 0.0 <= float(score) <= 1.0:
            raise ValueError('YOLO-World score is invalid')
        if (
                not _finite_number(class_value) or
                float(class_value) != int(class_value) or
                int(class_value) < 0):
            raise ValueError('YOLO-World class ID is invalid')
        label = _label_for_class(names, int(class_value))
        if not isinstance(label, str) or label != query:
            raise ValueError('YOLO-World label does not match active query')
        x1 = min(max(float(box[0]), 0.0), float(width))
        y1 = min(max(float(box[1]), 0.0), float(height))
        x2 = min(max(float(box[2]), 0.0), float(width))
        y2 = min(max(float(box[3]), 0.0), float(height))
        if x2 <= x1 or y2 <= y1:
            continue
        detections.append(GroundedDetection(
            x1=x1,
            y1=y1,
            x2=x2,
            y2=y2,
            score=float(score),
            label=label,
        ))
    detections.sort(key=lambda value: (
        -value.score,
        value.x1,
        value.y1,
        value.x2,
        value.y2,
        value.label,
    ))
    return tuple(detections[:max_detections])


class YoloWorldBackend:
    def __init__(
            self, model, dependencies, clip_checkpoint, confidence_floor,
            iou_threshold, input_size, max_detections, device, half,
            baseline_reserved_bytes):
        self.model = model
        self._dependencies = dependencies
        self._clip_checkpoint = Path(clip_checkpoint)
        self._confidence_floor = confidence_floor
        self._iou_threshold = iou_threshold
        self._input_size = input_size
        self._max_detections = max_detections
        self._device = device
        self._cuda_device = 'cuda:{}'.format(device)
        self._half = half
        self._baseline_reserved_bytes = baseline_reserved_bytes
        self._active_query = None

    @classmethod
    def from_local_model(
            cls,
            runtime_path,
            clip_runtime_path,
            world_checkpoint,
            clip_checkpoint,
            confidence_floor=0.05,
            iou_threshold=0.70,
            input_size=640,
            max_detections=256,
            device=0,
            half=True,
            dependencies=None):
        _validate_probability(confidence_floor, 'confidence_floor')
        _validate_probability(iou_threshold, 'iou_threshold')
        _validate_limit(max_detections)
        if (
                isinstance(input_size, bool) or
                not isinstance(input_size, int) or
                not 1 <= input_size <= 4096):
            raise ValueError('input_size must be in [1, 4096]')
        if isinstance(device, bool) or not isinstance(device, int) or device < 0:
            raise ValueError('device must be a non-negative integer')
        if not isinstance(half, bool):
            raise ValueError('half must be boolean')
        runtime_path = Path(runtime_path)
        clip_runtime_path = Path(clip_runtime_path)
        world_checkpoint = Path(world_checkpoint)
        clip_checkpoint = Path(clip_checkpoint)
        if runtime_path.is_symlink() or not runtime_path.is_dir():
            raise ValueError('runtime_path must be a local directory')
        if clip_runtime_path.is_symlink() or not clip_runtime_path.is_dir():
            raise ValueError('clip_runtime_path must be a local directory')
        for path, name in (
                (world_checkpoint, 'world_checkpoint'),
                (clip_checkpoint, 'clip_checkpoint')):
            if path.is_symlink() or not path.is_file():
                raise ValueError('{} must be a local file'.format(name))

        dependencies = dependencies or load_yolo_world_dependencies(
            runtime_path, clip_runtime_path)
        cuda_device = 'cuda:{}'.format(device)
        try:
            dependencies.torch.cuda.init()
            baseline = int(
                dependencies.torch.cuda.memory_reserved(cuda_device))
            dependencies.torch.cuda.reset_peak_memory_stats(cuda_device)
            dependencies.torch.load = dependencies.torch_load_original
            model = dependencies.yolo_world_class(
                str(world_checkpoint), verbose=False)
        except (AttributeError, OSError, RuntimeError, TypeError, ValueError) \
                as error:
            raise RuntimeError(
                'local YOLO-World model initialization failed') from error
        finally:
            # Ultralytics 8.2 patches torch.load process-wide. Its PyTorch
            # 1.13 compatibility wrapper forwards ``weights_only`` to Jetson's
            # unpickler, where that keyword is unsupported. Model inference
            # does not need the wrapper after construction, and leaving it
            # installed prevents the subsequent DINOv3 checkpoint load.
            dependencies.torch.load = dependencies.torch_load_original
        return cls(
            model=model,
            dependencies=dependencies,
            clip_checkpoint=clip_checkpoint,
            confidence_floor=float(confidence_floor),
            iou_threshold=float(iou_threshold),
            input_size=input_size,
            max_detections=max_detections,
            device=device,
            half=half,
            baseline_reserved_bytes=max(0, baseline),
        )

    @staticmethod
    def _validate_query(value):
        if (
                not isinstance(value, str) or not value or
                len(value) > 160 or
                any(ord(character) < 0x20 or ord(character) > 0x7e
                    for character in value)):
            raise ValueError(
                'YOLO-World query must be bounded printable ASCII')

    def _set_query(self, query):
        if query == self._active_query:
            return
        original_load = self._dependencies.clip.load

        def load_local(_name, *_args, **_kwargs):
            return original_load(
                str(self._clip_checkpoint),
                device=self._cuda_device,
                jit=False,
            )

        self._dependencies.clip.load = load_local
        try:
            self.model.set_classes([query])
        except Exception as error:
            raise RuntimeError(
                'YOLO-World vocabulary update failed') from error
        finally:
            self._dependencies.clip.load = original_load
        self._active_query = query

    def active_text_descriptor(self):
        try:
            features = self.model.model.txt_feats
            array = np.asarray(_to_list(features), dtype=np.float32)
        except (AttributeError, TypeError, ValueError) as error:
            raise ValueError('YOLO-World text descriptor is unavailable') \
                from error
        if array.shape != (1, 1, 512) or not np.isfinite(array).all():
            raise ValueError('YOLO-World text descriptor is invalid')
        descriptor = array.reshape(512).copy()
        norm = float(np.linalg.norm(descriptor))
        if not math.isfinite(norm) or norm <= 1e-12:
            raise ValueError('YOLO-World text descriptor has zero norm')
        descriptor /= norm
        return descriptor

    def predict(self, image, normalized_query):
        self._validate_query(normalized_query)
        self._set_query(normalized_query)
        source = str(Path(image)) if isinstance(
            image, (str, bytes, os.PathLike)) else image
        try:
            results = self.model.predict(
                source=source,
                imgsz=self._input_size,
                conf=self._confidence_floor,
                iou=self._iou_threshold,
                max_det=self._max_detections,
                device=self._device,
                half=self._half,
                augment=False,
                verbose=False,
                save=False,
                stream=False,
            )
        except (OSError, RuntimeError, TypeError, ValueError) as error:
            raise RuntimeError('YOLO-World inference failed') from error
        if not isinstance(results, (list, tuple)) or len(results) != 1:
            raise ValueError('YOLO-World must return exactly one image result')
        shape = getattr(results[0], 'orig_shape', None)
        if (
                not isinstance(shape, (list, tuple)) or len(shape) != 2 or
                any(isinstance(value, bool) or not isinstance(value, int) or
                    value <= 0 for value in shape)):
            raise ValueError('YOLO-World original image shape is invalid')
        height, width = shape
        return normalize_yolo_world_result(
            results[0],
            normalized_query,
            width,
            height,
            self._max_detections,
        )

    def synchronize(self):
        try:
            self._dependencies.torch.cuda.synchronize(self._cuda_device)
        except (AttributeError, RuntimeError) as error:
            raise RuntimeError('CUDA synchronization failed') from error

    def incremental_cuda_reserved_mib(self):
        try:
            peak = int(self._dependencies.torch.cuda.max_memory_reserved(
                self._cuda_device))
        except (AttributeError, RuntimeError, TypeError, ValueError) as error:
            raise RuntimeError('CUDA memory query failed') from error
        return max(0, peak - self._baseline_reserved_bytes) / _MIB
