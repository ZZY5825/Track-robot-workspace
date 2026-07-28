import importlib
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

from .contracts import TeacherDetection


_MIB = 1024.0 * 1024.0
_MAX_DETECTIONS = 256


@dataclass(frozen=True)
class HuggingFaceDependencies:
    torch: object
    image_module: object
    processor_class: object
    model_class: object


def load_huggingface_dependencies() -> HuggingFaceDependencies:
    try:
        torch = importlib.import_module('torch')
        image_module = importlib.import_module('PIL.Image')
        transformers = importlib.import_module('transformers')
        processor_class = transformers.AutoProcessor
        model_class = transformers.AutoModelForZeroShotObjectDetection
    except (ImportError, AttributeError, OSError) as error:
        raise RuntimeError(
            'Grounding DINO runtime dependencies are unavailable') from error
    return HuggingFaceDependencies(
        torch=torch,
        image_module=image_module,
        processor_class=processor_class,
        model_class=model_class,
    )


def _to_list(value):
    for method_name in ('detach', 'cpu'):
        method = getattr(value, method_name, None)
        if method is not None:
            value = method()
    method = getattr(value, 'tolist', None)
    return method() if method is not None else list(value)


def _finite_number(value) -> bool:
    return (
        isinstance(value, (int, float)) and
        not isinstance(value, bool) and
        math.isfinite(value)
    )


def _validate_limit(max_detections):
    if (
            isinstance(max_detections, bool) or
            not isinstance(max_detections, int) or
            not 1 <= max_detections <= _MAX_DETECTIONS):
        raise ValueError('max_detections must be in [1, 256]')


def _validate_threshold(value, name):
    if not _finite_number(value) or not 0.0 <= float(value) <= 1.0:
        raise ValueError('{} must be in [0, 1]'.format(name))


def normalize_detection_result(
        result, width, height, max_detections) -> Tuple[TeacherDetection, ...]:
    _validate_limit(max_detections)
    if (
            isinstance(width, bool) or not isinstance(width, int) or width <= 0
            or isinstance(height, bool) or not isinstance(height, int)
            or height <= 0):
        raise ValueError('image dimensions must be positive integers')
    if not isinstance(result, dict):
        raise ValueError('Grounding DINO result must be a mapping')
    try:
        boxes = _to_list(result['boxes'])
        scores = _to_list(result['scores'])
        labels = _to_list(
            result['labels'] if 'labels' in result
            else result['text_labels'])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError('Grounding DINO result fields are invalid') from error
    if not len(boxes) == len(scores) == len(labels):
        raise ValueError('Grounding DINO result lengths do not match')

    detections = []
    for box, score, label in zip(boxes, scores, labels):
        if not isinstance(box, (list, tuple)) or len(box) != 4:
            raise ValueError('Grounding DINO box is invalid')
        if not all(_finite_number(value) for value in box):
            raise ValueError('Grounding DINO box must be finite')
        if not _finite_number(score) or not 0.0 <= float(score) <= 1.0:
            raise ValueError('Grounding DINO score is invalid')
        if not isinstance(label, str) or not label:
            raise ValueError('Grounding DINO label is invalid')
        x1 = min(max(float(box[0]), 0.0), float(width))
        y1 = min(max(float(box[1]), 0.0), float(height))
        x2 = min(max(float(box[2]), 0.0), float(width))
        y2 = min(max(float(box[3]), 0.0), float(height))
        if x2 <= x1 or y2 <= y1:
            continue
        detections.append(TeacherDetection(
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


class HuggingFaceGroundingDino:
    def __init__(
            self, processor, model, dependencies, device, box_threshold,
            text_threshold, max_detections, baseline_reserved_bytes):
        self.processor = processor
        self.model = model
        self._dependencies = dependencies
        self._device = device
        self._box_threshold = box_threshold
        self._text_threshold = text_threshold
        self._max_detections = max_detections
        self._baseline_reserved_bytes = baseline_reserved_bytes

    @classmethod
    def from_local_model(
            cls,
            model_dir,
            box_threshold=0.05,
            text_threshold=0.05,
            max_detections=256,
            device='cuda',
            dependencies=None):
        _validate_threshold(box_threshold, 'box_threshold')
        _validate_threshold(text_threshold, 'text_threshold')
        _validate_limit(max_detections)
        if not isinstance(device, str) or not device:
            raise ValueError('device must be non-empty')
        model_path = Path(model_dir)
        if model_path.is_symlink() or not model_path.is_dir():
            raise ValueError('model_dir must be a local directory')

        dependencies = dependencies or load_huggingface_dependencies()
        try:
            baseline = int(
                dependencies.torch.cuda.memory_reserved(device))
            dependencies.torch.cuda.reset_peak_memory_stats(device)
            processor = dependencies.processor_class.from_pretrained(
                str(model_path), local_files_only=True)
            model = dependencies.model_class.from_pretrained(
                str(model_path), local_files_only=True)
            model = model.to(device)
            model.eval()
        except (AttributeError, OSError, RuntimeError, TypeError, ValueError) \
                as error:
            raise RuntimeError(
                'Grounding DINO local model initialization failed') from error
        return cls(
            processor=processor,
            model=model,
            dependencies=dependencies,
            device=device,
            box_threshold=float(box_threshold),
            text_threshold=float(text_threshold),
            max_detections=max_detections,
            baseline_reserved_bytes=max(0, baseline),
        )

    def predict(self, image_path, normalized_query):
        if not isinstance(normalized_query, str) or not normalized_query:
            raise ValueError('normalized query must be non-empty')
        try:
            with self._dependencies.image_module.open(
                    Path(image_path)) as source:
                image = source.convert('RGB')
                width, height = image.size
                inputs = self.processor(
                    images=image,
                    text=[[normalized_query]],
                    return_tensors='pt',
                )
                inputs = inputs.to(self._device)
                with self._dependencies.torch.no_grad():
                    outputs = self.model(**inputs)
                results = self.processor.post_process_grounded_object_detection(
                    outputs,
                    inputs.input_ids,
                    threshold=self._box_threshold,
                    text_threshold=self._text_threshold,
                    target_sizes=[(height, width)],
                )
        except (OSError, RuntimeError, TypeError, ValueError) as error:
            raise RuntimeError(
                'Grounding DINO inference failed') from error
        if not isinstance(results, (list, tuple)) or len(results) != 1:
            raise ValueError('Grounding DINO must return exactly one result')
        return normalize_detection_result(
            results[0], width, height, self._max_detections)

    def synchronize(self):
        try:
            self._dependencies.torch.cuda.synchronize(self._device)
        except (AttributeError, RuntimeError) as error:
            raise RuntimeError('CUDA synchronization failed') from error

    def incremental_cuda_reserved_mib(self):
        try:
            peak = int(
                self._dependencies.torch.cuda.max_memory_reserved(
                    self._device))
        except (AttributeError, RuntimeError, TypeError, ValueError) as error:
            raise RuntimeError('CUDA memory query failed') from error
        return max(0, peak - self._baseline_reserved_bytes) / _MIB
