import math
from typing import Any, Dict, Optional, Sequence

import numpy as np

from .evaluation import percentile


def latency_summary(values: Sequence[float]) -> Dict[str, float]:
    if not values:
        raise ValueError('latencies must not be empty')
    latencies = [float(value) for value in values]
    if any(not math.isfinite(value) or value < 0.0 for value in latencies):
        raise ValueError('latencies must be finite and non-negative')
    return {
        'count': len(latencies),
        'mean_ms': sum(latencies) / len(latencies),
        'p50_ms': percentile(latencies, 0.50),
        'p95_ms': percentile(latencies, 0.95),
        'maximum_ms': max(latencies),
    }


def unavailable_candidate(
        candidate_id: str,
        reason: str,
        python_compatible: bool = False,
        licence_approved: bool = False) -> Dict[str, object]:
    if not candidate_id.strip() or not reason.strip():
        raise ValueError('candidate_id and reason must not be empty')
    return {
        'candidate_id': candidate_id,
        'available': False,
        'python_compatible': bool(python_compatible),
        'licence_approved': bool(licence_approved),
        'memory_pass': False,
        'latency_pass': False,
        'phrase_region_recall': None,
        'p95_latency_ms': 0.0,
        'accuracy_status': 'not_evaluated',
        'reason': reason,
    }


def available_candidate(
        candidate_id: str,
        p95_latency_ms: float,
        peak_memory_mb: float,
        memory_limit_mb: float,
        phrase_region_recall: Optional[float],
        latency_limit_ms: float = 150.0) -> Dict[str, object]:
    if not candidate_id.strip():
        raise ValueError('candidate_id must not be empty')
    latency = float(p95_latency_ms)
    memory = float(peak_memory_mb)
    memory_limit = float(memory_limit_mb)
    if any(not math.isfinite(value) or value < 0.0 for value in (
            latency, memory, memory_limit)):
        raise ValueError('candidate metrics must be finite and non-negative')
    if phrase_region_recall is not None:
        recall = float(phrase_region_recall)
        if not math.isfinite(recall) or not 0.0 <= recall <= 1.0:
            raise ValueError('phrase_region_recall is outside its valid range')
    else:
        recall = None
    return {
        'candidate_id': candidate_id.strip(),
        'available': True,
        'python_compatible': True,
        'licence_approved': True,
        'memory_pass': memory <= memory_limit,
        'latency_pass': latency <= float(latency_limit_ms),
        'phrase_region_recall': recall,
        'p95_latency_ms': latency,
        'accuracy_status': (
            'evaluated' if recall is not None else 'not_evaluated'),
        'peak_memory_mb': memory,
        'memory_limit_mb': memory_limit,
    }


def benchmark_aligned_encoder(
        adapter: Any,
        images: Sequence[np.ndarray],
        query_text: str) -> Dict[str, object]:
    if not images:
        raise ValueError('images must not be empty')
    text = np.asarray(adapter.encode_text(query_text), dtype=np.float32)
    if text.ndim != 1 or not np.isfinite(text).all():
        raise ValueError('text embedding must be a finite vector')
    text_norm = float(np.linalg.norm(text))
    if text_norm <= 1e-12:
        raise ValueError('text embedding norm must be positive')
    latencies = []
    scores = []
    for image in images:
        encoding = adapter.encode_image_grid(np.asarray(image))
        embeddings = np.asarray(encoding.embeddings, dtype=np.float32)
        valid = np.asarray(encoding.valid_patch_mask, dtype=bool)
        if embeddings.ndim != 3 or embeddings.shape[:2] != valid.shape or (
                embeddings.shape[2] != text.shape[0]):
            raise ValueError('aligned encoding has an invalid shape')
        norms = np.linalg.norm(embeddings, axis=2) * text_norm
        score_map = np.sum(
            embeddings * text.reshape(1, 1, -1), axis=2) / np.maximum(
                norms, 1e-12)
        scores.extend(float(value) for value in score_map[valid])
        latencies.append(float(encoding.inference_ms))
    if not scores:
        raise ValueError('aligned encoding has no valid image cells')
    return {
        'encoder_id': str(adapter.encoder_id),
        'checkpoint_id': str(adapter.checkpoint_id),
        'query_text': query_text,
        'observation_count': len(images),
        'image_latency': latency_summary(latencies),
        'score_summary': {
            'minimum': min(scores),
            'mean': sum(scores) / len(scores),
            'maximum': max(scores),
        },
    }
