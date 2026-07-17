from dataclasses import dataclass
import unicodedata
from typing import Callable, Optional, Tuple

import numpy as np


def normalize_query(text: str) -> str:
    if not isinstance(text, str):
        raise TypeError('query text must be a string')
    normalized = unicodedata.normalize('NFKC', text)
    normalized = ' '.join(normalized.split())
    if not normalized:
        raise ValueError('query text must not be empty')
    return normalized


@dataclass(frozen=True)
class QueryEncoding:
    encoder_id: str
    normalized_text: str
    query_version: int
    vector: np.ndarray

    @property
    def cache_key(self) -> Tuple[str, str, int]:
        return self.encoder_id, self.normalized_text, self.query_version


class CachedTextEncoder:
    def __init__(
            self,
            encoder_id: str,
            encode_fn: Callable[[str], np.ndarray]):
        if not isinstance(encoder_id, str) or not encoder_id.strip():
            raise ValueError('encoder_id must not be empty')
        if not callable(encode_fn):
            raise TypeError('encode_fn must be callable')
        self._encoder_id = encoder_id.strip()
        self._encode_fn = encode_fn
        self._cached: Optional[QueryEncoding] = None
        self.cache_hits = 0
        self.cache_misses = 0

    @property
    def encoder_id(self) -> str:
        return self._encoder_id

    def encode(self, text: str, query_version: int) -> QueryEncoding:
        if not isinstance(query_version, int) or isinstance(query_version, bool):
            raise TypeError('query_version must be an integer')
        if query_version < 0:
            raise ValueError('query_version must be non-negative')
        normalized = normalize_query(text)
        key = self._encoder_id, normalized, query_version
        if self._cached is not None and self._cached.cache_key == key:
            self.cache_hits += 1
            return self._cached

        vector = np.asarray(self._encode_fn(normalized), dtype=np.float32)
        if vector.ndim != 1 or vector.size == 0 or not np.isfinite(vector).all():
            raise ValueError(
                'text encoder must return a finite one-dimensional vector')
        vector = vector.copy()
        vector.setflags(write=False)
        encoding = QueryEncoding(
            encoder_id=self._encoder_id,
            normalized_text=normalized,
            query_version=query_version,
            vector=vector)
        self._cached = encoding
        self.cache_misses += 1
        return encoding

    def clear(self) -> None:
        self._cached = None
