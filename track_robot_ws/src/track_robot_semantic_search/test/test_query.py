import numpy as np
import pytest

from track_robot_semantic_search.query import (
    CachedTextEncoder,
    normalize_query,
)


def test_normalize_query_applies_nfkc_and_collapses_whitespace():
    assert normalize_query('  fallen\u3000ｂｒａｎｃｈ\n near  path ') == (
        'fallen branch near path')


@pytest.mark.parametrize('text', ['', '   ', '\n\t'])
def test_normalize_query_rejects_empty_text(text):
    with pytest.raises(ValueError, match='must not be empty'):
        normalize_query(text)


def test_cached_encoder_reuses_same_query_and_version():
    calls = []

    def encode(text):
        calls.append(text)
        return np.asarray([1.0, 2.0], dtype=np.float32)

    cache = CachedTextEncoder('clip:test', encode)

    first = cache.encode('fallen   branch', 7)
    second = cache.encode('fallen branch', 7)

    assert first is second
    assert calls == ['fallen branch']
    assert cache.cache_hits == 1
    assert cache.cache_misses == 1


def test_cached_encoder_invalidates_on_version_change():
    calls = []

    def encode(text):
        calls.append(text)
        return np.asarray([len(calls)], dtype=np.float32)

    cache = CachedTextEncoder('clip:test', encode)

    first = cache.encode('branch', 1)
    second = cache.encode('branch', 2)

    assert first is not second
    assert calls == ['branch', 'branch']
    assert second.query_version == 2


def test_encoder_id_is_part_of_encoding_identity():
    left = CachedTextEncoder(
        'clip:left', lambda _: np.asarray([1.0], dtype=np.float32))
    right = CachedTextEncoder(
        'clip:right', lambda _: np.asarray([1.0], dtype=np.float32))

    assert left.encode('branch', 1).cache_key != right.encode('branch', 1).cache_key


@pytest.mark.parametrize('vector', [
    np.asarray([], dtype=np.float32),
    np.asarray([[1.0]], dtype=np.float32),
    np.asarray([np.nan], dtype=np.float32),
])
def test_cached_encoder_rejects_invalid_vectors(vector):
    cache = CachedTextEncoder('clip:test', lambda _: vector)

    with pytest.raises(ValueError, match='finite one-dimensional'):
        cache.encode('branch', 1)


def test_cached_encoder_rejects_negative_query_version():
    cache = CachedTextEncoder(
        'clip:test', lambda _: np.asarray([1.0], dtype=np.float32))

    with pytest.raises(ValueError, match='non-negative'):
        cache.encode('branch', -1)
