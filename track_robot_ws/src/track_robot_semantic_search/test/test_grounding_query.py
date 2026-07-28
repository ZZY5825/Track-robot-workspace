import pytest

from track_robot_semantic_search.grounding_query import (
    normalize_grounding_query,
)


def test_normalizes_one_english_visible_attribute_query():
    query = normalize_grounding_query(
        '  A   tall blue cylindrical container  ')
    assert query.raw_text == 'A tall blue cylindrical container'
    assert query.normalized_text == 'a tall blue cylindrical container'


@pytest.mark.parametrize('value', [
    '',
    '   ',
    '蓝色杯子',
    'blue\ncontainer',
    'x' * 161,
    None,
])
def test_rejects_out_of_contract_query(value):
    with pytest.raises(ValueError, match='grounding query'):
        normalize_grounding_query(value)


def test_punctuation_needed_by_open_vocabulary_prompt_is_preserved():
    query = normalize_grounding_query('a blue, toothpaste-like container')
    assert query.normalized_text == 'a blue, toothpaste-like container'
