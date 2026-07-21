from dataclasses import dataclass
import json

from .query import normalize_query


@dataclass(frozen=True)
class ActiveQuery:
    query_text: str
    query_id: int
    query_version: int


def parse_query_payload(payload: str) -> ActiveQuery:
    try:
        value = json.loads(payload)
    except (TypeError, json.JSONDecodeError) as exc:
        raise ValueError('query payload must be valid JSON') from exc
    if not isinstance(value, dict):
        raise ValueError('query payload must be a JSON object')
    if 'query_text' not in value:
        raise ValueError('query_text is required')
    query_text = normalize_query(value['query_text'])
    identifiers = {}
    for field in ('query_id', 'query_version'):
        item = value.get(field)
        if not isinstance(item, int) or isinstance(item, bool) or item < 0:
            raise ValueError('{} must be a non-negative integer'.format(field))
        identifiers[field] = item
    return ActiveQuery(
        query_text=query_text,
        query_id=identifiers['query_id'],
        query_version=identifiers['query_version'])
