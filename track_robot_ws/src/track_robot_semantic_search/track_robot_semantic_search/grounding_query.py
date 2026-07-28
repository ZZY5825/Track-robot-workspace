from dataclasses import dataclass


@dataclass(frozen=True)
class GroundingQuery:
    raw_text: str
    normalized_text: str


def normalize_grounding_query(value: str) -> GroundingQuery:
    if not isinstance(value, str):
        raise ValueError('grounding query must be a string')
    if any(ord(character) < 0x20 or ord(character) > 0x7e
           for character in value):
        raise ValueError(
            'grounding query must contain printable ASCII only')
    raw = ' '.join(
        part for part in value.strip().split(' ') if part)
    if not raw or len(raw) > 160:
        raise ValueError(
            'grounding query must contain 1 to 160 characters')
    return GroundingQuery(raw_text=raw, normalized_text=raw.lower())
