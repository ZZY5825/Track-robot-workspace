from dataclasses import dataclass
from typing import Mapping, Tuple


@dataclass(frozen=True)
class TeacherDetection:
    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    label: str


@dataclass(frozen=True)
class TeacherIdentity:
    candidate_id: str
    implementation: str
    code_revision: str
    checkpoint_id: str
    checkpoint_sha256: str
    licence: str
    platform: Mapping[str, str]
    input_size: Tuple[int, int]
