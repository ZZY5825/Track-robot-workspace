"""Fail-closed Phase 4B runtime-mode definitions."""

from dataclasses import dataclass
from enum import Enum


class RuntimeMode(Enum):
    PLANNING_ONLY = 'PLANNING_ONLY'
    MANUAL_NAV2_ACTIVE = 'MANUAL_NAV2_ACTIVE'
    SEMANTIC_SHADOW = 'SEMANTIC_SHADOW'
    SEMANTIC_ACTIVE = 'SEMANTIC_ACTIVE'
    ROTATION_ONLY_ACTIVE = 'ROTATION_ONLY_ACTIVE'

    @classmethod
    def parse(cls, value):
        try:
            return cls(str(value).strip().upper())
        except ValueError as error:
            choices = ', '.join(item.value for item in cls)
            raise ValueError(
                'invalid runtime_mode {!r}; expected one of {}'.format(
                    value, choices)
            ) from error


@dataclass(frozen=True)
class ModeSpec:
    planner: bool
    controller: bool
    bt_navigator: bool
    recoveries: bool
    safety_chain: bool
    semantic_adapter: bool


_MODE_SPECS = {
    RuntimeMode.PLANNING_ONLY: ModeSpec(
        planner=True,
        controller=False,
        bt_navigator=False,
        recoveries=False,
        safety_chain=False,
        semantic_adapter=False,
    ),
    RuntimeMode.MANUAL_NAV2_ACTIVE: ModeSpec(
        planner=True,
        controller=True,
        bt_navigator=True,
        recoveries=True,
        safety_chain=True,
        semantic_adapter=False,
    ),
    RuntimeMode.SEMANTIC_SHADOW: ModeSpec(
        planner=True,
        controller=False,
        bt_navigator=False,
        recoveries=False,
        safety_chain=False,
        semantic_adapter=True,
    ),
    RuntimeMode.SEMANTIC_ACTIVE: ModeSpec(
        planner=True,
        controller=True,
        bt_navigator=True,
        recoveries=True,
        safety_chain=True,
        semantic_adapter=True,
    ),
    RuntimeMode.ROTATION_ONLY_ACTIVE: ModeSpec(
        planner=False,
        controller=True,
        bt_navigator=False,
        recoveries=True,
        safety_chain=True,
        semantic_adapter=False,
    ),
}


def mode_spec(mode):
    if not isinstance(mode, RuntimeMode):
        mode = RuntimeMode.parse(mode)
    return _MODE_SPECS[mode]


def validate_mode_request(
        mode,
        enable_semantic_execution,
        enable_rotation_execution=False):
    if not isinstance(mode, RuntimeMode):
        mode = RuntimeMode.parse(mode)
    if (
            mode is RuntimeMode.SEMANTIC_ACTIVE
            and not bool(enable_semantic_execution)):
        raise ValueError(
            'SEMANTIC_ACTIVE requires enable_semantic_execution=true')
    if (
            mode is RuntimeMode.ROTATION_ONLY_ACTIVE and
            not bool(enable_rotation_execution)):
        raise ValueError(
            'ROTATION_ONLY_ACTIVE requires '
            'enable_rotation_execution=true')
