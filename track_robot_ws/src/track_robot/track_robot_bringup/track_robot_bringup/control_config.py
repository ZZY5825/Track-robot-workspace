"""ROS-independent configuration for semantic-search bringup."""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Mapping, Optional, Union


@dataclass(frozen=True)
class StageSpec:
    """The hardware and feature modules required by one supported stage."""

    name: str
    camera: bool
    lidar: bool
    base: bool
    imu: bool
    phase1: bool
    localization: bool
    tracklets: bool
    memory: bool


@dataclass(frozen=True)
class HardwareSelection:
    """Whether bringup should start each hardware module itself."""

    camera: bool
    lidar: bool
    base: bool
    imu: bool


STAGES = {
    'sensors': StageSpec(
        'sensors', True, True, True, True, False, False, False, False),
    'phase1': StageSpec(
        'phase1', True, False, False, False, True, False, False, False),
    'phase2': StageSpec(
        'phase2', True, True, True, True, True, True, True, True),
}


def resolve_stage(name: str) -> StageSpec:
    """Return the immutable policy for a supported stage name."""

    try:
        return STAGES[name]
    except KeyError as error:
        supported = ', '.join(sorted(STAGES))
        raise ValueError(
            "unknown semantic-search stage {!r}; expected one of {}".format(
                name, supported)) from error


def managed_environment(
        base: Mapping[str, str],
        domain_id: int = 20,
        dds_profile: Optional[Union[str, Path]] = None) -> Dict[str, str]:
    """Copy a string environment and apply bringup's fixed Domain 20."""

    if type(domain_id) is not int or domain_id != 20:
        raise ValueError('managed ROS domain must be 20')
    if any(not isinstance(key, str) or not isinstance(value, str)
           for key, value in base.items()):
        raise ValueError('base environment keys and values must be strings')

    result = dict(base)
    result['ROS_DOMAIN_ID'] = '20'
    if dds_profile:
        result['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(dds_profile)
    return result


def default_workspace_paths(workspace_root: Union[str, Path]) -> Dict[str, Path]:
    """Return the workspace-relative model and bringup configuration paths."""

    root = Path(workspace_root)
    config = root / 'src' / 'track_robot' / 'track_robot_bringup' / 'config'
    return {
        'workspace_root': root,
        'runtime_path': root / 'models' / 'phase1_runtime' / 'python',
        'checkpoint_path': root / 'models' / 'phase1' / 'ViT-B-32.pt',
        'defaults_path': config / 'semantic_search_defaults.yaml',
        'dds_profile': config / 'fastdds_semantic_search.xml',
    }
