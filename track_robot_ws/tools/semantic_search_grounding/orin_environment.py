import hashlib
import json
import os
import platform
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Callable, Mapping, Optional


SCHEMA_VERSION = '1.0.0'
HOST_ROLE = 'r0c_orin_candidate'
ULTRALYTICS_VERSION = '8.2.103'
_MAX_OUTPUT = 16384
_L4T_RELEASE = re.compile(r'# R(\d+)')


@dataclass(frozen=True)
class OrinEnvironmentProbe:
    report: Mapping[str, object]

    @property
    def host_eligible(self) -> bool:
        return self.report['host_eligible'] is True

    @property
    def runtime_ready(self) -> bool:
        return self.report['runtime_ready'] is True


def _default_platform_probe():
    l4t_release = ''
    try:
        first_line = Path('/etc/nv_tegra_release').read_text(
            encoding='utf-8').splitlines()[0]
    except (OSError, UnicodeError, IndexError):
        pass
    else:
        match = _L4T_RELEASE.search(first_line)
        if match:
            l4t_release = match.group(1)
    return {
        'architecture': platform.machine(),
        'l4t_release': l4t_release,
        'os': platform.platform(),
        'python': platform.python_version(),
    }


def _safe_directory(path):
    path = Path(path)
    return not path.is_symlink() and path.is_dir()


def _inside(path_value, root):
    try:
        path = Path(path_value).resolve()
        root = Path(root).resolve()
        return os.path.commonpath((str(path), str(root))) == str(root)
    except (OSError, TypeError, ValueError):
        return False


def _default_dependency_probe(runtime_path, clip_runtime_path):
    script = r'''
import importlib.metadata
import json
report = {}
try:
    import torch
    cuda_available = bool(torch.cuda.is_available())
    device_name = torch.cuda.get_device_name(0) if cuda_available else ''
    report['torch'] = {
        'available': True,
        'version': str(torch.__version__),
        'cuda_available': cuda_available,
        'device_name': str(device_name),
        'origin': str(torch.__file__),
    }
except Exception:
    report['torch'] = {
        'available': False, 'version': '', 'cuda_available': False,
        'device_name': '', 'origin': '',
    }
try:
    import ultralytics
    try:
        from ultralytics import YOLOWorld
        yolo_world_available = YOLOWorld is not None
    except Exception:
        yolo_world_available = False
    report['ultralytics'] = {
        'available': True,
        'version': str(ultralytics.__version__),
        'yolo_world_available': yolo_world_available,
        'origin': str(ultralytics.__file__),
    }
except Exception:
    report['ultralytics'] = {
        'available': False, 'version': '',
        'yolo_world_available': False, 'origin': '',
    }
try:
    import clip
    try:
        version = importlib.metadata.version('clip')
    except importlib.metadata.PackageNotFoundError:
        version = ''
    report['clip'] = {
        'available': True,
        'version': str(version),
        'origin': str(clip.__file__),
    }
except Exception:
    report['clip'] = {'available': False, 'version': '', 'origin': ''}
print('R0C_JSON:' + json.dumps(report, sort_keys=True))
'''
    environment = dict(os.environ)
    current_pythonpath = environment.get('PYTHONPATH', '')
    values = [str(Path(runtime_path)), str(Path(clip_runtime_path))]
    if current_pythonpath:
        values.append(current_pythonpath)
    environment['PYTHONPATH'] = os.pathsep.join(values)
    environment['MPLCONFIGDIR'] = '/tmp/matplotlib-r0c-probe'
    environment['YOLO_CONFIG_DIR'] = '/tmp/ultralytics-r0c-probe'
    try:
        result = subprocess.run(
            [sys.executable, '-c', script],
            check=False,
            capture_output=True,
            text=True,
            timeout=30.0,
            env=environment,
        )
    except (OSError, subprocess.SubprocessError):
        return {}
    for line in reversed(result.stdout[-_MAX_OUTPUT:].splitlines()):
        if line.startswith('R0C_JSON:'):
            try:
                value = json.loads(line[len('R0C_JSON:'):])
            except json.JSONDecodeError:
                return {}
            return value if isinstance(value, dict) else {}
    return {}


def _sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _checkpoint_report(path):
    path = Path(path)
    report = {'present': False, 'sha256': '', 'filename': path.name}
    try:
        if path.is_symlink() or not path.is_file():
            return report
        report['sha256'] = _sha256_file(path)
    except OSError:
        return report
    report['present'] = True
    return report


def _runtime_value(dependencies, name, keys):
    source = dependencies.get(name, {})
    value = {}
    for key, default in keys:
        item = source.get(key, default)
        value[key] = item if isinstance(default, bool) else str(item)[:512]
    return value


def probe_orin_environment(
        runtime_path,
        clip_runtime_path,
        world_checkpoint,
        clip_checkpoint,
        platform_probe: Optional[Callable[[], Mapping[str, str]]] = None,
        dependency_probe: Optional[Callable[[Path, Path], Mapping[
            str, Mapping[str, object]]]] = None,
        ) -> OrinEnvironmentProbe:
    runtime_path = Path(runtime_path)
    clip_runtime_path = Path(clip_runtime_path)
    platform_value = dict((platform_probe or _default_platform_probe)())
    architecture = str(platform_value.get('architecture', ''))[:64]
    l4t_release = str(platform_value.get('l4t_release', ''))[:64]
    os_name = str(platform_value.get('os', ''))[:256]
    python_version = str(platform_value.get('python', ''))[:64]
    host_eligible = architecture == 'aarch64' and l4t_release.startswith('35')

    runtime_present = _safe_directory(runtime_path)
    clip_runtime_present = _safe_directory(clip_runtime_path)
    dependencies = {}
    if host_eligible and runtime_present and clip_runtime_present:
        dependencies = dict(
            (dependency_probe or _default_dependency_probe)(
                runtime_path, clip_runtime_path))
    runtime = {
        'torch': _runtime_value(dependencies, 'torch', (
            ('available', False),
            ('version', ''),
            ('cuda_available', False),
            ('device_name', ''),
            ('origin', ''),
        )),
        'ultralytics': _runtime_value(dependencies, 'ultralytics', (
            ('available', False),
            ('version', ''),
            ('yolo_world_available', False),
            ('origin', ''),
        )),
        'clip': _runtime_value(dependencies, 'clip', (
            ('available', False),
            ('version', ''),
            ('origin', ''),
        )),
    }
    world = _checkpoint_report(world_checkpoint)
    clip = _checkpoint_report(clip_checkpoint)
    composite = ''
    if world['present'] and clip['present']:
        composite = hashlib.sha256(
            '{}:{}'.format(world['sha256'], clip['sha256']).encode(
                'ascii')).hexdigest()

    reasons = []
    if architecture != 'aarch64':
        reasons.append('architecture_not_aarch64')
    if not l4t_release.startswith('35'):
        reasons.append('l4t_release_not_35')
    if host_eligible:
        if not runtime_present:
            reasons.append('runtime_path_unavailable')
        if not clip_runtime_present:
            reasons.append('clip_runtime_path_unavailable')
        if runtime_present and clip_runtime_present:
            if not runtime['torch']['available']:
                reasons.append('torch_unavailable')
            if not runtime['torch']['version'].startswith('1.13.0a0'):
                reasons.append('torch_version_mismatch')
            if not runtime['torch']['cuda_available']:
                reasons.append('torch_cuda_unavailable')
            if 'orin' not in runtime['torch']['device_name'].lower():
                reasons.append('cuda_device_not_orin')
            if not runtime['ultralytics']['available']:
                reasons.append('ultralytics_unavailable')
            if runtime['ultralytics']['version'] != ULTRALYTICS_VERSION:
                reasons.append('ultralytics_version_mismatch')
            if not runtime['ultralytics']['yolo_world_available']:
                reasons.append('yolo_world_unavailable')
            if not _inside(
                    runtime['ultralytics']['origin'], runtime_path):
                reasons.append('ultralytics_not_isolated')
            if not runtime['clip']['available']:
                reasons.append('clip_unavailable')
            if not _inside(runtime['clip']['origin'], clip_runtime_path):
                reasons.append('clip_not_isolated')
        if not world['present']:
            reasons.append('world_checkpoint_unavailable')
        if not clip['present']:
            reasons.append('clip_checkpoint_unavailable')

    report = {
        'schema_version': SCHEMA_VERSION,
        'host_role': HOST_ROLE,
        'architecture': architecture,
        'l4t_release': l4t_release,
        'os': os_name,
        'python': python_version,
        'paths': {
            'runtime_present': runtime_present,
            'clip_runtime_present': clip_runtime_present,
        },
        'runtime': runtime,
        'models': {
            'world': world,
            'clip': clip,
            'composite_sha256': composite,
        },
        'host_eligible': host_eligible,
        'runtime_ready': host_eligible and not reasons,
        'reasons': reasons,
    }
    return OrinEnvironmentProbe(report=MappingProxyType(report))
