import hashlib
import importlib
import platform
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from types import MappingProxyType
from typing import Callable, Mapping, Optional


SCHEMA_VERSION = '1.0.0'
HOST_ROLE = 'r0b_desktop_teacher'
_GPU_QUERY = (
    'name,memory.total,driver_version',
)
_GPU_FORMAT = 'csv,noheader,nounits'
_MAX_COMMAND_OUTPUT = 4096


@dataclass(frozen=True)
class EnvironmentProbe:
    report: Mapping[str, object]

    @property
    def host_eligible(self) -> bool:
        return self.report['host_eligible'] is True

    @property
    def runtime_ready(self) -> bool:
        return self.report['runtime_ready'] is True


def _default_platform_probe() -> Mapping[str, str]:
    return {
        'architecture': platform.machine(),
        'os': platform.platform(),
        'python': platform.python_version(),
    }


def _default_command_runner(arguments):
    try:
        result = subprocess.run(
            arguments,
            check=False,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except (OSError, subprocess.SubprocessError):
        return {'returncode': 127, 'stdout': '', 'stderr': ''}
    return {
        'returncode': result.returncode,
        'stdout': result.stdout[:_MAX_COMMAND_OUTPUT],
        'stderr': '',
    }


def _module_version(module, fallback='') -> str:
    value = getattr(module, '__version__', fallback)
    return str(value)[:256] if value else ''


def _default_module_probe() -> Mapping[str, Mapping[str, object]]:
    report = {}
    try:
        torch = importlib.import_module('torch')
    except (ImportError, OSError):
        report['torch'] = {
            'available': False,
            'version': '',
            'cuda_available': False,
        }
    else:
        try:
            cuda_available = bool(torch.cuda.is_available())
        except (AttributeError, RuntimeError):
            cuda_available = False
        report['torch'] = {
            'available': True,
            'version': _module_version(torch),
            'cuda_available': cuda_available,
        }

    for import_name, report_name in (
            ('transformers', 'transformers'),
            ('PIL', 'pillow'),
            ('cv2', 'opencv')):
        try:
            module = importlib.import_module(import_name)
        except (ImportError, OSError):
            report[report_name] = {'available': False, 'version': ''}
        else:
            report[report_name] = {
                'available': True,
                'version': _module_version(module),
            }
    return report


def _parse_gpu_rows(value: str):
    devices = []
    for raw_line in value[:_MAX_COMMAND_OUTPUT].splitlines():
        parts = [part.strip() for part in raw_line.split(',')]
        if len(parts) != 3:
            continue
        try:
            memory_total_mib = int(parts[1])
        except ValueError:
            continue
        if not parts[0] or not parts[2] or memory_total_mib <= 0:
            continue
        devices.append({
            'name': parts[0][:256],
            'memory_total_mib': memory_total_mib,
            'driver_version': parts[2][:64],
        })
    return devices


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _model_report(
        model_dir: Optional[Path],
        checkpoint_file: Optional[Path]):
    report = {
        'model_dir_present': False,
        'checkpoint_present': False,
        'checkpoint_sha256': '',
    }
    if model_dir is None:
        return report
    model_dir = Path(model_dir)
    if model_dir.is_symlink() or not model_dir.is_dir():
        return report
    report['model_dir_present'] = True
    if checkpoint_file is None:
        return report
    relative = PurePosixPath(str(checkpoint_file))
    if relative.is_absolute() or '..' in relative.parts:
        return report
    checkpoint_path = model_dir / relative
    try:
        if checkpoint_path.is_symlink() or not checkpoint_path.is_file():
            return report
        report['checkpoint_sha256'] = _sha256_file(checkpoint_path)
    except OSError:
        return report
    report['checkpoint_present'] = True
    return report


def _runtime_value(
        modules: Mapping[str, Mapping[str, object]],
        name: str,
        cuda=False):
    value = modules.get(name, {})
    result = {
        'available': value.get('available') is True,
        'version': str(value.get('version', ''))[:256],
    }
    if cuda:
        result['cuda_available'] = value.get('cuda_available') is True
    return result


def probe_environment(
        model_dir: Optional[Path] = None,
        checkpoint_file: Optional[Path] = None,
        platform_probe: Optional[Callable[[], Mapping[str, str]]] = None,
        command_runner: Optional[Callable[[object], Mapping[str, object]]] = None,
        module_probe: Optional[
            Callable[[], Mapping[str, Mapping[str, object]]]] = None,
        ) -> EnvironmentProbe:
    platform_value = dict(
        (platform_probe or _default_platform_probe)())
    architecture = str(platform_value.get('architecture', ''))[:64]
    os_name = str(platform_value.get('os', ''))[:256]
    python_version = str(platform_value.get('python', ''))[:64]

    command = command_runner or _default_command_runner
    command_result = command([
        'nvidia-smi',
        '--query-gpu={}'.format(','.join(_GPU_QUERY)),
        '--format={}'.format(_GPU_FORMAT),
    ])
    devices = (
        _parse_gpu_rows(str(command_result.get('stdout', '')))
        if command_result.get('returncode') == 0 else [])
    host_eligible = architecture in {'x86_64', 'amd64'} and bool(devices)

    modules = (
        (module_probe or _default_module_probe)()
        if host_eligible else {})
    runtime = {
        'torch': _runtime_value(modules, 'torch', cuda=True),
        'transformers': _runtime_value(modules, 'transformers'),
        'pillow': _runtime_value(modules, 'pillow'),
        'opencv': _runtime_value(modules, 'opencv'),
    }
    model = _model_report(model_dir, checkpoint_file)

    reasons = []
    if architecture not in {'x86_64', 'amd64'}:
        reasons.append('architecture_not_x86_64')
    if not devices:
        reasons.append('nvidia_smi_unavailable')
    if host_eligible:
        for name in ('torch', 'transformers', 'pillow', 'opencv'):
            if not runtime[name]['available']:
                reasons.append('{}_unavailable'.format(name))
        if (runtime['torch']['available'] and
                not runtime['torch']['cuda_available']):
            reasons.append('torch_cuda_unavailable')
        if not model['model_dir_present']:
            reasons.append('model_dir_unavailable')
        if not model['checkpoint_present']:
            reasons.append('checkpoint_unavailable')

    runtime_ready = host_eligible and not reasons
    report = {
        'schema_version': SCHEMA_VERSION,
        'host_role': HOST_ROLE,
        'architecture': architecture,
        'os': os_name,
        'python': python_version or '{}.{}.{}'.format(*sys.version_info[:3]),
        'gpu': {
            'nvidia_smi_available': bool(devices),
            'devices': devices,
        },
        'runtime': runtime,
        'model': model,
        'host_eligible': host_eligible,
        'runtime_ready': runtime_ready,
        'reasons': reasons,
    }
    return EnvironmentProbe(report=MappingProxyType(report))
