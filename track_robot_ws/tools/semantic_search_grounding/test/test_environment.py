from pathlib import Path


def _platform(architecture='x86_64'):
    return {
        'architecture': architecture,
        'os': 'Ubuntu 22.04.5 LTS',
        'python': '3.11.9',
    }


def _modules(cuda=True):
    return {
        'torch': {
            'available': True,
            'version': '2.4.1',
            'cuda_available': cuda,
        },
        'transformers': {
            'available': True,
            'version': '4.57.0',
        },
        'pillow': {
            'available': True,
            'version': '10.4.0',
        },
        'opencv': {
            'available': True,
            'version': '4.10.0',
        },
    }


def _nvidia_smi_success(_arguments):
    return {
        'returncode': 0,
        'stdout': 'NVIDIA RTX 4090, 24564, 550.90.07\n',
        'stderr': '',
    }


def test_rejects_aarch64_jetson_as_desktop_teacher_host():
    from tools.semantic_search_grounding.environment import probe_environment

    result = probe_environment(
        platform_probe=lambda: _platform('aarch64'),
        command_runner=lambda _arguments: {
            'returncode': 127, 'stdout': '', 'stderr': 'not found'},
        module_probe=lambda: {},
    )

    assert result.host_eligible is False
    assert result.runtime_ready is False
    assert 'architecture_not_x86_64' in result.report['reasons']
    assert result.report['architecture'] == 'aarch64'


def test_rejects_x86_desktop_without_visible_nvidia_gpu():
    from tools.semantic_search_grounding.environment import probe_environment

    result = probe_environment(
        platform_probe=lambda: _platform(),
        command_runner=lambda _arguments: {
            'returncode': 1, 'stdout': '', 'stderr': 'driver unavailable'},
        module_probe=lambda: _modules(),
    )

    assert result.host_eligible is False
    assert 'nvidia_smi_unavailable' in result.report['reasons']


def test_desktop_without_python_runtime_is_eligible_but_not_ready():
    from tools.semantic_search_grounding.environment import probe_environment

    result = probe_environment(
        platform_probe=lambda: _platform(),
        command_runner=_nvidia_smi_success,
        module_probe=lambda: {
            'torch': {'available': False, 'version': '', 'cuda_available': False},
            'transformers': {'available': False, 'version': ''},
            'pillow': {'available': False, 'version': ''},
            'opencv': {'available': False, 'version': ''},
        },
    )

    assert result.host_eligible is True
    assert result.runtime_ready is False
    assert result.report['gpu']['devices'][0]['name'] == 'NVIDIA RTX 4090'
    assert result.report['gpu']['devices'][0]['memory_total_mib'] == 24564
    assert 'torch_unavailable' in result.report['reasons']
    assert 'transformers_unavailable' in result.report['reasons']
    assert 'pillow_unavailable' in result.report['reasons']
    assert 'opencv_unavailable' in result.report['reasons']


def test_complete_local_desktop_runtime_is_ready(tmp_path):
    from tools.semantic_search_grounding.environment import probe_environment

    model_dir = tmp_path / 'grounding-dino-tiny'
    model_dir.mkdir()
    checkpoint = model_dir / 'model.safetensors'
    checkpoint.write_bytes(b'checkpoint')

    result = probe_environment(
        model_dir=model_dir,
        checkpoint_file=Path('model.safetensors'),
        platform_probe=lambda: _platform(),
        command_runner=_nvidia_smi_success,
        module_probe=lambda: _modules(),
    )

    assert result.host_eligible is True
    assert result.runtime_ready is True
    assert result.report['reasons'] == []
    assert result.report['model']['model_dir_present'] is True
    assert result.report['model']['checkpoint_present'] is True
    assert len(result.report['model']['checkpoint_sha256']) == 64


def test_report_does_not_expose_environment_or_credentials():
    from tools.semantic_search_grounding.environment import probe_environment

    result = probe_environment(
        platform_probe=lambda: _platform('aarch64'),
        command_runner=lambda _arguments: {
            'returncode': 127, 'stdout': '', 'stderr': 'secret-token'},
        module_probe=lambda: {},
    )

    serialized = repr(dict(result.report))
    assert 'secret-token' not in serialized
    assert 'environment' not in result.report
    assert 'env' not in result.report
    assert set(result.report) == {
        'schema_version', 'host_role', 'architecture', 'os', 'python',
        'gpu', 'runtime', 'model', 'host_eligible', 'runtime_ready',
        'reasons',
    }
