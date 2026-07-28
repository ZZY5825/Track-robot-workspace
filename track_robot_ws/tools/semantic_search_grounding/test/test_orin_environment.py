from pathlib import Path


def _platform(architecture='aarch64', l4t_release='35.1.0'):
    return {
        'architecture': architecture,
        'l4t_release': l4t_release,
        'os': 'Linux-5.10.104-tegra-aarch64',
        'python': '3.8.10',
    }


def _dependencies(runtime_path, clip_runtime_path, cuda=True,
                  ultralytics_version='8.2.103',
                  yolo_world_available=True,
                  isolated=True):
    ultralytics_origin = (
        runtime_path / 'ultralytics' / '__init__.py'
        if isolated else
        Path('/home/user/.local/lib/python3.8/site-packages/'
             'ultralytics/__init__.py'))
    return {
        'torch': {
            'available': True,
            'version': '1.13.0a0+936e9305.nv22.11',
            'cuda_available': cuda,
            'device_name': 'Orin' if cuda else '',
            'origin': '/home/user/.local/lib/python3.8/site-packages/'
                      'torch/__init__.py',
        },
        'ultralytics': {
            'available': True,
            'version': ultralytics_version,
            'yolo_world_available': yolo_world_available,
            'origin': str(ultralytics_origin),
        },
        'clip': {
            'available': True,
            'version': '1.0',
            'origin': str(clip_runtime_path / 'clip' / '__init__.py'),
        },
    }


def _paths(tmp_path):
    runtime = tmp_path / 'r0c-runtime'
    clip_runtime = tmp_path / 'clip-runtime'
    runtime.mkdir()
    clip_runtime.mkdir()
    (runtime / 'ultralytics').mkdir()
    (clip_runtime / 'clip').mkdir()
    world = tmp_path / 'yolov8s-worldv2.pt'
    clip = tmp_path / 'ViT-B-32.pt'
    world.write_bytes(b'world')
    clip.write_bytes(b'clip')
    return runtime, clip_runtime, world, clip


def _probe(tmp_path, platform=None, dependency_builder=None):
    from tools.semantic_search_grounding.orin_environment import (
        probe_orin_environment,
    )

    runtime, clip_runtime, world, clip = _paths(tmp_path)
    builder = dependency_builder or (
        lambda runtime_value, clip_runtime_value: _dependencies(
            runtime_value, clip_runtime_value))
    result = probe_orin_environment(
        runtime_path=runtime,
        clip_runtime_path=clip_runtime,
        world_checkpoint=world,
        clip_checkpoint=clip,
        platform_probe=lambda: platform or _platform(),
        dependency_probe=lambda _runtime, _clip_runtime: builder(
            runtime, clip_runtime),
    )
    return result, runtime, clip_runtime


def test_rejects_non_orin_architecture_before_dependency_probe(tmp_path):
    from tools.semantic_search_grounding.orin_environment import (
        probe_orin_environment,
    )

    calls = []
    runtime, clip_runtime, world, clip = _paths(tmp_path)
    result = probe_orin_environment(
        runtime_path=runtime,
        clip_runtime_path=clip_runtime,
        world_checkpoint=world,
        clip_checkpoint=clip,
        platform_probe=lambda: _platform('x86_64'),
        dependency_probe=lambda *_args: calls.append('dependencies'),
    )

    assert result.host_eligible is False
    assert result.runtime_ready is False
    assert 'architecture_not_aarch64' in result.report['reasons']
    assert calls == []


def test_rejects_wrong_l4t_release(tmp_path):
    result, _runtime, _clip_runtime = _probe(
        tmp_path, platform=_platform(l4t_release='36.4.0'))

    assert result.host_eligible is False
    assert 'l4t_release_not_35' in result.report['reasons']


def test_requires_both_isolated_runtime_directories(tmp_path):
    from tools.semantic_search_grounding.orin_environment import (
        probe_orin_environment,
    )

    missing = tmp_path / 'missing'
    world = tmp_path / 'world.pt'
    clip = tmp_path / 'clip.pt'
    world.write_bytes(b'world')
    clip.write_bytes(b'clip')
    result = probe_orin_environment(
        runtime_path=missing,
        clip_runtime_path=missing,
        world_checkpoint=world,
        clip_checkpoint=clip,
        platform_probe=lambda: _platform(),
        dependency_probe=lambda *_args: {},
    )

    assert result.host_eligible is True
    assert result.runtime_ready is False
    assert 'runtime_path_unavailable' in result.report['reasons']
    assert 'clip_runtime_path_unavailable' in result.report['reasons']


def test_rejects_global_ultralytics_shadowing(tmp_path):
    holder = {}

    def dependencies():
        return _dependencies(
            holder['runtime'], holder['clip_runtime'], isolated=False)

    runtime, clip_runtime, world, clip = _paths(tmp_path)
    holder.update(runtime=runtime, clip_runtime=clip_runtime)
    from tools.semantic_search_grounding.orin_environment import (
        probe_orin_environment,
    )
    result = probe_orin_environment(
        runtime_path=runtime,
        clip_runtime_path=clip_runtime,
        world_checkpoint=world,
        clip_checkpoint=clip,
        platform_probe=lambda: _platform(),
        dependency_probe=lambda *_args: dependencies(),
    )

    assert result.runtime_ready is False
    assert 'ultralytics_not_isolated' in result.report['reasons']


def test_requires_exact_runtime_api_cuda_and_orin_device(tmp_path):
    result, runtime, clip_runtime = _probe(
        tmp_path,
        dependency_builder=lambda runtime, clip_runtime: _dependencies(
            runtime, clip_runtime,
            cuda=False,
            ultralytics_version='8.0.239',
            yolo_world_available=False,
        ),
    )

    assert result.runtime_ready is False
    assert 'ultralytics_version_mismatch' in result.report['reasons']
    assert 'yolo_world_unavailable' in result.report['reasons']
    assert 'torch_cuda_unavailable' in result.report['reasons']


def test_rejects_checkpoint_symlink(tmp_path):
    from tools.semantic_search_grounding.orin_environment import (
        probe_orin_environment,
    )

    runtime, clip_runtime, world, clip = _paths(tmp_path)
    link = tmp_path / 'world-link.pt'
    link.symlink_to(world)
    result = probe_orin_environment(
        runtime_path=runtime,
        clip_runtime_path=clip_runtime,
        world_checkpoint=link,
        clip_checkpoint=clip,
        platform_probe=lambda: _platform(),
        dependency_probe=lambda *_args: _dependencies(
            runtime, clip_runtime),
    )

    assert result.runtime_ready is False
    assert result.report['models']['world']['present'] is False
    assert 'world_checkpoint_unavailable' in result.report['reasons']


def test_complete_orin_runtime_is_ready_with_composite_identity(tmp_path):
    result, runtime, clip_runtime = _probe(tmp_path)

    assert result.host_eligible is True
    assert result.runtime_ready is True
    assert result.report['reasons'] == []
    assert result.report['runtime']['ultralytics']['origin'].startswith(
        str(runtime))
    assert result.report['runtime']['clip']['origin'].startswith(
        str(clip_runtime))
    assert len(result.report['models']['world']['sha256']) == 64
    assert len(result.report['models']['clip']['sha256']) == 64
    assert len(result.report['models']['composite_sha256']) == 64


def test_report_does_not_expose_environment_or_credentials(tmp_path):
    result, _runtime, _clip_runtime = _probe(tmp_path)

    serialized = repr(dict(result.report)).lower()
    assert 'token' not in serialized
    assert 'password' not in serialized
    assert 'environment' not in result.report
