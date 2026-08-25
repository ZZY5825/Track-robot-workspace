import struct
import sys
from pathlib import Path

import numpy as np


VISUALIZATION_DIR = Path(__file__).resolve().parents[1]
if str(VISUALIZATION_DIR) not in sys.path:
    sys.path.insert(0, str(VISUALIZATION_DIR))

from human_following_3d_geometry import (  # noqa: E402
    build_visual_scene,
    convex_outer_surface,
    read_binary_stl,
    simplify_mesh,
)


def write_stl(path, triangles):
    payload = bytearray(b'test mesh'.ljust(80, b'\0'))
    payload.extend(struct.pack('<I', len(triangles)))
    for triangle in triangles:
        values = [0.0, 0.0, 1.0] + np.asarray(triangle, dtype=float).reshape(-1).tolist()
        payload.extend(struct.pack('<12fH', *values, 0))
    path.write_bytes(payload)


def test_read_binary_stl_decodes_and_deterministically_limits_faces(tmp_path):
    path = tmp_path / 'mesh.stl'
    triangles = [
        [[index, 0, 0], [index, 1, 0], [index, 0, 1]]
        for index in range(5)
    ]
    write_stl(path, triangles)

    first = read_binary_stl(path, max_faces=3, seed=11)
    second = read_binary_stl(path, max_faces=3, seed=11)

    assert first['original_face_count'] == 5
    assert first['faces'].shape == (3, 3)
    assert first['vertices'].shape == (9, 3)
    np.testing.assert_allclose(first['vertices'], second['vertices'])


def test_build_visual_scene_applies_joint_visual_origin_and_mesh_scale(tmp_path):
    package = tmp_path / 'fixture_description'
    mesh_dir = package / 'meshes'
    mesh_dir.mkdir(parents=True)
    write_stl(mesh_dir / 'tip.stl', [[[0, 0, 0], [1, 0, 0], [0, 1, 0]]])
    urdf = '''
<robot name="fixture">
  <link name="base"/>
  <link name="tip">
    <visual>
      <origin xyz="0 1 0" rpy="0 0 0"/>
      <geometry><mesh filename="package://fixture_description/meshes/tip.stl" scale="2 2 2"/></geometry>
    </visual>
  </link>
  <joint name="turn" type="revolute">
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <parent link="base"/><child link="tip"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
</robot>'''

    scene = build_visual_scene(
        urdf, {'fixture_description': package}, 'base',
        {'turn': np.pi / 2}, max_faces_per_mesh=10)

    assert len(scene['meshes']) == 1
    mesh = scene['meshes'][0]
    assert mesh['link'] == 'tip'
    np.testing.assert_allclose(mesh['vertices'][0], [0.0, 0.0, 0.0], atol=1e-7)
    np.testing.assert_allclose(mesh['vertices'][1], [0.0, 2.0, 0.0], atol=1e-7)
    np.testing.assert_allclose(scene['link_transforms']['tip'][:3, 3], [1, 0, 0])


def test_convex_outer_surface_is_finite_and_deterministic():
    rng = np.random.default_rng(5)
    points = rng.normal(size=(500, 3))
    points /= np.linalg.norm(points, axis=1)[:, None]
    points *= rng.uniform(0.6, 1.0, size=(500, 1))

    first = convex_outer_surface(points, max_points=200, seed=7)
    second = convex_outer_surface(points, max_points=200, seed=7)

    assert first['input_point_count'] == 500
    assert first['sampled_point_count'] == 200
    assert first['faces'].shape[1] == 3
    assert np.isfinite(first['vertices']).all()
    np.testing.assert_allclose(first['vertices'], second['vertices'])
    np.testing.assert_array_equal(first['faces'], second['faces'])


def test_simplify_mesh_reduces_faces_without_destroying_bounds():
    triangles = []
    for x in np.linspace(0, 1, 20):
        for y in np.linspace(0, 1, 20):
            triangles.extend([
                [[x, y, 0], [x + 0.04, y, 0], [x, y + 0.04, 0]],
                [[x + 0.04, y, 0], [x + 0.04, y + 0.04, 0], [x, y + 0.04, 0]],
            ])
    vertices = np.asarray(triangles, dtype=float).reshape(-1, 3)
    faces = np.arange(len(vertices)).reshape(-1, 3)

    simplified = simplify_mesh(vertices, faces, target_faces=180)

    assert len(simplified['faces']) <= 220
    assert len(simplified['faces']) < len(faces)
    np.testing.assert_allclose(
        simplified['vertices'].min(axis=0), vertices.min(axis=0), atol=0.04)
    np.testing.assert_allclose(
        simplified['vertices'].max(axis=0), vertices.max(axis=0), atol=0.04)
