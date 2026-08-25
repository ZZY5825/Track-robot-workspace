#!/usr/bin/env python3
"""Dependency-light URDF/STL geometry helpers for publication rendering."""

import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull


def read_binary_stl(path, max_faces=None, seed=7):
    """Read a binary STL and optionally retain a deterministic face subset."""
    data = Path(path).read_bytes()
    if len(data) < 84:
        raise ValueError(f'invalid binary STL: {path}')
    original_count = struct.unpack('<I', data[80:84])[0]
    expected_size = 84 + original_count * 50
    if len(data) != expected_size:
        raise ValueError(f'unsupported or truncated binary STL: {path}')
    selected = np.arange(original_count)
    if max_faces is not None and original_count > int(max_faces):
        rng = np.random.default_rng(seed)
        selected = np.sort(rng.choice(original_count, int(max_faces), replace=False))
    vertices = np.empty((len(selected) * 3, 3), dtype=float)
    for output_index, face_index in enumerate(selected):
        offset = 84 + int(face_index) * 50
        values = struct.unpack('<12fH', data[offset:offset + 50])
        vertices[output_index * 3:(output_index + 1) * 3] = np.asarray(
            values[3:12], dtype=float).reshape(3, 3)
    faces = np.arange(len(vertices), dtype=int).reshape(-1, 3)
    return {
        'vertices': vertices,
        'faces': faces,
        'original_face_count': int(original_count),
        'rendered_face_count': int(len(faces)),
    }


def _numbers(element, attribute, default):
    if element is None or element.get(attribute) is None:
        return np.asarray(default, dtype=float)
    return np.asarray([float(value) for value in element.get(attribute).split()], dtype=float)


def _rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    matrix = np.eye(4)
    matrix[:3, :3] = _rpy_matrix(np.asarray(rpy, dtype=float))
    matrix[:3, 3] = np.asarray(xyz, dtype=float)
    return matrix


def _axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm == 0:
        return np.eye(4)
    x, y, z = axis / norm
    c, s, d = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    result = np.eye(4)
    result[:3, :3] = np.asarray([
        [c + x*x*d, x*y*d - z*s, x*z*d + y*s],
        [y*x*d + z*s, c + y*y*d, y*z*d - x*s],
        [z*x*d - y*s, z*y*d + x*s, c + z*z*d],
    ])
    return result


def _axis_translation(axis, distance):
    result = np.eye(4)
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm:
        result[:3, 3] = axis / norm * float(distance)
    return result


def _resolve_package_uri(uri, package_roots):
    prefix = 'package://'
    if not uri.startswith(prefix):
        return Path(uri)
    package_and_path = uri[len(prefix):].split('/', 1)
    if len(package_and_path) != 2 or package_and_path[0] not in package_roots:
        raise ValueError(f'unresolved mesh URI: {uri}')
    return Path(package_roots[package_and_path[0]]) / package_and_path[1]


def _material_rgba(visual):
    material = visual.find('material')
    color = material.find('color') if material is not None else None
    values = _numbers(color, 'rgba', (0.58, 0.61, 0.64, 1.0))
    return values.astype(float).tolist()


def _cluster_mesh_once(vertices, faces, grid_bins):
    minimum = vertices.min(axis=0)
    extent = vertices.max(axis=0) - minimum
    maximum_extent = float(np.max(extent))
    if maximum_extent <= 0:
        return {'vertices': vertices.copy(), 'faces': faces.copy()}
    cell_size = maximum_extent / max(2, int(grid_bins))
    keys = np.floor((vertices - minimum) / cell_size + 1e-10).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    clustered = np.zeros((int(inverse.max()) + 1, 3), dtype=float)
    counts = np.bincount(inverse)
    np.add.at(clustered, inverse, vertices)
    clustered /= counts[:, None]
    remapped = inverse[np.asarray(faces, dtype=int)]
    valid = ((remapped[:, 0] != remapped[:, 1]) &
             (remapped[:, 1] != remapped[:, 2]) &
             (remapped[:, 0] != remapped[:, 2]))
    remapped = remapped[valid]
    if not len(remapped):
        return {'vertices': vertices.copy(), 'faces': faces.copy()}
    canonical = np.sort(remapped, axis=1)
    _, unique_indices = np.unique(canonical, axis=0, return_index=True)
    return {'vertices': clustered, 'faces': remapped[np.sort(unique_indices)]}


def simplify_mesh(vertices, faces, target_faces=2000):
    """Simplify a triangle surface by deterministic vertex clustering."""
    vertices = np.asarray(vertices, dtype=float)
    faces = np.asarray(faces, dtype=int)
    target_faces = int(target_faces)
    if len(faces) <= target_faces:
        return {'vertices': vertices.copy(), 'faces': faces.copy()}
    grid_bins = max(4, int(round(math.sqrt(target_faces / 1.8))))
    best = None
    for _ in range(4):
        candidate = _cluster_mesh_once(vertices, faces, grid_bins)
        count = len(candidate['faces'])
        if best is None or abs(count - target_faces) < abs(len(best['faces']) - target_faces):
            best = candidate
        if target_faces * 0.72 <= count <= target_faces * 1.20:
            break
        if count < target_faces * 0.72:
            grid_bins = max(grid_bins + 1, int(round(grid_bins * 1.20)))
        else:
            grid_bins = max(4, int(round(grid_bins * 0.85)))
    return best


def build_visual_scene(urdf_xml, package_roots, root_link, joint_positions=None,
                       max_faces_per_mesh=1800, seed=7):
    """Resolve URDF visual meshes into world coordinates for one joint pose."""
    root = ET.fromstring(urdf_xml)
    joint_positions = dict(joint_positions or {})
    children = {}
    for joint in root.findall('joint'):
        origin = joint.find('origin')
        record = {
            'name': joint.get('name'),
            'type': joint.get('type', 'fixed'),
            'parent': joint.find('parent').get('link'),
            'child': joint.find('child').get('link'),
            'origin': _transform(
                _numbers(origin, 'xyz', (0, 0, 0)),
                _numbers(origin, 'rpy', (0, 0, 0))),
            'axis': _numbers(joint.find('axis'), 'xyz', (1, 0, 0)),
        }
        children.setdefault(record['parent'], []).append(record)

    transforms = {root_link: np.eye(4)}
    queue = [root_link]
    while queue:
        parent = queue.pop(0)
        for joint in children.get(parent, []):
            motion = np.eye(4)
            value = float(joint_positions.get(joint['name'], 0.0))
            if joint['type'] in ('revolute', 'continuous'):
                motion = _axis_rotation(joint['axis'], value)
            elif joint['type'] == 'prismatic':
                motion = _axis_translation(joint['axis'], value)
            transforms[joint['child']] = transforms[parent] @ joint['origin'] @ motion
            queue.append(joint['child'])

    meshes = []
    mesh_index = 0
    for link in root.findall('link'):
        link_name = link.get('name')
        if link_name not in transforms:
            continue
        for visual in link.findall('visual'):
            mesh_element = visual.find('./geometry/mesh')
            if mesh_element is None or not mesh_element.get('filename'):
                continue
            mesh_path = _resolve_package_uri(mesh_element.get('filename'), package_roots)
            geometry = read_binary_stl(mesh_path)
            scale = _numbers(mesh_element, 'scale', (1, 1, 1))
            visual_origin = visual.find('origin')
            visual_transform = _transform(
                _numbers(visual_origin, 'xyz', (0, 0, 0)),
                _numbers(visual_origin, 'rpy', (0, 0, 0)))
            vertices = geometry['vertices'] * scale[None, :]
            homogeneous = np.column_stack([vertices, np.ones(len(vertices))])
            world = (transforms[link_name] @ visual_transform @ homogeneous.T).T[:, :3]
            render_geometry = {'vertices': world, 'faces': geometry['faces']}
            if max_faces_per_mesh is not None and len(geometry['faces']) > int(max_faces_per_mesh):
                render_geometry = simplify_mesh(
                    world, geometry['faces'], target_faces=max_faces_per_mesh)
            meshes.append({
                'link': link_name,
                'path': str(mesh_path),
                'vertices': render_geometry['vertices'],
                'faces': render_geometry['faces'],
                'rgba': _material_rgba(visual),
                'original_face_count': geometry['original_face_count'],
                'rendered_face_count': int(len(render_geometry['faces'])),
            })
            mesh_index += 1
    return {
        'meshes': meshes,
        'link_transforms': transforms,
        'joint_positions': joint_positions,
    }


def convex_outer_surface(points, max_points=5000, seed=7):
    """Build a deterministic convex outer surface over finite xyz samples."""
    points = np.asarray(points, dtype=float)
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) < 4:
        raise ValueError('at least four finite 3D points are required')
    selected = np.arange(len(points))
    if len(points) > int(max_points):
        rng = np.random.default_rng(seed)
        selected = np.sort(rng.choice(len(points), int(max_points), replace=False))
    sampled = points[selected]
    hull = ConvexHull(sampled)
    return {
        'vertices': sampled,
        'faces': hull.simplices.astype(int),
        'input_point_count': int(len(points)),
        'sampled_point_count': int(len(sampled)),
        'surface_face_count': int(len(hull.simplices)),
        'volume_m3': float(hull.volume),
        'area_m2': float(hull.area),
        'seed': int(seed),
    }
