# Robot Envelope 3D Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the flat robot-envelope figure with an evidence-backed isometric 3D rendering of the actual URDF/STL robot, its kinematic outer reach surface, representative arm poses, and truncated sensing geometry.

**Architecture:** A focused pure-Python geometry module parses binary STL and URDF visuals, performs joint-aware forward kinematics, and creates deterministic render meshes. The existing extended-figure renderer consumes those meshes and the recorded replay metadata to replace only the robot-envelope PNG/PDF/JSON triplet.

**Tech Stack:** Python 3, NumPy, SciPy `ConvexHull`, Matplotlib `mplot3d`, XML/URDF, binary STL, pytest.

## Global Constraints

- Do not add a new runtime dependency.
- Do not start sensors, ROS nodes, or robot motion.
- Label the reach surface as a convex kinematic outer envelope, not collision-free reachability.
- Clip visualized ZED/LiDAR coverage to 2.5 m and disclose the configured 0.5–10.0 m range.
- Preserve PNG, vector PDF, and JSON provenance.
- Do not commit or push; leave outputs for review.

---

### Task 1: URDF/STL 3D geometry helpers

**Files:**
- Create: `tools/visualization/human_following_3d_geometry.py`
- Create: `tools/visualization/tests/test_human_following_3d_geometry.py`

**Interfaces:**
- Consumes: binary STL paths, integrated URDF XML, `package://` roots, joint-position dictionaries, sampled xyz points.
- Produces: `read_binary_stl(path, max_faces, seed)`, `build_visual_scene(urdf_xml, package_roots, root_link, joint_positions, max_faces_per_mesh)`, and `convex_outer_surface(points, max_points, seed)`.

- [ ] Write tests using a one-triangle binary STL and a two-link URDF fixture. Assert decoded vertices/faces, mesh scale/origin/joint transforms, deterministic face sampling, and finite outward hull triangles.
- [ ] Run `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tools/visualization/tests/test_human_following_3d_geometry.py` and confirm import failure before implementation.
- [ ] Implement binary STL decoding with `struct`, URDF visual resolution with `xml.etree.ElementTree`, the same RPY/axis transform convention as the existing workspace sampler, and seeded `scipy.spatial.ConvexHull` subsampling.
- [ ] Re-run the focused test and require all assertions to pass.

### Task 2: Replace the envelope renderer

**Files:**
- Modify: `tools/visualization/render_human_following_extended_figures.py`

**Interfaces:**
- Consumes: `build_visual_scene`, `convex_outer_surface`, existing 30,000 gripper samples, recorded `CameraInfo`, and URDF-derived sensor origins.
- Produces: one isometric 3D scene and expanded JSON provenance for `robot-sensing-manipulation-envelope-v1`.

- [ ] Add mesh rendering helpers using `Poly3DCollection`, link-class colors, depth-aware opacity, equal 3D aspect, and a subtle ground plane.
- [ ] Define one opaque presentation pose and three deterministic translucent ghost poses within PiPER joint limits; render the mobile base once and arm visuals for every pose.
- [ ] Render the gripper convex outer surface, a robot-forward ZED frustum using recorded horizontal/vertical FoV, and a 360-degree LiDAR annulus at the URDF sensor height, all clipped at 2.5 m.
- [ ] Delete the old rectangle proxy, top-view hexbin, side-view hexbin, and L515 scatter presentation from `render_envelope`.
- [ ] Store presentation poses, mesh counts, hull counts, clip range, actual configured range, and scientific limitations in the JSON sidecar.

### Task 3: Render and verify

**Files:**
- Replace: `docs/assets/paper/results/robot-sensing-manipulation-envelope-v1.png`
- Replace: `docs/assets/paper/results/robot-sensing-manipulation-envelope-v1.pdf`
- Replace: `docs/assets/paper/results/robot-sensing-manipulation-envelope-v1.json`

**Interfaces:**
- Consumes: the updated renderer and existing replay/URDF evidence.
- Produces: the reviewable figure triplet without changing the other four extended figures.

- [ ] Run the full renderer with the existing replay, extended evidence, safety evidence, URDF, base mesh, and output directory arguments.
- [ ] Inspect the new PNG for recognizable robot geometry, legible reach surface, clear sensor volumes, non-overlapping labels, and absence of old top/side scatter panels.
- [ ] Run focused geometry/analysis tests, `py_compile`, strict JSON assertions, `file` on PNG/PDF, hashes on the other four figure triplets before/after regeneration, and `git diff --check`.
