# Robot Sensing and Manipulation Envelope — 3D Redesign

## Objective

Replace the current top/side scatter presentation with a single publication-quality
3D isometric scene that makes the physical robot, manipulator reach, and sensing
coverage immediately legible.

## Visual composition

- Render the integrated Bunker, PiPER, gripper, camera holder, and L515 from the
  actual URDF-referenced STL meshes.
- Use one opaque presentation pose for the complete robot and three low-opacity
  PiPER poses to demonstrate how representative boundary regions are reached.
- Convert the existing deterministic 30,000-sample gripper dataset into a
  translucent 3D convex outer surface. Label it **kinematic outer envelope**;
  do not claim collision-free reachability or uniform occupancy.
- Draw the recorded ZED intrinsics as a truncated 3D camera frustum.
- Draw the LiDAR as a truncated 360-degree horizontal annulus at the modeled
  sensor height. Clip both sensing illustrations to 2.5 m so the robot remains
  visible, while annotating the configured human-tracking range as 0.5–10.0 m.
- Remove the rectangular robot proxy, top-view workspace scatter, side-view
  workspace scatter, and L515 pose cloud.

## Geometry and data flow

1. Parse the integrated URDF visual elements, origins, scales, link hierarchy,
   and joint axes.
2. Resolve `package://` mesh paths against the local source tree and read binary
   STL triangles without adding a new dependency.
3. Apply forward kinematics to each visual mesh for a defined presentation pose
   and three representative ghost poses.
4. Build a deterministic convex hull from the existing sampled gripper points
   with SciPy and render its boundary triangles as a translucent surface.
5. Anchor the sensing geometry at the URDF-derived ZED and LiDAR locations and
   compute ZED horizontal/vertical angles from recorded `CameraInfo`.

## Scientific boundaries

- The reach surface is an outer visualization bound over sampled joint-limit
  positions; it can include unsampled or collision-invalid interior locations.
- The ZED frustum direction is a robot-forward schematic because the integrated
  URDF exposes the camera root but not the complete optical-frame mesh chain.
- The LiDAR annulus depicts horizontal coverage only; vertical FoV is not shown.
- No external calibration, live sensor, physical motion, or Semantic Search data
  is used.

## Output and validation

- Replace only `robot-sensing-manipulation-envelope-v1.{png,pdf,json}` and its
  rendering implementation.
- Preserve PNG review output, vector PDF, and a JSON sidecar containing mesh,
  joint-pose, hull, frustum, clipping, and limitation provenance.
- Verify deterministic geometry, finite transforms, valid files, visual absence
  of clipping/overlap, and unchanged generation of the other four extended
  evidence figures.
