# Bunker Pro 2 Sensor Station Mount Design

## Goal

Mount `FullCase.STL` rigidly on the centre of the Bunker Pro 2 top rail and display the combined model correctly in RViz2.

## Measured geometry

- The robot mesh uses metres. Its highest rail surface is at approximately `z = 0.016 m`.
- Vertices on that surface span approximately `x = -0.4241 m` to `0.4090 m`, placing the rail midpoint at `x = -0.0075 m`, `y = 0 m`.
- `FullCase.STL` measures `537.5 x 447.55 x 467 mm` and has its mesh origin at the minimum corner `(0, 0, 0)`.
- The station's 537.5 mm dimension will run along the robot's front-to-back x-axis. Its 447.55 mm dimension will run across the y-axis, and its 467 mm dimension will point upward along z.

## URDF structure

Add a `sensor_station_link` beneath `base_link` through a `sensor_station_joint` of type `fixed`.

The joint frame will be placed at the top-rail midpoint:

```text
xyz = -0.0075 0 0.016
rpy = 0 0 0
```

The visual mesh will use:

```text
filename = package://bunker_pro2/meshes/FullCase.STL
scale = 0.001 0.001 0.001
origin xyz = -0.26875 -0.223775 0
origin rpy = 0 0 0
```

The visual origin subtracts the STL's x/y bounding-box midpoint, aligning the station's bottom-centre point with the joint and rail midpoint. The z offset remains zero so the station bottom rests on the rail surface.

## Collision and dynamics scope

This first version adds only the visual geometry. The 194,524-triangle station mesh will not be reused as a collision mesh because it would be unnecessarily expensive for simulation. Collision geometry and inertial properties require physical dimensions and mass information and are outside this RViz visualization change.

## Verification

Automated contract tests will require:

- `sensor_station_link` and its visual mesh;
- the millimetre-to-metre mesh scale;
- the centre-correcting visual origin;
- a fixed joint from `base_link` to `sensor_station_link` at the rail midpoint;
- both mesh assets to be installed by the ROS 2 package.

After the tests pass, the package will be rebuilt, checked with `check_urdf`, launched in RViz2, and inspected through a new screenshot to confirm that the station is visible, upright, centred, and resting on the rail.
