# Bunker Pro 2 Sensor Station Mount Design

## Goal

Mount `FullCase.STL` rigidly on the centre of the Bunker Pro 2 top rail and display the combined model correctly in RViz2.

## Measured geometry

- The robot mesh uses metres. Its highest rail surface is at approximately `z = 0.016 m`.
- Vertices on that surface span approximately `x = -0.4241 m` to `0.4090 m`, placing the rail midpoint at `x = -0.0075 m`, `y = 0 m`.
- `FullCase.STL` measures `537.5 x 447.55 x 467 mm` and has its mesh origin at the minimum corner `(0, 0, 0)`.
- After the requested `roll = +90 degrees` and `yaw = 180 degrees`, the station's 537.5 mm dimension runs along the robot's front-to-back x-axis, its 467 mm dimension runs across y, and its 447.55 mm dimension points upward along z.

## URDF structure

Add a `sensor_station_link` beneath `base_link` through a `sensor_station_joint` of type `fixed`.

The joint frame will be placed at the top-rail midpoint:

```text
xyz = -0.0075 0 0.016
rpy = 1.57079632679 0 3.14159265359
```

The visual mesh will use:

```text
filename = package://bunker_pro2/meshes/FullCase.STL
scale = 0.001 0.001 0.001
origin xyz = -0.26875 0 -0.2335
origin rpy = 0 0 0
```

The visual origin subtracts the STL's x/z bounding-box midpoint. After the fixed-joint rotation, the original `y = 0` face becomes the bottom mounting face, its centre aligns with the joint and rail midpoint, and that face rests on the rail surface.

## Collision and dynamics scope

This first version adds only the visual geometry. The 194,524-triangle station mesh will not be reused as a collision mesh because it would be unnecessarily expensive for simulation. Collision geometry and inertial properties require physical dimensions and mass information and are outside this RViz visualization change.

## Verification

Automated contract tests will require:

- `sensor_station_link` and its visual mesh;
- the millimetre-to-metre mesh scale;
- the centre-correcting visual origin;
- a fixed joint from `base_link` to `sensor_station_link` at the rail midpoint;
- both mesh assets to be installed by the ROS 2 package.

After the tests pass, the package will be rebuilt, checked with `check_urdf`, launched in RViz2, and inspected through a new screenshot to confirm that the station has the requested roll/yaw, is centred, and rests on the rail.
