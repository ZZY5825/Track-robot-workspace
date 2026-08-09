# Sensor Station Camera TF Design

## Goal

Publish a `camera_link` frame from the existing `sensor_station_link` without adding duplicate camera geometry.

## URDF structure

Add an empty `<link name="camera_link" />` and connect it with a fixed joint named `sensor_station_camera_joint`:

```text
sensor_station_link
  -> sensor_station_camera_joint (fixed)
    -> camera_link
```

The joint pose is expressed in the `sensor_station_link` coordinate frame. The supplied millimetre translation is converted to metres:

```text
xyz = 0.2212 0 0.318
rpy = 0 0 0
```

The camera inherits the sensor-station orientation. `camera_link` has no visual, collision, inertial, STL, or optical child frame.

## Verification

Add one contract test that checks the link, fixed joint, parent, child, translation, rotation, and absence of visual geometry. Then run the package tests, `check_urdf`, and the ROS 2 package build.
