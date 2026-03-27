#!/bin/bash
set -e

WS=~/track_robot_ws
BRINGUP_SCRIPTS=$WS/src/track_robot/track_robot_bringup/scripts

echo "[INFO] Sourcing ROS2 Foxy..."
source /opt/ros/foxy/setup.bash

echo "[INFO] Sourcing workspace..."
source $WS/install/setup.bash

echo "[INFO] Bringing up CAN..."
bash $BRINGUP_SCRIPTS/start_bunker_can.sh

echo "[INFO] Launching Jetson bringup..."
ros2 launch track_robot_bringup jetson_base.launch.py
