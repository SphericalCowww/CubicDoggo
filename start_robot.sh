#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/kali/Documents/CubicDoggo/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export QT_QPA_PLATFORM=xcb

ros2 launch my_robot_bringup cubic_doggo.with_lifecycle.launch.py




