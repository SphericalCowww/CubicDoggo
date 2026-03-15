#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/kali/Documents/CubicDoggo/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export QT_QPA_PLATFORM=xcb

ros2 launch my_robot_bringup cubic_doggo.with_lifecycle.launch.py &
LAUNCH_PID=$!
(
sleep 10
ros2 topic pub -1 /leg_set_named example_interfaces/msg/String "{data: 'stand'}"

sleep 2
ros2 topic pub -1 /leg_set_named example_interfaces/msg/String "{data: 'stand'}"

sleep 2
ros2 service call /leg_walk_toggle std_srvs/srv/SetBool "{data: true}"

sleep 20
ros2 service call /leg_walk_toggle std_srvs/srv/SetBool "{data: false}"

sleep 2
ros2 topic pub -1 /leg_set_named example_interfaces/msg/String "{data: 'stand'}"

sleep 2
ros2 topic pub -1 /leg_set_named example_interfaces/msg/String "{data: 'rest'}"

sleep 5
ros2 lifecycle set /cubic_doggo_lifecycle deactivate

sleep 2
ros2 lifecycle set /cubic_doggo_lifecycle shutdown

sleep 2
kill $LAUNCH_PID
) &
wait




