# Homemade 12-DOF 4-Legged Robot Recipe

<img src="https://github.com/SphericalCowww/ROS_Cubic_Doggo/blob/main/CubicDoggo.png" width="300">

Derived from: <a href="https://github.com/SphericalCowww/ROS_leggedRobot_testBed">GitHub</a>/<a href="https://www.reddit.com/r/robotics/comments/1rouerc/first_time_building_a_hobbyist_robot_from_scratch/">Reddit</a>

Upgraded the center of mass: <a href="https://www.reddit.com/r/ROS/comments/1roiee3/i_built_a_4legged_12dof_robot_dog_using_ros_2_i/">Reddit</a>

Upgraded the walk gait: <a href="https://www.reddit.com/r/ROS/comments/1t8g1my/cubic_doggo_update_phew_it_finally_walks_with/">Reddit</a>

All the CAD models can be found here: <a href="https://github.com/SphericalCowww/CubicDoggo/tree/main/src/my_robot_description/mesh/CADv1">link</a>

# General

This is a project that 

- full <a href="https://github.com/SphericalCowww/CubicDoggo/tree/main/src/my_robot_description/mesh/CADv1">3D-printed</a>/commertial parts, no machine shop required, no specialized 3D-printer, no custom PCBs, no special motors
- all open source software, ROS2/MoveIt2, and FreeCAD/Cura
- modulo for building from the ground up
- no gears and avoiding tiny parts unless required by the servos/electronics. All nuts aiming for 3M
- 2 locations require soldering, however
- "Cubic" for minimalistic 3D design
- all servos are the same for simplicity
- exposed electronics for incorporating any future add-ons

## Hardware Requirements

| device | DYNAMIXEL models | number | specification |
| - | - | - | - |
| controller | <a href="https://www.raspberrypi.com/products/raspberry-pi-5/">rasp pi 5</a> | 1 | 16Gb RAM | 
| DC-DC step-down convertor | Hailege <a href="https://www.amazon.de/Hailege-Module-Step-Down-Supply-Converter/dp/B07XFMMY1F">24V/12V to 5V/5A</a> | 1 | USB Port port to rasp pi 5,  DC 5.5mm x 2.5mm Male to battery | 
| USB communication interface | DYNAMIXEL <a href="https://emanual.robotis.com/docs/en/parts/interface/u2d2/">U2D2</a> | 1 | Can control 12 servo in daisy chain if properly powered |
| communication/power hub | DYNAMIXEL <a href="https://emanual.robotis.com/docs/en/parts/interface/u2d2_power_hub/">U2D2 power hub board</a> | 2 | Operating voltage	3.5-24V withg a maximum current	of 10A |
| servo motor | DYNAMIXEL <a href="https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/">XL430-W250-T</a> | 12 | Max stall torque: 1.5 N*m (at 12.0V, 1.4A, 1.071 Nm/A) |
| battery | ZYGY <a href="https://www.amazon.de/dp/B0BB6RMM5Q">11.1V 2000mAh</a> | 2 | already includes protection. Need adapters for: T-plug => XT60 Male => DC 5.5mm x 2.5mm Male | 
| power supply | SMPS | - | 12V/5A for testing without batteries | 
| bearings | standard bearing, threaded rod, rod-end bearing, <a href="https://www.amazon.de/dp/B01M2ZCLKX">spacer</a> | 8, 4, 4, 4 | as long as the dimensions work out together, but the rod length so far is 60mm |
| bolts and nuts | | | DYNAMIXEL provides most < M3 bolts; use lock nuts; also best to have threaded inserts for 3D-printer materials, but designed choices can get around it |

#### 3D CAD dimensions

  * using 3M screws overall, as they are much more common. Piece thickness will be 4mm, except when a 3M screw is required on the thickness axis, then it's 6mm. For aperture dimension, 1.7mm radius for clearance, and 1.25mm for tapped without a heated insert.

### Power System

  * The daisy chain is recommended to chain 4 or fewer servos to avoid delay
  * Using a single U2D2 to connect to two U2D2 power hub boards, which requires a Y-cable from U2D2 to the 2 hubs: need to custom-connect two <a href="https://emanual.robotis.com/docs/en/dxl/x/xl320/#connector-information">MOLEX 51065-0300</a> cables to make a Y-cable.
  * Powering the hubs using their SMPS DC jacks. Then use the molex/screw terminal as an output to power the Raspberry Pi 5 via a 12V-to-5V DC-DC converter.

<img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/powerSystem.png" width="500"> <img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/powerSystemSharingGround.png" width="200"> 

# Running a single servo on ROS2

# Running a single leg on ROS2

# Running peripherals

# Running full robot

## Basic Commands

Start the robot (skip launching rviz_node, joy_driver_node, or joy_controller_node if needed):

    ros2 launch my_robot_bringup cubic_doggo.with_lifecycle.launch.py
    # on another terminal
    ros2 topic pub -1 /leg_set_named example_interfaces/msg/String "{data: "rest"}"
    ros2 topic pub -1 /leg_set_named example_interfaces/msg/String "{data: "stand"}"
    ros2 topic pub -1 /leg_set_named example_interfaces/msg/String "{data: "sit"}"
    ros2 topic pub -1 /leg_set_named example_interfaces/msg/String "{data: "bow"}"
    ros2 topic pub -1 /leg_set_joint example_interfaces/msg/Float64MultiArray "{data: [0, 3.14, 3.14, 3.54]}"
    ros2 topic pub -1 /leg_set_pose my_robot_interface/msg/CubicDoggoLegPoseTarget "{leg_index: 0, x: -0.092, y: 0.053, z: 0.135}" 
    ros2 service call /leg_walk_toggle std_srvs/srv/SetBool "{data: true}"

Debugging the ``joy_controller_node``:

    ls /dev/input/js*
    # output: /dev/input/js0
    # otherwise do: sudo jstest /dev/input/js0
    ros2 run joy joy_enumerate_devices
    # if no device found
    sudo usermod -aG input $USER
    sudo reboot
    
    ros2 run joy joy_node
    # on another terminal
    ros2 run my_robot_controller cubic_doggo_joy_control
    # on yet another terminal
    ros2 node info /cubic_doggo_joy_control    
    ros2 topic info /joy --verbose
    ros2 topic echo /joy
    
### check low power/battery alarm

For the low rasp pi power alarm, it's set by the rasp_pi_peripheral_node:

    for d in /sys/class/hwmon/hwmon*; do echo -n "$d: "; cat "$d/name"; done   # find the correct path for power alarm
    # update /home/cubicdoggo/Documents/CubicDoggo/src/my_robot_bringup/launch/cubic_doggo.with_lifecycle.launch.py
    echo timer | sudo tee /sys/class/leds/ACT/trigger                          # trigger LED first
    sudo chmod -R 777 /sys/class/leds/ACT/                                     # enable LED to indicate battery condition
    # test the node if needed
    ros2 run my_robot_commander rasp_pi_peripheral_node --ros-args -p power_path:=$(which ever power alarm path)
    # create a fake alarm by
    echo 1 > /tmp/fake_alarm
### launch at the start of turning on rasp pi

    chmod +x /home/cubicdoggo/Documents/CubicDoggo/start_robot.sh
    sudo cp /home/cubicdoggo/Documents/CubicDoggo/robot_startup.service /etc/systemd/system/robot_startup.service
    sudo chmod 644 /etc/systemd/system/robot_startup.service
    sudo systemctl enable robot_startup.service                                # now will start at reboot
    sudo systemctl daemon-reload                                               # reload whenever there is a change
    # sudo systemctl restart robot_startup.service                             # restart, even if is rrunning
    # sudo systemctl stop    robot_startup.service                             # stop right now, kill all relevant nodes
    # sudo systemctl disable robot_startup.service                             # disable at reboot
    # journalctl -u robot_startup.service -n 100 > start_robot_output.txt      # to check the output
    # journalctl -u robot_startup.service -f

# Running a single servo on ROS2

# Future development directions

- Cubic Doggo MKII: strengthen the 2nd servo holding design, use PLA+, use STS-3215 (high torque, cheaper, but harder to develop)
- Cubic Doggo <a href="https://github.com/chvmp/champ">CHAMP</a>: adapting IMU, and check the compatibility to the CHAMP framework
- Cubic Doggo RL: walk gait trained by the RL using whatever open software available

# Acknowledgement

- Thank my second brother, who is a mechanical engineer, for advising me to use the locknuts, all-directional bearings, and battery for RC boats, as well as buying me a bunch of screws for cheap. But he also suggested me to use 100% 3D-filling with cavities to cut weight instead, which I followed only to a degree. Anyways, he doesn't have a GitHub account, so probably will never find out.
