# Cubic Doggo: Homemade 12-DOF 4-Legged Robot Recipe

<img src="https://github.com/SphericalCowww/ROS_Cubic_Doggo/blob/main/CubicDoggo.png" width="300"> 

**Building from scratch a walking robot dog with all commercial/3D-printed parts and ROS2!**

Demos: derived from (<a href="https://github.com/SphericalCowww/ROS_leggedRobot_testBed">GitHub</a>/<a href="https://www.reddit.com/r/robotics/comments/1rouerc/first_time_building_a_hobbyist_robot_from_scratch/">Reddit</a>), upgrade in center of mass (<a href="https://www.reddit.com/r/ROS/comments/1roiee3/i_built_a_4legged_12dof_robot_dog_using_ros_2_i/">Reddit</a>), upgrade in walk gait (<a href="https://www.reddit.com/r/ROS/comments/1t8g1my/cubic_doggo_update_phew_it_finally_walks_with/">Reddit</a>)

All the FreeCAD files can be found here: <a href="https://github.com/SphericalCowww/CubicDoggo/tree/main/src/my_robot_description/mesh/CADv1">link</a>

# Ingredients

## Hardware Requirements

| device | DYNAMIXEL models | number | specification |
| - | - | - | - |
| servo motor | DYNAMIXEL <a href="https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/">XL430-W250-T</a> | 12 | Max stall torque: 1.5 N*m (at 12.0V, 1.4A, 1.071 Nm/A) |
| USB communication interface | DYNAMIXEL <a href="https://emanual.robotis.com/docs/en/parts/interface/u2d2/">U2D2</a> | 1 | Can control 12 servo in daisy chain if properly powered |
| communication/power hub | DYNAMIXEL <a href="https://emanual.robotis.com/docs/en/parts/interface/u2d2_power_hub/">U2D2 power hub board</a> | 2 | Operating voltage	3.5-24V withg a maximum current	of 10A |
| onboard computer | <a href="https://www.raspberrypi.com/products/raspberry-pi-5/">rasp pi 5</a> | 1 | | 
| DC-DC step-down convertor | Hailege <a href="https://www.amazon.de/Hailege-Module-Step-Down-Supply-Converter/dp/B07XFMMY1F">24V/12V to 5V/5A</a> | 1 | USB Port port to rasp pi 5,  DC 5.5mm x 2.5mm Male to battery | 
| battery | ZYGY <a href="https://www.amazon.de/dp/B0BB6RMM5Q">11.1V 2000mAh</a> | 2 | They already include protection. Need Charger. Need adapters for: T-plug => XT60 Male => DC 5.5mm x 2.5mm Male | 
| bearings | M3 bearing+<a href="https://www.amazon.de/dp/B01M2ZCLKX">spacer</a>, threaded rod, rod-end bearing | 8, 4, 4, 4 | rod length of 60mm to match the leg length; other dimensions can be accomodated by modifying the CAD |
| bolts and nuts | | | M3 hardware is used throughout, except where required to accommodate the servos and electronic boards; use locknuts |

Optional:

| device | DYNAMIXEL models | number | specification |
| - | - | - | - |
| power supply | SMPS | - | 12V/5A for testing without batteries | 

## Special soldering requirements

- modulo for building from the ground up
- no gears and avoiding tiny parts unless required by the servos/electronics. All nuts aiming for 3M
- exposed electronics for incorporating any future add-ons


#### 3D CAD dimensions

  * using 3M screws overall, as they are much more common. Piece thickness will be 4mm, except when a 3M screw is required on the thickness axis, then it's 6mm. For aperture dimension, 1.7mm radius for clearance, and 1.25mm for tapped without a heated insert.

### Power System

  * The daisy chain is recommended to chain 4 or fewer servos to avoid delay
  * Using a single U2D2 to connect to two U2D2 power hub boards, which requires a Y-cable from U2D2 to the 2 hubs: need to custom-connect two <a href="https://emanual.robotis.com/docs/en/dxl/x/xl320/#connector-information">MOLEX 51065-0300</a> cables to make a Y-cable.
  * Powering the hubs using their SMPS DC jacks. Then use the molex/screw terminal as an output to power the Raspberry Pi 5 via a 12V-to-5V DC-DC converter.

<img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/powerSystem.png" width="500"> <img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/powerSystemSharingGround.png" width="200"> 

# Running a single servo on ROS2

### setting the servo IDs

Connecting servo to U2D2 according to <a href="https://www.youtube.com/watch?v=FIj_NULYOKQ">YouTube</a>:

<img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/basicConnection_DYNAMIXEL.png" width="200">

Use the following App <a href="https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/">DYNAMIXEL Wizard 2.0</a> to change the servo ID (default 1) to, say, ID 10. Note that if any of the servos have the same ID, they will NOT show up in the scan. The servos also all initially come with an ID of 1, so they must be connected one by one to U2D2 to change their IDs accordingly.

    connect U2D2 to computer => open DYNAMIXEL Wizard 2.0
    # Options 
    ## Select protocol to scan => Protocol 2.0 (only)
    ## Select port to scan => which ever port is connected
    ## Select baudrate to scan => 57600 bps and 2000000bpt => OK
    ## Select ID range to scan => End => 50
    ## OK
    # Scan
    # Item 
    ## (Address 7) ID => ID 11 (on the right) => Save (may need to scroll down) 
    ## (Address 8) Baud Rate (Bus) => 2Mbps (on the right) => Save (may need to scroll down) 
    ## (Address 9) Return Delay Time => 0 (on the right) => Save (may need to scroll down) 
    ## (Address 68) Status Return Level => 2 (on the right) => Save (may need to scroll down) 

Can also test out the servo:

    # LED (top right toggle)
    # Torque (top right toggle) => set the value for motion (can select Velocity/Position mode)

## Installing the <a href="https://github.com/ROBOTIS-GIT/DynamixelSDK">dynamixel-sdk</a> and  <a href="https://github.com/ROBOTIS-GIT/dynamixel-workbench">dynamixel-workbench</a>

Following <a href="https://github.com/SphericalCowww/ROS_init_practice">github</a> to install ROS. To install drivers for Dynamixel, 

    sudo apt install ros-jazzy-dynamixel-sdk* ros-jazzy-dynamixel-hardware* ros-jazzy-dynamixel-workbench*
    dpkg -l | grep dynamixel
    ros2 pkg list | grep dynamixel

Then copy <a href="https://github.com/ROBOTIS-GIT/dynamixel-workbench/tree/main/dynamixel_workbench_toolbox/examples/src">src</a> directly under ``/src/my_toolbox_dynamixel_workbench``. And for every ``.cpp`` file, change the following line:

    #include <DynamixelWorkbench.h>

To, 

    #include <cstdlib>
    #include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

Then ``cd ROS_leggedRobot_testBed`` and build,

    colcon build
    source install/setup.bash

Connect U2D2 to Rasp Pi USB port. To check if ROS2 sees the servos: 

    sudo dmesg | tail -n 20
    # look for:
    ## usb 2-2: Detected FT232H
    ## usb 2-2: FTDI USB Serial Device converter now attached to ttyUSB0
    ls -l /dev/ttyUSB*
    sudo chmod a+rw /dev/ttyUSB0                   # required everytime after reconnection
    ros2 run my_toolbox_dynamixel_workbench model_scan /dev/ttyUSB0 2000000
    ros2 run my_toolbox_dynamixel_workbench position /dev/ttyUSB0 2000000 11 0.5

Update USB port latency to 1 ms. Note that this USB signal is delicate; use as short and high-quality a USB cable as possible.: 

    lsusb
    # look for: Bus 002 Device 003: ID 0403:6014 Future Technology Devices International, Ltd FT232H Single HS USB-UART/FIFO IC
    sudo vim /etc/udev/rules.d/99-dynamixel-latency.rules
    ---------- add: 
    SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTR{device/latency_timer}="1", SYMLINK+="ttyU2D2", MODE="0666", GROUP="dialout"
    # ATTR{device/latency_timer}="1": Sets the 1ms latency (The "Sync Read" fix).
    # SYMLINK+="ttyU2D2": (Optional but helpful) This creates a static name for your device. You can now use /dev/ttyU2D2 in your code instead of /dev/ttyUSB0, so it won't break if you plug in another USB device.
    # MODE="0666": Allows your ROS node to access the port without needing sudo.
    ---------- 
    sudo vim /etc/udev/rules.d/99-ftdi.rules
    ---------- add: 
    SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}==6014, MODE="0666", GROUP="dialout"
    SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE="0666", GROUP="dialout"
    # This gives the correct permission to the USB port in question, otherwise, whenever reconnected, needs to do: sudo chmod a+rw /dev/ttyUSB0
    ---------- 
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ls -l /dev/ttyU2D2
    # if doesn't exist, do: sudo apt remove brltty
    cat /sys/class/tty/ttyUSB0/device/latency_timer
    # which should show 1 for 1 ms

Update to run the firmware with proper permissions to avoid latency:

    # without sudo, we will see the following WARNING in ros2 launch:
    ## [ros2_control_node-2] [WARN] [1766867979.090889912] [controller_manager]: Could not enable FIFO RT scheduling policy: with error number <1> (Operation not permitted). See [https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] for details on how to enable realtime scheduling.
    # with sudo, we can do the following:
    ## sudo bash -c "source /opt/ros/jazzy/setup.bash; source install/setup.bash; ros2 launch my_robot_bringup my_robot.with_commander.launch.py"
    # however, eventually we want to run without sudo in case it messes up with other permissions:
    sudo addgroup realtime
    sudo usermod -a -G realtime $USER
    sudo vim /etc/security/limits.d/realtime.conf:
    ----------  add:
    @realtime soft rtprio 99
    @realtime soft priority 99
    @realtime soft memlock unlimited
    @realtime hard rtprio 99
    @realtime hard priority 99
    @realtime hard memlock unlimited
    ---------- 
    sudo reboot
    # 

    ### testing the driver in ROS2

    colcon build
    source install/setup.bash
    ros2 run my_robot_firmware testRaspPi5_dynamixel_u2d2_leg1swing_xl430
    ps -ef | grep testRaspPi5_dynamixel_u2d2_leg1swing_xl430                 # to kill it before it ends
    # only when dynamixels are not connected to into a leg: 
    ## ros2 run my_robot_firmware testRaspPi5_dynamixel_u2d2_channel0_xl430 
    ## ros2 run my_robot_firmware testRaspPi5_dynamixel_u2d2_leg1swipe_xl430

# Running a single leg on ROS2

### testing the driver in ROS2

    colcon build
    source install/setup.bash
    ros2 run my_robot_firmware testRaspPi5_dynamixel_u2d2_leg1swing_xl430
    ps -ef | grep testRaspPi5_dynamixel_u2d2_leg1swing_xl430                 # to kill it before it ends
    # only when dynamixels are not connected to into a leg: 
    ## ros2 run my_robot_firmware testRaspPi5_dynamixel_u2d2_channel0_xl430 
    ## ros2 run my_robot_firmware testRaspPi5_dynamixel_u2d2_leg1swipe_xl430

    ### testing the driver with ros2_control and MoveIt
Under ``ma_robot.ros2_control.xacro``, switch ``<plugin>mock_components/GenericSystem</plugin-->`` to ``<plugin>ma_robot_namespace::HardwareInterfaceU2D2_ma_robot</plugin>``. The latter plugin type can be found at the bottom of ``src/my_robot_firmware/hardware_interface_ma_robot_dynamixel_u2d2_xl430.xml``. Then run the following:

    colcon build
    source install/setup.bash
    ros2 launch my_robot_bringup ma_robot.with_commander.launch.py
    ros2 topic info /arm_set_name
    ros2 topic pub -1 /arm_set_named example_interfaces/msg/String "{data: "arm_pose1"}"
    ros2 topic info /arm_set_joint
    ros2 topic pub -1 /arm_set_joint example_interfaces/msg/Float64MultiArray "{data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]}"
    ros2 topic info /arm_set_pose
    ros2 topic pub -1 /arm_set_pose my_robot_interface/msg/MaRobotArmPoseTarget "{x: 0.7, y: 0.0, z: 0.4, roll: 3.14, pitch: 0.0, yaw: 0.0, use_cartesian_path: false}"
    ros2 topic info /gripper_set_open
    ros2 topic pub -1 /gripper_set_open example_interfaces/msg/Bool "{data: false}"

## Launch ROS2 interface with 1 leg

### launch urdf

Then run the following:

    colcon build
    source install/setup.bash
    ros2 launch my_robot_description my_robot.rviz.launch.xacro.py
    # if no config loaded
    ## Fixed Frame: base_link
    ## Add: RobotModel
    ## RobotModel: Description Topic: /robot_description

[Video demo](https://raw.githubusercontent.com/SphericalCowww/ROS_leggedRobot_testBed/main/rViz1Leg.mp4)

### moveit2 setup assistance with a leg

Launch the MoveIt assistance:

    sudo apt update
    sudo apt install
    colcon build
    source install/setup.bash
    ros2 launch moveit_setup_assistant setup_assistant.launch.py
    # Create New Moveit Configuration Package (or edit if the configuration files already exist)
    # Browse => src/my_robot_description/urdf/my_robot.urdf.xacro => Load Files
    # Start Screen: can toggle visual/collision
    # Self-Collisions => Generate Collision Matrix: removes all never-in-contact and adjacent collisions
    # Virtual Joints => Add Virtual Joint => Virtual Joint Name: virtual_joint => Parent Frame Name: world => Joint Type: fixed => Save
    ## no need if already defined in urdf. Can always comment out afterwards in /src/my_robot_moveit_config/config/my_robot.srdf 
    # Planning Groups => Add Group => Group Name: leg1 => Kinametic Solver: kdl_kinematics_plugin => Add Joints => 
    ## choose with right arrow "servo1_servo1_padding", "servo2_servo2_padding", "servo3_calfFeet", and "calfFeet_calfSphere" => Save
    # Robot Poses => Add Pose => all joints at 0 => Pose Name: home => Save: can add a few other ones for debugging
    # ros2_control URDF Model => position for Command Interfaces and State Interfaces => Add interfaces
    # ROS2 Controllers => Auto Add JointTrajectoryController
    # Moveit Controllers => Auto Add FollowJointsTrajectory
    # Author Information => add anything (e.g. "my_robot", "my_robot@gmail.com"), otherwise bugged
    # Configuration Files => Browse: src/my_robot_moveit_config/ => Generate Package: double check if files are generated => Exit Setup Assistant

Fix the following file:

    # src/my_robot_moveit_config/config/joint_limits.yaml => max_velocity: 20.0, has_acceleration_limits: true, max_acceleration: 10.0 (need to be float)
    # src/my_robot_moveit_config/config/moveit_controllers.yaml => add the following under leg1_controller: 
    ## action_ns: follow_joint_trajectory
    ## default: true
    # src/my_robot_moveit_config/config/initial_positions.yaml => servo1_servo1_padding: 3.14, servo2_servo2_padding: 3.14, servo3_calfJoint: 3.14
    # src/my_robot_moveit_config/config/my_robot.srdf => include only the following in <group name="leg1">:
    ## <group name="leg1">
    ##     <chain base_link="base_link" tip_link="calfSphere"/>
    ## </group>
    # src/my_robot_moveit_config/config/kinematics.yaml => replace with the following:
    ##leg1:
    ##  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
    ##  kinematics_solver_search_resolution: 0.005
    ##  kinematics_solver_timeout: 0.05
    ##  kinematics_solver_attempts: 3
    ##  position_only_ik: True        # this one is important because the leg does NOT care about the orientation of the end effector

### launch the demo:

    colcon build
    source install/setup.bash
    ros2 launch my_robot_moveit_config demo.launch.py
    # ignore: [move_group-3] [ERROR] [1758361830.007872451] [move_group.moveit.moveit.ros.occupancy_map_monitor]: No 3D sensor plugin(s) defined for octomap updates
    # ignore: [rviz2-4] [ERROR] [1758361834.128908606] [moveit_143394722.moveit.ros.motion_planning_frame]: Action server: /recognize_objects not available
    # MotionPlanning:
    ## Planning Group: leg1
    ## Goal State: pose1
    ## Plan
    ## Execute

Note that to move the motion wheel in rViz:

    # toggle: Approx IK Soluations
    # toggle if needed: MotionPlanning => Planned Path => Loop Animation
    # toggle if needed: Use Cartesian Path 

### launch with a proper launch file:

    mv src/my_robot_moveit_config/config/ros2_controllers.yaml src/my_robot_bringup/config/my_robot_controllers.yaml
    # change the following line if needed in my_robot_controllers.yaml
    ## update_rate: 100 # Hz
    mv src/my_robot_moveit_config/config/my_robot.ros2_control.xacro src/my_robot_description/urdf/
    rm src/my_robot_moveit_config/config/my_robot.urdf.xacro
    # modify the following line in my_robot.ros2_control.xacro:
    ## remove: <xacro:property name="initial_positions" value="${xacro.load_yaml(initial_positions_file)['initial_positions']}"/>
    ## for all servos, update to: <param name="initial_value">3.14</param> 
    # adding the following line in my_robot.urdf.xacro:
    ## <xacro:include filename="my_robot.ros2_control.xacro" />
    colcon build
    source install/setup.bash
    ros2 launch my_robot_bringup my_robot.launch.py
    # NOTE: Sometimes it takes a second try to have everything registered
    # Add => MotionPlanning
    ## Context => Planning Library => ompl
    ## Planning => Goal State: pose1 => Plan => Execute

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

# Future development directions

- Cubic Doggo MKII: strengthen the 2nd servo holding design, test with PLA+ (less likely to crack), test with STS-3215 (high torque, cheaper, but harder to develop), and use a Bluetooth controller
- Cubic Doggo <a href="https://github.com/chvmp/champ">CHAMP</a>: adapting IMU, and checking the compatibility with the CHAMP framework
- Cubic Doggo RL: walk gait trained by the RL using whatever open software available

# Acknowledgement

- Thank my second brother, who is a mechanical engineer, for advising me to use the locknuts, all-directional bearings, and battery for RC boats, as well as buying me a bunch of screws for cheap. 
