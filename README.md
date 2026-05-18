# Cubic Doggo: Homemade 12-DOF 4-Legged Robot Recipe

<img src="https://github.com/SphericalCowww/ROS_Cubic_Doggo/blob/main/CubicDoggo1.png" height="300"> <img src="https://github.com/SphericalCowww/ROS_Cubic_Doggo/blob/main/CubicDoggo2.png" height="300"> 

**Building from scratch a walking robot dog with all commercial/3D-printed parts and ROS2 Jazzy!**

Demos: derived from (<a href="https://github.com/SphericalCowww/ROS_leggedRobot_testBed">GitHub</a>/<a href="https://www.reddit.com/r/robotics/comments/1rouerc/first_time_building_a_hobbyist_robot_from_scratch/">Reddit</a>), upgrade in center of mass (<a href="https://www.reddit.com/r/ROS/comments/1roiee3/i_built_a_4legged_12dof_robot_dog_using_ros_2_i/">Reddit</a>), upgrade in walk gait (<a href="https://www.reddit.com/r/ROS/comments/1t8g1my/cubic_doggo_update_phew_it_finally_walks_with/">Reddit</a>)

## Ingredients

### 3D printing

The 3D printer model VOXELAB Aquila X2 is used, but as long as it prints PLA, it's good. All the FreeCAD files can be found here: <a href="https://github.com/SphericalCowww/CubicDoggo/tree/main/src/my_robot_description/mesh/CADv1">link</a>.

### Hardware requirements

| device | models | count | specification |
| - | - | - | - |
| servo motor | DYNAMIXEL <a href="https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/">XL430-W250-T</a> | 12 | Max stall torque: 1.5 N*m (at 12.0V, 1.4A, 1.071 Nm/A). Need also corresponding signal wires of various lengths |
| USB communication interface | DYNAMIXEL <a href="https://emanual.robotis.com/docs/en/parts/interface/u2d2/">U2D2</a> | 1 | Can control 12 servo in daisy chain if properly powered |
| communication/power hub | DYNAMIXEL <a href="https://emanual.robotis.com/docs/en/parts/interface/u2d2_power_hub/">U2D2 power hub board</a> | 2 | Operating voltage	3.5-24V withg a maximum current	of 10A |
| onboard computer | Raspberry Pi 5 | 1 | For running just IK, a small RAM is sufficient; Pi 4 could be good enough as long as ROS2 Jazzy can be installed | 
| DC-DC step-down convertor | Hailege <a href="https://www.amazon.de/Hailege-Module-Step-Down-Supply-Converter/dp/B07XFMMY1F">24V/12V to 5V/5A</a> | 1 | USB Port port to RaspPi,  DC 5.5mm x 2.5mm Male to battery | 
| battery | ZYGY <a href="https://www.amazon.de/dp/B0BB6RMM5Q">11.1V 2000mAh</a> | 2 | They already include protection. Need Charger. Need adapters for: T-plug => XT60 Male => DC 5.5mm x 2.5mm Male | 
| bearings | M3 bearing+<a href="https://www.amazon.de/dp/B01M2ZCLKX">spacer</a>, threaded rod, rod-end bearing | 8, 4, 4, 4 | rod length of 60mm to match the leg length; other dimensions can be accomodated by modifying the CAD |
| bolts and nuts | | | M3 screws are used throughout, except where required to accommodate the servos and electronic boards; use locknuts |

Other than the M3 screw, one specific requirement is the 1.2cm M2 screw (bag on the left in the photo) for the servo, which is not provided by the DYNAMIXEL package (bag on the right). Any shorter, the connection is too fragile, and any longer, the servo will be stuck from the inside. The M2.5 screws are generally provided by the DYNAMIXEL package, but other electrical boards may need them too, such as the RaspPi.

<img src="https://github.com/SphericalCowww/CubicDoggo/blob/main/fig_servo.png" width="400">

Optional:

| device | models | count | specification |
| - | - | - | - |
| power supply | SMPS | 1 | 12V/5A for testing without batteries | 
| capacitor | 1000uF | 2 | rating 25V or higher | 

### Special soldering requirements

  * Solder two <a href="https://emanual.robotis.com/docs/en/dxl/x/xl320/#connector-information">MOLEX 51065-0300</a> cables together to form a Y-wire, otherwise U2D2 doesn't have enough connectors
  * Optional: connect a wire between the 2 U2D2 power hubs such that they have shared ground. Then solder a capacitor at each end to the power rail. Careful about all the connection directions; otherwise, the whole power system can be severely damaged

<img src="https://github.com/SphericalCowww/CubicDoggo/blob/main/fig_Ywire.png" height="200"> <img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/powerSystemSharingGround.png" height="200"> 

### Power system

  * Daisy chain no more than 3 servos to avoid delay
  * Powering the hubs using their SMPS DC jacks. Then use the molex/screw terminal as an output to power the RaspPi via a ~12V-to-5V DC-DC converter.

<img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/powerSystem.png" width="500"> 

### Tools used

There are quite a few screws in difficult locations with the current design, which may require several tools for full assembly and repair in timely manner. The photo shows the tool used for this project.

<img src="https://github.com/SphericalCowww/CubicDoggo/blob/main/fig_tools.png" width="400">

## Running a Single Servo on ROS2

### Setting the servo IDs

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

For the 12 servos, the IDs are set as 11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, and 43.

### Installing the <a href="https://github.com/ROBOTIS-GIT/DynamixelSDK">dynamixel-sdk</a> and <a href="https://github.com/ROBOTIS-GIT/dynamixel-workbench">dynamixel-workbench</a>

Following <a href="https://github.com/SphericalCowww/ROS_init_practice">github</a> to install ROS. To install drivers for Dynamixel, 

    sudo apt install ros-jazzy-dynamixel-sdk* ros-jazzy-dynamixel-hardware* ros-jazzy-dynamixel-workbench*
    dpkg -l | grep dynamixel
    ros2 pkg list | grep dynamixel

Clone this repository,

    git clone https://github.com/SphericalCowww/CubicDoggo.git
    cd CubicDoggo

Then copy <a href="https://github.com/ROBOTIS-GIT/dynamixel-workbench/tree/main/dynamixel_workbench_toolbox/examples/src">src</a> directly under ``CubicDoggo/src/my_toolbox_dynamixel_workbench``. And for every ``.cpp`` file, change the following line:

    #include <DynamixelWorkbench.h>

To, 

    #include <cstdlib>
    #include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

Then connect U2D2 to the RaspPi USB port. To check if ROS2 sees the servos: 

    cd CubicDoggo
    colcon build
    source install/setup.bash
    sudo dmesg | tail -n 20
    # look for:
    ## usb 2-2: Detected FT232H
    ## usb 2-2: FTDI USB Serial Device converter now attached to ttyUSB0
    ls -l /dev/ttyUSB*
    sudo chmod a+rw /dev/ttyUSB0                   # required everytime after reconnection
    ros2 run my_toolbox_dynamixel_workbench model_scan /dev/ttyUSB0 2000000
    ros2 run my_toolbox_dynamixel_workbench position /dev/ttyUSB0 2000000 11 0.5

Check <a href="https://github.com/SphericalCowww/ROS_leggedRobot_testBed">ROS_leggedRobot_testBed GitHub</a>, specifically <a href="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/src/my_robot_firmware/src/testRaspPi5_dynamixel_u2d2_channel0_xl430.cpp">firmware code</a> for more single servo application. However, do NOT run it on a connected robot. 

### Setting the latency

Update USB port latency to 1 ms. Note that this USB signal is delicate; use the shortest, highest-quality USB cable possible.: 

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
    ## [ros2_control_node-2] [WARN] [1766867979.090889912] [controller_manager]: Could not enable FIFO RT scheduling policy: with error number <1> (Operation not permitted). See [https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] for details on enabling real-time scheduling.
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

## Running a single leg on ROS2

### Assembly and Launching URDF

Remember to twist the servo cables to reduce the signal interference from the power lines.

<img src="https://github.com/SphericalCowww/CubicDoggo/blob/main/fig_leg0.png" height="200"> <img src="https://github.com/SphericalCowww/CubicDoggo/blob/main/fig_leg1.png" height="200"> <img src="https://github.com/SphericalCowww/CubicDoggo/blob/main/fig_leg2.png" height="200"> <img src="https://github.com/SphericalCowww/CubicDoggo/blob/main/fig_leg3.png" height="200">

Run the following to view the moving parts:

    cd CubicDoggo
    colcon build
    source install/setup.bash
    ros2 launch my_robot_description my_robot.rviz.launch.xacro.py
    # if no config loaded
    ## Fixed Frame: base_link
    ## Add: RobotModel
    ## RobotModel: Description Topic: /robot_description

[Video demo](https://raw.githubusercontent.com/SphericalCowww/ROS_leggedRobot_testBed/main/rViz1Leg.mp4)

### Launching MoveIt2 demo:

    cd CubicDoggo
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

    # toggle: Approx IK Solutions
    # toggle if needed: MotionPlanning => Planned Path => Loop Animation
    # toggle if needed: Use Cartesian Path 

### Testing the ros2_control and MoveIt

Under ``CubicDoggo/src/my_robot_description/urdf/my_robot.ros2_control.xacro``, 
   
   * use ``<plugin>mock_components/GenericSystem</plugin>`` if just want rViz
   * use ``<plugin>ma_robot_namespace::HardwareInterfaceU2D2_my_robot</plugin>`` if want to control hardware

Then run:
    
    cd CubicDoggo
    colcon build
    source install/setup.bash
    ros2 launch my_robot_bringup my_robot.with_lifecycle.launch.py
    # on another terminal
    ros2 topic pub -1 /leg1_set_named example_interfaces/msg/String "{data: "pose1"}"
    ros2 topic pub -1 /leg1_set_joint example_interfaces/msg/Float64MultiArray "{data: [3.14, 3.14, 3.14]}"
    ros2 topic pub -1 /leg1_set_pose my_robot_interface/msg/MyRobotLeg1PoseTarget "{x: -0.092, y: 0.053, z: 0.135, use_cartesian_path: false}" 
    ros2 service call /leg1_walk_toggle std_srvs/srv/SetBool "{data: true}"     # IK very easy to fail
    ros2 service call /leg1_walk_toggle std_srvs/srv/SetBool "{data: false}"
    ros2 lifecycle set /my_robot_lifecycle deactivate
    ros2 lifecycle set /my_robot_lifecycle shutdown

### Launching with launch file:

    cd CubicDoggo
    colcon build
    source install/setup.bash
    ros2 launch my_robot_bringup my_robot.launch.py
    # NOTE: Sometimes it takes a second try to have everything registered
    # Add => MotionPlanning
    ## Context => Planning Library => ompl
    ## Planning => Goal State: pose1 => Plan => Execute

## Running full robot

### Assembly and Launching URDF

<img src="https://github.com/SphericalCowww/ROS_Cubic_Doggo/blob/main/CubicDoggo3.png" height="200"> <img src="https://github.com/SphericalCowww/ROS_Cubic_Doggo/blob/main/CubicDoggo4.png" height="200"> <img src="https://github.com/SphericalCowww/ROS_Cubic_Doggo/blob/main/CubicDoggo5.png" height="200"> <img 
src="https://github.com/SphericalCowww/ROS_Cubic_Doggo/blob/main/CubicDoggo6.png" height="200"> <img 
src="https://github.com/SphericalCowww/ROS_Cubic_Doggo/blob/main/CubicDoggo7.png" height="200"> 

Run the following to view the moving parts:

    cd CubicDoggo
    colcon build
    source install/setup.bash
    ros2 launch my_robot_description cubic_doggo.rviz.launch.xacro.py

Notice that the placement of the first servo is different between the front and back legs.

### Launching with commands

Under: ``CubicDoggo/src/my_robot_description/urdf/cubic_doggo.ros2_control.xacro``

   * use ``<plugin>mock_components/GenericSystem</plugin>`` if just want rViz
   * use ``<<plugin>cubic_doggo_namespace/HardwareInterfaceU2D2_cubic_doggo</plugin>`` if want to control hardware

Under: ``CubicDoggo/src/my_robot_bringup/launch/cubic_doggo.with_lifecycle.launch.py``

   * comment out ``rviz_node`` if don't want rViz to show
   * comment out ``joy_driver_node`` and ``joy_controller_node`` if don't want the joystick controller
   * comment out ``peripheral_node`` if don't want warning from low power

Start the robot (skip launching rviz_node, joy_driver_node, or joy_controller_node if needed):

    cd CubicDoggo
    colcon build
    source install/setup.bash
    # comment in rvis node in CubicDoggo/src/my_robot_bringup/launch/cubic_doggo.with_lifecycle.launch.py
    ros2 launch my_robot_bringup cubic_doggo.with_lifecycle.launch.py
    # on another terminal
    ros2 topic pub -1 /leg_set_named example_interfaces/msg/String "{data: "rest"}"
    ros2 topic pub -1 /leg_set_named example_interfaces/msg/String "{data: "stand"}"
    ros2 topic pub -1 /leg_set_named example_interfaces/msg/String "{data: "sit"}"
    ros2 topic pub -1 /leg_set_named example_interfaces/msg/String "{data: "bow"}"
    ros2 topic pub -1 /leg_set_joint example_interfaces/msg/Float64MultiArray "{data: [0, 3.14, 3.14, 3.54]}"
    ros2 topic pub -1 /leg_set_pose my_robot_interface/msg/CubicDoggoLegPoseTarget "{leg_index: 0, x: -0.092, y: 0.053, z: 0.135}" 
    ros2 service call /leg_walk_toggle std_srvs/srv/SetBool "{data: true}"
    ros2 service call /leg_walk_toggle std_srvs/srv/SetBool "{data: false}"

Check out all the gait options under ``CubicDoggo/src/my_robot_commander/src/cubic_doggo_lifecycle.cpp``:

    # double maxVelScale = 1.0, maxAccScale = 1.0;
    # int    waypoint_N     = 100;       // number of waypoints for each cycle,      default 100
    # double waypoint_dt    = 0.01;      // second for each waypoint,                default 0.01
    # double IK_bufferTime  = 0.10;      // time at end of cycle buffer for IK calc, default 0.10
    # double swing_fraction = 0.50;      // creep < 0.25 < stable trot < 0.5 < trot
    # double lift = 0.02, x_stride_max = 0.02, y_stride_max = 0.025, x_shift = 0.0, y_shift = 0.0;
    # double x_stride = 0.0, y_stride = 0.0;

### Testing the joystick controller:

    ls /dev/input/js*
    # output: /dev/input/js0
    # otherwise do: sudo jstest /dev/input/js0
    ros2 run joy joy_enumerate_devices
    # if no device found
    sudo usermod -aG input $USER
    sudo reboot

    cd CubicDoggo
    colcon build
    source install/setup.bash
    ros2 run joy joy_node
    # on another terminal
    ros2 run my_robot_controller cubic_doggo_joy_control
    # on yet another terminal
    ros2 node info /cubic_doggo_joy_control    
    ros2 topic info /joy --verbose
    ros2 topic echo /joy

### Testing low power alarm

For the low RaspPi power alarm, it's set by the rasp_pi_peripheral_node:

    for d in /sys/class/hwmon/hwmon*; do echo -n "$d: "; cat "$d/name"; done   # find the correct path for power alarm
    # update /home/cubicdoggo/Documents/CubicDoggo/src/my_robot_bringup/launch/cubic_doggo.with_lifecycle.launch.py
    echo timer | sudo tee /sys/class/leds/ACT/trigger                          # trigger LED first
    sudo chmod -R 777 /sys/class/leds/ACT/                                     # enable LED to indicate battery condition
    # test the node if needed
    ros2 run my_robot_commander rasp_pi_peripheral_node --ros-args -p power_path:=$(which ever power alarm path)
    # create a fake alarm by
    echo 1 > /tmp/fake_alarm
    
### Launching at the start of RaspPi

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

## Next directions

- Cubic Doggo MKII: strengthen the 2nd servo holding design (and perhaps to make the overall design easier to assemble), test with PLA+ (less likely to crack), test with STS-3215 (high torque, cheaper, but harder to develop), and use a Bluetooth controller
- Cubic Doggo <a href="https://github.com/chvmp/champ">CHAMP</a>: adapting IMU, and checking the compatibility with the CHAMP framework
- Cubic Doggo RL: walk gait trained by the RL using whatever open software available

## Acknowledgement

- Thank my second brother, who is a mechanical engineer, for advising me to use the locknuts, all-directional bearings, and battery for RC boats, as well as buying me a bunch of screws for cheap. 
