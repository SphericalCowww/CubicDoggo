# Development for a 12-DOF 4-Legged Robot

Derivation of the <a href="https://github.com/SphericalCowww/ROS_leggedRobot_testBed">GitHub Repository</a>:

<img src="https://github.com/SphericalCowww/ROS_Cubic_Doggo/blob/main/CubicDoggo.png" width="300">

Goal 1: Resolving the walking gait. The current walking gait (<a href="https://www.reddit.com/r/robotics/comments/1rouerc/first_time_building_a_hobbyist_robot_from_scratch/">link</a>) is already showing the problem of not being able to lift the feet in action. Perhaps this can be resolved by gait optimization, but it can also be limited intrinsic motor speed, maximum current from the batteries, the feet being too heavy, or just not having enough friction on the feet.

## Training with Isaac Sim

### Hardware

Planned hardware spec (<a href="https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html">requirement reference</a>):

  * AMD Ryzen 7 7800X3D - 8 x 4.2 GHz (TurboBoost bis 5.00 GHz, 8 Kerne / 16 Threads, 96MB Cache)
  * NVIDIA GeForce RTX 5080 (16GB GDDR7)
  * 64 GB DDR5 RAM
  * 2000 GB M.2 SSD
  * Gigabit Ethernet LAN, Wi-Fi 6
  * 6x USB 3.2, 4x USB 2.0, 2x HDMI 2.1a, 3x DisplayPort 1.4a, 1x RJ-45, 1x Mikrophon, 1x Kopfhörer, Line-In/ Line-Out/ Mikrofon
  * Xilence Performance A+ M705D => updated to: be quiet! Pure Loop 2 FX 240mm
  * 850 Watt
  * 210 x 430 x 444 mm

### Installation

To install Isaac Sim, follow <a href="https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/install_workstation.html">installation guide</a> and get zip file from <a href="https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/download.html">download links</a>:

    sudo apt update
    sudo apt install software-properties-common -y
    sudo add-apt-repository ppa:deadsnakes/ppa -y
    sudo apt update
    sudo apt install python3.11 python3.11-venv python3.11-dev

    python3.11 -m venv isaacsim-env
    source isaacsim-env/bin/activate
    pip install --upgrade pip
    pip install isaacsim[compatibility-check]   # do not install any other packages yet
    isaacsim isaacsim.exp.compatibility_check   # if see orange: Settings => Power => Power Mode => Performance
    deactivate

    sudo snap install code --classic   # installing Visual Studio Code

    mkdir isaacsim
    unzip ~/Downloads/isaac-sim-standalone-5.1.0-linux-x86_64.zip -d isaacsim
    cd isaacsim
    ./post_install.sh
    ./isaac-sim.sh # after the initial opening, an APP "Isaac Sim" actually appears

    sudo apt install nvtop
    nvtop                        # for monitoring GPU
    sudo apt install psensor
    # check psensor App, for monitoring temperatures 
    # check also APP "NVIDIA X Server Settings" to adjust GPU settings


To load the assets locally, follow <a href="https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_faq.html">"Assets" guide</a> and get zip file from <a href="https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/download.html">download links (3 parts)</a>:

    mkdir .../Kit/assets
    cd .../Kit/assets
    7z x isaac-sim-assets-complete-5.1.0.zip.001
    vim .../isaacsim/apps/isaacsim.exp.base.kit
    # following: https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_faq.html
    # find: 
    ## [settings]
    # add below:
    ## persistent.isaac.asset_root.default = ".../Kit/assets/Isaac/5.1"
    ## exts."isaacsim.gui.content_browser".folders = [
    ##     ".../Kit/assets/Isaac/5.1/Isaac/Robots",
    ##     ".../Kit/assets/Isaac/5.1/Isaac/People",
    ##     ".../Kit/assets/Isaac/5.1/Isaac/IsaacLab",
    ##     ".../Kit/assets/Isaac/5.1/Isaac/Props",
    ##     ".../Kit/assets/Isaac/5.1/Isaac/Environments",
    ##     ".../Kit/assets/Isaac/5.1/Isaac/Materials",
    ##     ".../Kit/assets/Isaac/5.1/Isaac/Samples",
    ##     ".../Kit/assets/Isaac/5.1/Isaac/Sensors",
    ## ]
    # open Issac Sim
    # load the assets directly from "Content" on the bottom left

To install Isaac Lab, follow <a href="https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html">installation guide</a>:

    mkdir miniconda3
    cd miniconda3/
    wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
    bash Miniconda3-latest-Linux-x86_64.sh -b -u -p .
    ./bin/conda init bash
    source /home/cubicdoggo/.bashrc

    conda create -n isaaclab_env python=3.11
    # update .bashrc
    # close terminal and restart
    conda activate isaaclab_env
    pip install --upgrade pip
    pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
    pip install -U torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
    git clone https://github.com/isaac-sim/IsaacLab.git
    sudo apt install cmake build-essential
    mv IsaacLab/ isaaclab
    cd isaaclab
    mkdir -p /home/cubicdoggo/Documents/miniconda3/envs/isaaclab_env/lib/python3.11/site-packages/isaacsim/.vscode
    ln -sf /home/cubicdoggo/Documents/isaacsim/.vscode/settings.json /home/cubicdoggo/Documents/miniconda3/envs/isaaclab_env/lib/python3.11/site-packages/isaacsim/.vscode/settings.json
    ./isaaclab.sh --install 

And to check for the installation (may need to wait a bit):

    python scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Velocity-Rough-Anymal-C-v0 --num_envs=1000
    python scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Velocity-Rough-Anymal-C-v0 --headless
    # check "Mean reward", wait 1 hr
    python scripts/reinforcement_learning/rsl_rl/play.py --task=Isaac-Velocity-Rough-Anymal-C-v0 --num_envs=20
    # to find the saved trained files:
    cd /home/cubicdoggo/Documents/isaaclab/logs/rsl_rl/anymal_c_rough

### URDF Import

    ros2 run xacro xacro cubic_doggo.urdf.xacro > cubic_doggo.urdf
    # change: 
    ## package:/ => full path
    # remove: 
    ## </joint><link name="world"/> ## <joint name="world_base_link" type="fixed">
    ##   <parent link="world"/>
    ##   <child  link="base_link"/>
    ## <origin xyz="0 0 0" rpy="0 0 0"/>

    # rm ~/.cache/ # if needed
    # File => Import => ../ROS_leggedRobot_testBed/src/my_robot_description/urdf/cubic_doggo.urdf
    # Model => Create in Stage
    # Links => Movable Base 
    # Joints & Drives => Joint Configuration => Stiffness
    # Collider Type => Convex Decomposition
    # Import
    # Check: Console (bottom left) or Terminal for errors

    # Stage Light (center-right top) => Grey Studio 
    # Create => Physics => Ground plane => pull it under World
    ## Rotate the robot upright
    # Tools => Physics => Physics Inspector => cubic_doggo (defaultPrim) => Refresh
    ## Move joints to the correct positions (although inaccurate) => click green tick to confirm
    # Run (left panel)

<img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/IsaacSimImport.png" width="600">
