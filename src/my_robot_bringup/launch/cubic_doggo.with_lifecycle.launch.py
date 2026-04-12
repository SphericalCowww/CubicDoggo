from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder
import os

######################################################################################################################
def generate_launch_description():
    robot_description_path   = get_package_share_path("my_robot_description")
    robot_bringup_path       = get_package_share_path("my_robot_bringup")
    robot_moveit_config_path = get_package_share_path("cubic_doggo_moveit_config")   
 
    urdf_path          = os.path.join(robot_description_path,   "urdf",   "cubic_doggo.urdf.xacro")
    robot_controllers  = os.path.join(robot_bringup_path,       "config", "cubic_doggo_controllers.yaml")
    moveit_config_path = os.path.join(robot_moveit_config_path, "launch", "move_group.launch.py")
    rviz_config_path   = os.path.join(robot_description_path,   "rviz",   "cubic_doggo.urdf_config.rviz")

    robot_description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}],
    )

    moveit_config = (
        MoveItConfigsBuilder("cubic_doggo", package_name="cubic_doggo_moveit_config")
        .robot_description(file_path=urdf_path)
        .to_moveit_configs()
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            robot_controllers,
            {'use_steady_clock_for_update': False},],
    )
    # check src/my_robot_bringup/config/cubic_doggo_controllers.yaml 
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    all_legs_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["all_legs_controller"],
    )
    moveit_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_config_path),
        launch_arguments={}.items(),
    )
    lifecycle_node = Node(
        package="my_robot_commander",
        executable="cubic_doggo_lifecycle",
        parameters=[
            moveit_config.robot_description,           # the URDF math
            moveit_config.robot_description_semantic,  # the SRDF 
            moveit_config.robot_description_kinematics,# the kinematics.yaml
            moveit_config.joint_limits,                # the joint_limits.yaml 
            {"jump_threshold": 0.15},                  # for computeCartesianPath
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    joy_driver_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[{
            "deadzone": 0.05,
            "autorepeat_rate": 20.0,
            # "device_id": 0            # force a specific controller if needed
        }]
    )
    joy_controller_node = Node(
        package="my_robot_controller",
        executable="cubic_doggo_joy_control",
        remappings=[("joy", "/joy")] 
    )
    
    return LaunchDescription([
        SetParameter(name="jump_threshold", value=0.15),
        SetParameter(name="use_sim_time", value=False),
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        all_legs_controller_spawner,
        moveit_launcher,
        lifecycle_node,
        #rviz_node,
        joy_driver_node,
        joy_controller_node,
    ])

######################################################################################################################
