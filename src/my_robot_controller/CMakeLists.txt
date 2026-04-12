cmake_minimum_required(VERSION 3.8)
project(my_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

###########################################################################
add_executable(cubic_doggo_teleop_key src/cubic_doggo_teleop_key.cpp)
ament_target_dependencies(cubic_doggo_teleop_key rclcpp example_interfaces)

add_executable(cubic_doggo_joy_control src/cubic_doggo_joy_control.cpp)
ament_target_dependencies(cubic_doggo_joy_control rclcpp example_interfaces sensor_msgs)

install(TARGETS cubic_doggo_teleop_key cubic_doggo_joy_control
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
