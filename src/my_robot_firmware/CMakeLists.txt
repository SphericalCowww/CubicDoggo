cmake_minimum_required(VERSION 3.8)
project(my_robot_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_workbench REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

###########################################################################
add_library(cubic_doggo_hardware_interface SHARED
  src/hardware_interface_cubic_doggo_dynamixel_u2d2_xl430.cpp
)
ament_target_dependencies(cubic_doggo_hardware_interface rclcpp rclcpp_lifecycle dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox hardware_interface pluginlib)
pluginlib_export_plugin_description_file(hardware_interface cubic_doggo_hardware_interface.xml)
install(TARGETS cubic_doggo_hardware_interface
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY   include
  DESTINATION include
) 
ament_export_libraries(
    cubic_doggo_hardware_interface
)
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib)



ament_package()



