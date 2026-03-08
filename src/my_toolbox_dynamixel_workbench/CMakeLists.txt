cmake_minimum_required(VERSION 3.8)
project(my_toolbox_dynamixel_workbench)

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
add_executable(model_scan src/a_Model_Scan.cpp)
ament_target_dependencies(model_scan dynamixel_workbench rclcpp dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox)

add_executable(ping src/b_Ping.cpp)
ament_target_dependencies(ping dynamixel_workbench rclcpp dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox)

add_executable(id_change src/c_ID_Change.cpp)
ament_target_dependencies(id_change dynamixel_workbench rclcpp dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox)

add_executable(bps_change src/d_BPS_Change.cpp)
ament_target_dependencies(bps_change dynamixel_workbench rclcpp dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox)

add_executable(mode_change src/e_Mode_Change.cpp)
ament_target_dependencies(mode_change dynamixel_workbench rclcpp dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox)

add_executable(reboot src/f_Reboot.cpp)
ament_target_dependencies(reboot dynamixel_workbench rclcpp dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox)

add_executable(reset src/g_Reset.cpp)
ament_target_dependencies(reset dynamixel_workbench rclcpp dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox)

add_executable(position src/h_Position.cpp)
ament_target_dependencies(position dynamixel_workbench rclcpp dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox)

add_executable(velocity src/i_Velocity.cpp)
ament_target_dependencies(velocity dynamixel_workbench rclcpp dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox)

add_executable(current_based_position src/j_Current_Based_Position.cpp)
ament_target_dependencies(current_based_position dynamixel_workbench rclcpp dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox)

add_executable(read_write src/k_Read_Write.cpp)
ament_target_dependencies(read_write dynamixel_workbench rclcpp dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox)

add_executable(sync_write src/l_Sync_Write.cpp)
ament_target_dependencies(sync_write dynamixel_workbench rclcpp dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox)

add_executable(sync_read_write src/m_Sync_Read_Write.cpp)
ament_target_dependencies(sync_read_write dynamixel_workbench rclcpp dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox)

add_executable(bulk_read_write src/n_Bulk_Read_Write.cpp)
ament_target_dependencies(bulk_read_write dynamixel_workbench rclcpp dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox)

add_executable(find_dynamixel src/o_Find_Dynamixel.cpp)
ament_target_dependencies(find_dynamixel dynamixel_workbench rclcpp dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox)

add_executable(monitor src/p_Monitor.cpp)
ament_target_dependencies(monitor dynamixel_workbench rclcpp dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox)
target_compile_features(monitor PUBLIC cxx_std_11)

install(TARGETS model_scan ping id_change bps_change mode_change reboot reset position velocity current_based_position read_write sync_write sync_read_write bulk_read_write find_dynamixel monitor
    DESTINATION lib/${PROJECT_NAME}
)


ament_package()




