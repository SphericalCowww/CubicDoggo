#include <iostream>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "my_robot_firmware/hardware_interface_my_robot_dynamixel_u2d2_xl430.hpp"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace my_robot_namespace {
    hardware_interface::CallbackReturn HardwareInterfaceU2D2_my_robot::on_init
        (const hardware_interface::HardwareComponentInterfaceParams &params) 
    {
        if (hardware_interface::SystemInterface::on_init(params) !=
            hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(get_logger(), "on_init()");

        dxl_return_ = dxl_wb_.init(PORT_NAME, BAUD_RATE, &log_);
        if (dxl_return_ == false) {
            RCLCPP_ERROR(get_logger(), "on_init(): failed to open the port %s!", PORT_NAME);
            return hardware_interface::CallbackReturn::ERROR;
        } else {
            RCLCPP_INFO(get_logger(), "on_init(): initialize with baud rate: %d", BAUD_RATE);
        } 


        /////////// see: src/my_robot_description/urdf/my_robot.ros2_control.xacro
        servo_channels_[0] = std::stoi(params.hardware_info.hardware_parameters.at("servo1_channel"));
        servo_channels_[1] = std::stoi(params.hardware_info.hardware_parameters.at("servo2_channel"));
        servo_channels_[2] = std::stoi(params.hardware_info.hardware_parameters.at("servo3_channel"));
        joint_names = {"servo1_servo1_padding", 
                       "servo2_servo2_padding", 
                       "servo3_calfFeet"};
        rad_positions_init_[0] = DXL_PI;
        rad_positions_init_[1] = DXL_PI;
        rad_positions_init_[2] = DXL_PI;
        ///////////


        return hardware_interface::CallbackReturn::SUCCESS;     
    }
    hardware_interface::CallbackReturn HardwareInterfaceU2D2_my_robot::on_configure 
        (const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(get_logger(), "on_configure()");
        (void) previous_state;
        
        for (uint8_t servo_idx = 0; servo_idx < servo_N_; servo_idx++) {
            dxl_return_ = dxl_wb_.ping(servo_channels_[servo_idx], &model_number_, &log_);
            if (dxl_return_ == false) {
                RCLCPP_ERROR(get_logger(), "on_configure(): failed to ping!");
                return hardware_interface::CallbackReturn::ERROR;
            } else {
                RCLCPP_INFO(get_logger(), "on_configure(): pinging id: %d, model_number : %d\n", 
                                          servo_channels_[servo_idx], model_number_);
            }
            // int32_t velocity = 0, int32_t acceleration = 0 => position mode
            dxl_return_ = dxl_wb_.jointMode(servo_channels_[servo_idx], 0, 0, &log_);
            if (dxl_return_ == false) {
                RCLCPP_ERROR(get_logger(), "on_configure(): failed join position mode!");
                return hardware_interface::CallbackReturn::ERROR;
            } else {
                RCLCPP_INFO(get_logger(), "on_configure(): position mode for ch %d, model_number : %d\n", 
                                          servo_channels_[servo_idx], model_number_);
            }
        }
        dxl_wb_.addSyncReadHandler(servo_channels_[0], "Present_Position", &log_);
        handler_index_read_pos_ = dxl_wb_.getTheNumberOfSyncReadHandler() - 1;
        dxl_wb_.addSyncReadHandler(servo_channels_[0], "Present_Velocity", &log_);
        handler_index_read_vel_ = dxl_wb_.getTheNumberOfSyncReadHandler() - 1;
        dxl_wb_.addSyncReadHandler(servo_channels_[0], "Present_Current", &log_);
        handler_index_read_eff_ = dxl_wb_.getTheNumberOfSyncReadHandler() - 1;
        dxl_wb_.addSyncWriteHandler(servo_channels_[0], "Goal_Position", &log_);
        handler_index_write_pos_ = dxl_wb_.getTheNumberOfSyncWriteHandler() - 1;
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn HardwareInterfaceU2D2_my_robot::on_activate  
        (const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(get_logger(), "on_activate()");
        (void) previous_state;
       
        for (uint8_t servo_idx = 0; servo_idx < servo_N_; servo_idx++) initialize_servo_(servo_idx);
        dxl_return_ = dxl_wb_.syncWrite(handler_index_write_pos_, servo_channels_, servo_N_, dxl_positions_, 1,&log_);
        for (uint8_t servo_idx = 0; servo_idx < servo_N_; servo_idx++) {
            set_state(joint_names[servo_idx]+"/position", rad_positions_[servo_idx]);
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn HardwareInterfaceU2D2_my_robot::on_deactivate
        (const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(get_logger(), "on_deactivate()");
        (void) previous_state;
        for (uint8_t servo_idx = 0; servo_idx < servo_N_; servo_idx++) initialize_servo_(servo_idx);
        dxl_return_ = dxl_wb_.syncWrite(handler_index_write_pos_, servo_channels_, servo_N_, dxl_positions_, 1,&log_);

        return hardware_interface::CallbackReturn::SUCCESS;
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    hardware_interface::return_type HardwareInterfaceU2D2_my_robot::read 
        (const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
        RCLCPP_DEBUG(get_logger(), "read()");
        (void) period;
        if (write_first_call_ == true) {
            start_time_ = time;
            write_first_call_ = false;
        }
        rclcpp::Duration lifetime = time - start_time_;
    
        dxl_return_ = dxl_wb_.syncRead(handler_index_read_pos_, servo_channels_, servo_N_, &log_);
        if (dxl_return_ == false) {
            RCLCPP_ERROR(get_logger(), "read(): syncRead fails");
            return hardware_interface::return_type::ERROR;
        }
        dxl_wb_.getSyncReadData(handler_index_read_pos_, servo_channels_, servo_N_, dxl_positions_,  &log_);
        dxl_wb_.syncRead(handler_index_read_vel_, servo_channels_, servo_N_, &log_);
        dxl_wb_.getSyncReadData(handler_index_read_vel_, servo_channels_, servo_N_, dxl_velocities_, &log_);
        dxl_wb_.syncRead(handler_index_read_eff_, servo_channels_, servo_N_, &log_);
        dxl_wb_.getSyncReadData(handler_index_read_eff_, servo_channels_, servo_N_, dxl_efforts_,    &log_);
        for (uint8_t servo_idx = 0; servo_idx < servo_N_; servo_idx++) {
            rad_positions_ [servo_idx] = (double) dxl_positions_ [servo_idx]*(2.0*DXL_PI)/(MAX_POSITION-MIN_POSITION);
            rad_velocities_[servo_idx] = (double) dxl_velocities_[servo_idx]*0.229*(2.0*DXL_PI/60.0);
            rad_efforts_   [servo_idx] = (double) dxl_efforts_   [servo_idx]; 
            // see: src/my_robot_description/urdf/my_robot.ros2_control.xacro
            set_state(joint_names[servo_idx]+"/position", rad_positions_ [servo_idx]);
            set_state(joint_names[servo_idx]+"/velocity", rad_velocities_[servo_idx]);
            set_state(joint_names[servo_idx]+"/effort",   rad_efforts_   [servo_idx]);
        }
        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type HardwareInterfaceU2D2_my_robot::write
        (const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
        RCLCPP_DEBUG(get_logger(), "write()");
        (void) time;
        (void) period; 
        
        // see: src/my_robot_description/urdf/my_robot.ros2_control.xacro
        for (uint8_t servo_idx = 0; servo_idx < servo_N_; servo_idx++) {
            rad_positions_[servo_idx] = get_command(joint_names[servo_idx]+"/position");
            if (std::isnan(rad_positions_[servo_idx]) == true) initialize_servo_(servo_idx); 
            dxl_positions_[servo_idx] = (int32_t)(rad_positions_[servo_idx]*(MAX_POSITION-MIN_POSITION)/(2.0*DXL_PI));
        }
        dxl_return_ = dxl_wb_.syncWrite(handler_index_write_pos_, servo_channels_, servo_N_, dxl_positions_, 1,&log_);
        if (dxl_return_ == false) {
            RCLCPP_ERROR(get_logger(), "write(): syncWrite fails");
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void HardwareInterfaceU2D2_my_robot::initialize_servo_(uint8_t servo_id) {
        rad_positions_[servo_id] = rad_positions_init_[servo_id];
        dxl_positions_[servo_id] = (int32_t)(rad_positions_[servo_id]*(MAX_POSITION-MIN_POSITION)/(2.0*DXL_PI));
        rad_velocities_[servo_id] = 0.0;
        dxl_velocities_[servo_id] = 0;
        rad_efforts_[servo_id] = 0.0;
        dxl_efforts_[servo_id] = 0;
    }
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot_namespace::HardwareInterfaceU2D2_my_robot, hardware_interface::SystemInterface)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////









