#include <iostream>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "my_robot_firmware/hardware_interface_cubic_doggo_dynamixel_u2d2_xl430.hpp"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace cubic_doggo_namespace {
    hardware_interface::CallbackReturn HardwareInterfaceU2D2_cubic_doggo::on_init
        (const hardware_interface::HardwareComponentInterfaceParams &params) 
    {
        if (hardware_interface::SystemInterface::on_init(params) !=
            hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(get_logger(), "hardware_interface:on_init()");

        dxl_return_ = dxl_wb_.init(PORT_NAME, BAUD_RATE, &log_);
        if (dxl_return_ == false) {
            RCLCPP_ERROR(get_logger(), "hardware_interface:on_init(): failed to open the port %s!", PORT_NAME);
            return hardware_interface::CallbackReturn::ERROR;
        } else {
            RCLCPP_INFO(get_logger(), "hardware_interface:on_init(): initialize with baud rate: %d", BAUD_RATE);
        } 


        /////////// see: src/my_robot_description/urdf/cubic_doggo.ros2_control.xacro
        servo_channels_[0]  = std::stoi(params.hardware_info.hardware_parameters.at("servo1_channel_FL"));
        servo_channels_[1]  = std::stoi(params.hardware_info.hardware_parameters.at("servo2_channel_FL"));
        servo_channels_[2]  = std::stoi(params.hardware_info.hardware_parameters.at("servo3_channel_FL"));
        servo_channels_[3]  = std::stoi(params.hardware_info.hardware_parameters.at("servo1_channel_FR"));
        servo_channels_[4]  = std::stoi(params.hardware_info.hardware_parameters.at("servo2_channel_FR"));
        servo_channels_[5]  = std::stoi(params.hardware_info.hardware_parameters.at("servo3_channel_FR"));
        servo_channels_[6]  = std::stoi(params.hardware_info.hardware_parameters.at("servo1_channel_BL"));
        servo_channels_[7]  = std::stoi(params.hardware_info.hardware_parameters.at("servo2_channel_BL"));
        servo_channels_[8]  = std::stoi(params.hardware_info.hardware_parameters.at("servo3_channel_BL"));
        servo_channels_[9]  = std::stoi(params.hardware_info.hardware_parameters.at("servo1_channel_BR"));
        servo_channels_[10] = std::stoi(params.hardware_info.hardware_parameters.at("servo2_channel_BR"));
        servo_channels_[11] = std::stoi(params.hardware_info.hardware_parameters.at("servo3_channel_BR"));
        joint_names = {"servo1_servo1_padding_FL", 
                       "servo2_servo2_padding_FL", 
                       "servo3_calfFeet_FL",
                       "servo1_servo1_padding_FR",
                       "servo2_servo2_padding_FR",
                       "servo3_calfFeet_FR",
                       "servo1_servo1_padding_BL",
                       "servo2_servo2_padding_BL",
                       "servo3_calfFeet_BL",
                       "servo1_servo1_padding_BR",
                       "servo2_servo2_padding_BR",
                       "servo3_calfFeet_BR"};
        rad_positions_init_[0]  = 3.14;                      
        rad_positions_init_[1]  = 2.54;                      //DXL_PI - DXL_PI/4.0;
        rad_positions_init_[2]  = 4.14;                      //DXL_PI - DXL_PI/8.0;
        rad_positions_init_[3]  = 3.14;                      
        rad_positions_init_[4]  = 3.74;                      //DXL_PI + DXL_PI/4.0;
        rad_positions_init_[5]  = 2.14;                      //DXL_PI + DXL_PI/8.0;
        rad_positions_init_[6]  = 3.14;                     
        rad_positions_init_[7]  = 2.54;                      //DXL_PI - DXL_PI/4.0;
        rad_positions_init_[8]  = 4.14;                      //DXL_PI - DXL_PI/8.0;
        rad_positions_init_[9]  = 3.14;                      
        rad_positions_init_[10] = 3.74;                      //DXL_PI + DXL_PI/4.0;
        rad_positions_init_[11] = 2.14;                      //DXL_PI + DXL_PI/8.0;
        ///////////


        return hardware_interface::CallbackReturn::SUCCESS;     
    }
    hardware_interface::CallbackReturn HardwareInterfaceU2D2_cubic_doggo::on_configure 
        (const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(get_logger(), "hardware_interface:on_configure()");
        (void) previous_state;
        
        for (uint8_t servo_idx = 0; servo_idx < servo_N_; servo_idx++) {
            dxl_return_ = dxl_wb_.ping(servo_channels_[servo_idx], &model_number_, &log_);
            if (dxl_return_ == false) {
                RCLCPP_ERROR(get_logger(), "hardware_interface:on_configure(): failed to ping!");
                return hardware_interface::CallbackReturn::ERROR;
            } else {
                RCLCPP_INFO(get_logger(), "hardware_interface:on_configure(): pinging id: %d, model_number : %d\n", 
                                          servo_channels_[servo_idx], model_number_);
            }
            // int32_t velocity = 0, int32_t acceleration = 0 => position mode
            dxl_return_ = dxl_wb_.jointMode(servo_channels_[servo_idx], 0, 0, &log_);
            if (dxl_return_ == false) {
                RCLCPP_ERROR(get_logger(), "hardware_interface:on_configure(): failed join position mode!");
                return hardware_interface::CallbackReturn::ERROR;
            } else {
                RCLCPP_INFO(get_logger(), "hardware_interface:on_configure(): "
                                          "position mode for ch %d, model_number : %d\n", 
                                          servo_channels_[servo_idx], model_number_);
            }
            dxl_wb_.itemWrite(servo_channels_[servo_idx], "Current_Limit",   800, &log_);   // torque: current limit
            dxl_wb_.itemWrite(servo_channels_[servo_idx], "Position_P_Gain", 600, &log_);   // PID: lower P gain
            dxl_wb_.itemWrite(servo_channels_[servo_idx], "Position_D_Gain", 100, &log_);   // PID: adding damping
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
    hardware_interface::CallbackReturn HardwareInterfaceU2D2_cubic_doggo::on_activate  
        (const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(get_logger(), "hardware_interface:on_activate()");
        (void) previous_state;
       
        for (uint8_t servo_idx = 0; servo_idx < servo_N_; servo_idx++) initialize_servo_(servo_idx);
        dxl_return_ = dxl_wb_.syncWrite(handler_index_write_pos_, servo_channels_, servo_N_, dxl_positions_, 1,&log_);
        for (uint8_t servo_idx = 0; servo_idx < servo_N_; servo_idx++) {
            set_state(joint_names[servo_idx]+"/position", rad_positions_[servo_idx]);
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn HardwareInterfaceU2D2_cubic_doggo::on_deactivate
        (const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(get_logger(), "hardware_interface:on_deactivate()");
        (void) previous_state;
        for (uint8_t servo_idx = 0; servo_idx < servo_N_; servo_idx++) initialize_servo_(servo_idx);
        dxl_return_ = dxl_wb_.syncWrite(handler_index_write_pos_, servo_channels_, servo_N_, dxl_positions_, 1,&log_);

        return hardware_interface::CallbackReturn::SUCCESS;
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    hardware_interface::return_type HardwareInterfaceU2D2_cubic_doggo::read 
        (const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
        RCLCPP_DEBUG(get_logger(), "hardware_interface:read()");
        (void) period;
        if (write_first_call_ == true) {
            start_time_ = time;
            write_first_call_ = false;
        }
        rclcpp::Duration lifetime = time - start_time_;
    
        dxl_return_ = dxl_wb_.syncRead(handler_index_read_pos_, servo_channels_, servo_N_, &log_);
        if (dxl_return_ == false) {
            RCLCPP_ERROR(get_logger(), "hardware_interface:read(): syncRead fails");
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
            // see: src/my_robot_description/urdf/cubic_doggo.ros2_control.xacro
            set_state(joint_names[servo_idx]+"/position", rad_positions_ [servo_idx]);
            set_state(joint_names[servo_idx]+"/velocity", rad_velocities_[servo_idx]);
            set_state(joint_names[servo_idx]+"/effort",   rad_efforts_   [servo_idx]);
        }
        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type HardwareInterfaceU2D2_cubic_doggo::write
        (const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
        RCLCPP_DEBUG(get_logger(), "hardware_interface:write()");
        (void) time;
        (void) period; 
        
        // see: src/my_robot_description/urdf/cubic_doggo.ros2_control.xacro
        for (uint8_t servo_idx = 0; servo_idx < servo_N_; servo_idx++) {
            rad_positions_[servo_idx] = get_command(joint_names[servo_idx]+"/position");
            if (std::isnan(rad_positions_[servo_idx]) == true) initialize_servo_(servo_idx); 
            dxl_positions_[servo_idx] = (int32_t)(rad_positions_[servo_idx]*(MAX_POSITION-MIN_POSITION)/(2.0*DXL_PI));
        }
        dxl_return_ = dxl_wb_.syncWrite(handler_index_write_pos_, servo_channels_, servo_N_, dxl_positions_, 1,&log_);
        if (dxl_return_ == false) {
            RCLCPP_ERROR(get_logger(), "hardware_interface:write(): syncWrite fails");
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void HardwareInterfaceU2D2_cubic_doggo::initialize_servo_(uint8_t servo_id) {
        rad_positions_[servo_id] = rad_positions_init_[servo_id];
        dxl_positions_[servo_id] = (int32_t)(rad_positions_[servo_id]*(MAX_POSITION-MIN_POSITION)/(2.0*DXL_PI));
        rad_velocities_[servo_id] = 0.0;
        dxl_velocities_[servo_id] = 0;
        rad_efforts_[servo_id] = 0.0;
        dxl_efforts_[servo_id] = 0;
    }
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cubic_doggo_namespace::HardwareInterfaceU2D2_cubic_doggo, hardware_interface::SystemInterface)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////









