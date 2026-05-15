#ifndef HARDWARE_INTERFACE_MY_ROBOT_DYNAMIXEL_U2D2_XL430_HPP
#define HARDWARE_INTERFACE_MY_ROBOT_DYNAMIXEL_U2D2_XL430_HPP

#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

#define PORT_NAME    "/dev/ttyU2D2"
#define BAUD_RATE    2000000
#define MIN_POSITION 0.0
#define MAX_POSITION 4095.0
#define DXL_PI       3.1415926

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace my_robot_namespace {
    class HardwareInterfaceU2D2_my_robot: public hardware_interface::SystemInterface
    {
        public:
            // interface override
            hardware_interface::CallbackReturn 
                on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;
            hardware_interface::return_type 
                read(const rclcpp::Time & time, const rclcpp::Duration &period) override;
            hardware_interface::return_type 
                write(const rclcpp::Time & time, const rclcpp::Duration &period) override;
            // lifecycle node override
            hardware_interface::CallbackReturn 
                on_configure(const rclcpp_lifecycle::State &previous_state) override;
            hardware_interface::CallbackReturn 
                on_activate(const rclcpp_lifecycle::State &previous_state) override;
            hardware_interface::CallbackReturn 
                on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        private:
            bool debug_bool = false;

            std::shared_ptr<rclcpp::Node> node_;
            DynamixelWorkbench dxl_wb_;
            uint16_t model_number_ = 0;
            const char *log_;
            bool dxl_return_ = false;

            bool write_first_call_ = true;
            rclcpp::Time start_time_;

            uint8_t servo_N_ = 3;
            std::vector<std::string> joint_names;
            uint8_t servo_channels_[3];
            double  rad_positions_init_[3];
            double  rad_positions_ [3];
            double  rad_velocities_[3];
            double  rad_efforts_   [3];
            int32_t dxl_positions_ [3];
            int32_t dxl_velocities_[3];
            int32_t dxl_efforts_   [3];
            uint8_t handler_index_write_pos_;
            uint8_t handler_index_read_pos_;
            uint8_t handler_index_read_vel_;
            uint8_t handler_index_read_eff_;

            void initialize_servo_(uint8_t servo_id);
    };    
}

#endif
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
