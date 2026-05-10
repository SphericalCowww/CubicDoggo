#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "example_interfaces/msg/string.hpp"
#include "my_robot_interface/msg/cubic_doggo_leg_feet_target.hpp"
#include "std_srvs/srv/set_bool.hpp"
using custom_feet_array = my_robot_interface::msg::CubicDoggoLegFeetTarget;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class CubicDoggoJoyControl : public rclcpp::Node {
public:
    CubicDoggoJoyControl() : Node("cubic_doggo_joy_control") {
        auto joy_qos = rclcpp::QoS(10).reliable();
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 
            joy_qos,
            std::bind(&CubicDoggoJoyControl::joy_callback_, this, std::placeholders::_1)
        );
        command_pub_ = this->create_publisher<example_interfaces::msg::String>("/leg_set_named", 10);
        walk_client_ = this->create_client<std_srvs::srv::SetBool>            ("/leg_walk_toggle");
        feet_pub_    = this->create_publisher<custom_feet_array>              ("/leg_set_feet", 10);
        RCLCPP_INFO(this->get_logger(), "CubicDoggoJoyControl:constructor()"
                                        "controller node started, listening on /joy...");
    }

private:
    void joy_callback_(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (prev_buttons_.size() != msg->buttons.size()) {
            prev_buttons_.resize(msg->buttons.size(), 0);
        }
        if (msg->buttons.size() > 3) {
            if (msg->buttons[3] == 1 && prev_buttons_[3] == 0) {
                send_pose_("stand");
            }
            if (msg->buttons[2] == 1 && prev_buttons_[2] == 0) {
                send_pose_("sit");
            }
            if (msg->buttons[1] == 1 && prev_buttons_[1] == 0) {
                send_pose_("bow");
            }
            if (msg->buttons[0] == 1 && prev_buttons_[0] == 0) {
                send_pose_("rest");
            }
        }
        prev_buttons_ = msg->buttons;
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if (prev_axes_.size() != msg->axes.size()) {
            prev_axes_.resize(msg->axes.size(), 0);
        }
        if (msg->axes.size() > 7) {
            if (msg->axes[7] > 0.5 && prev_axes_[7] <= 0.5) {
                call_walk_(true);
            }
            if (msg->axes[7] < -0.5 && prev_axes_[7] >= -0.5) {
                call_walk_(false);
            }
           
            double deadzone = 0.05; 
            auto feet_msg = custom_feet_array();
            double raw_x = msg->axes[3];  
            double raw_y = msg->axes[4];
            feet_msg.x = (std::abs(raw_x) > deadzone) ? raw_x : 0.0;
            feet_msg.y = (std::abs(raw_y) > deadzone) ? raw_y : 0.0;
            feet_pub_->publish(feet_msg);
        }

        prev_axes_ = msg->axes;
    }
    void send_pose_(std::string pose_name) {
        example_interfaces::msg::String out_msg;
        out_msg.data = pose_name;
        command_pub_->publish(out_msg);
        RCLCPP_INFO(this->get_logger(), "CubicDoggoJoyControl:send_pose_(): '%s' command sent", pose_name.c_str());
    }
    void call_walk_(bool walk_state) {
        if (!walk_client_->wait_for_service(std::chrono::milliseconds(500))) {
            RCLCPP_WARN(this->get_logger(), "CubicDoggoJoyControl:call_walk_(): "
                                            "service /leg_walk_toggle not available!");
            return;
        }
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = walk_state;
        auto result = walk_client_->async_send_request(request);
        std::string walk_state_str = walk_state ? "true" : "false";
        RCLCPP_INFO(this->get_logger(), "CubicDoggoJoyControl:call_walk_(): walk state '%s' sent", 
                                        walk_state_str.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr        joy_subscriber_;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr command_pub_;
    rclcpp::Publisher<custom_feet_array>::SharedPtr               feet_pub_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr             walk_client_;
    std::vector<int>   prev_buttons_;
    std::vector<float> prev_axes_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CubicDoggoJoyControl>());
    rclcpp::shutdown();
    return 0;
}


