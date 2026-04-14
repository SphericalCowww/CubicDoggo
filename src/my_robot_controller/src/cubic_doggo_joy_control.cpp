#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "example_interfaces/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

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
        if (msg->buttons.size() > 7) {
            if (msg->buttons[7] == 1 && prev_buttons_[7] == 0) {
                call_walk_(true);
            }
            if (msg->buttons[6] == 1 && prev_buttons_[6] == 0) {
                call_walk_(false);
            }
        }
        prev_buttons_ = msg->buttons;
    }
    void send_pose_(std::string pose_name) {
        example_interfaces::msg::String out_msg;
        out_msg.data = pose_name;
        command_pub_->publish(out_msg);
        RCLCPP_INFO(this->get_logger(), "CubicDoggoJoyControl:send_pose_(): '%s' command sent", pose_name.c_str());
    }
    void call_walk_(bool walk_state) {
        if (!walk_client_->wait_for_service(std::chrono::milliseconds(500))) {
            RCLCPP_WARN(this->get_logger(), "Service /leg_walk_toggle not available!");
            return;
        }
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = walk_state;
        auto result = walk_client_->async_send_request(request);
        std::string walk_state_str = walk_state ? "true" : "false";
        RCLCPP_INFO(this->get_logger(), "CubicDoggoJoyControl:call_walk_(): walk state '%s' sent", 
                                        walk_state_str.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr command_pub_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr walk_client_;
    std::vector<int> prev_buttons_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CubicDoggoJoyControl>());
    rclcpp::shutdown();
    return 0;
}


