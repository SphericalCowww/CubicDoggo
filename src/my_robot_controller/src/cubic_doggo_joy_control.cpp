#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "example_interfaces/msg/string.hpp"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class CubicDoggoJoyControl : public rclcpp::Node {
public:
    CubicDoggoJoyControl() : Node("cubic_doggo_joy_control") {
        auto joy_qos = rclcpp::QoS(10).reliable();
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 
            joy_qos,
            std::bind(&CubicDoggoJoyControl::joy_callback, this, std::placeholders::_1)
        );
        command_pub_ = this->create_publisher<example_interfaces::msg::String>("/leg_set_named", 10);
        RCLCPP_INFO(this->get_logger(), "CubicDoggoJoyControl:constructor()"
                                        "controller node started, listening on /joy...");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (prev_buttons_.size() != msg->buttons.size()) {
            prev_buttons_.resize(msg->buttons.size(), 0);
        }
        if (msg->buttons.size() > 3) {
            if (msg->buttons[3] == 1 && prev_buttons_[3] == 0) {
                example_interfaces::msg::String out_msg;
                out_msg.data = "stand";
                command_pub_->publish(out_msg);
                RCLCPP_INFO(this->get_logger(), "CubicDoggoJoyControl:joy_callback():"
                                                "joy button 'Y' pressed, 'stand' command sent");
            }
            if (msg->buttons[2] == 1 && prev_buttons_[2] == 0) {
                example_interfaces::msg::String out_msg;
                out_msg.data = "sit";
                command_pub_->publish(out_msg);
                RCLCPP_INFO(this->get_logger(), "CubicDoggoJoyControl:joy_callback():"
                                                "joy button 'X' pressed, 'sit' command sent");
            }
            if (msg->buttons[1] == 1 && prev_buttons_[1] == 0) {
                example_interfaces::msg::String out_msg;
                out_msg.data = "bow";
                command_pub_->publish(out_msg);
                RCLCPP_INFO(this->get_logger(), "CubicDoggoJoyControl:joy_callback():"
                                                "joy button 'B' pressed, 'bow' command sent");
            }
            if (msg->buttons[0] == 1 && prev_buttons_[0] == 0) {
                example_interfaces::msg::String out_msg;
                out_msg.data = "rest";
                command_pub_->publish(out_msg);
                RCLCPP_INFO(this->get_logger(), "CubicDoggoJoyControl:joy_callback():"
                                                "joy button 'A' pressed, 'rest' command sent");
            }
        }
        prev_buttons_ = msg->buttons;
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr command_pub_;
    std::vector<int> prev_buttons_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CubicDoggoJoyControl>());
    rclcpp::shutdown();
    return 0;
}


