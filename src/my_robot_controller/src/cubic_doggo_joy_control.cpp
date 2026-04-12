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
        RCLCPP_INFO(this->get_logger(), "Jazzy Joy Node Started. Listening on /joy...");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Jazzy Joy data flowing!");
        if (msg->buttons.size() > 3) {
            if (msg->buttons[3] == 1 && prev_y_state_ == 0) {
                auto out_msg = example_interfaces::msg::String();
                out_msg.data = "stand";
                command_pub_->publish(out_msg);
                RCLCPP_INFO(this->get_logger(), "Xbox 'Y' -> STAND Command Sent");
            }
            prev_y_state_ = msg->buttons[3];
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr command_pub_;
    int prev_y_state_ = 0;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CubicDoggoJoyControl>());
    rclcpp::shutdown();
    return 0;
}


