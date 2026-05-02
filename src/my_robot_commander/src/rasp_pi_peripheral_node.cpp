#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <fstream>
#include <filesystem>

class PiPeripheralNode : public rclcpp::Node {
public:
    PiPeripheralNode() : Node("pi_peripheral_node") {
        this->declare_parameter("voltage_threshold", 10.5);
        this->declare_parameter("led_path", "/sys/class/leds/ACT/");

        // Check if we are actually on a Pi
        std::string path = this->get_parameter("led_path").as_string();
        is_pi_ = std::filesystem::exists(path);

        if (is_pi_) {
            RCLCPP_INFO(get_logger(), "Raspberry Pi LED detected at %s", path.c_str());
        } else {
            RCLCPP_WARN(get_logger(), "Not on a Pi (or no LED access). Will log battery warnings to console only.");
        }

        battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "battery_status", 10, std::bind(&PiPeripheralNode::batteryCallback, this, std::placeholders::_1));
    }

private:
    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        double threshold = this->get_parameter("voltage_threshold").as_double();

        if (msg->voltage < threshold) {
            if (is_pi_) {
                setLedTrigger("timer"); // Hardware blink
            } else {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "LOW BATTERY WARNING: %fV", msg->voltage);
            }
        } else {
            if (is_pi_) setLedTrigger("none");
        }
    }

    void setLedTrigger(const std::string &trigger) {
        // Optimization: Only write if the trigger state actually changed
        if (current_trigger_ == trigger) return;

        std::string path = this->get_parameter("led_path").as_string() + "trigger";
        std::ofstream file(path);
        if (file.is_open()) {
            file << trigger;
            current_trigger_ = trigger;
            RCLCPP_INFO(get_logger(), "LED Trigger set to: %s", trigger.c_str());
        }
    }

    bool is_pi_;
    std::string current_trigger_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PiPeripheralNode>());
    rclcpp::shutdown();
    return 0;
}
