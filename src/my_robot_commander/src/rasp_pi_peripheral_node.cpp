#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <filesystem>
#include <string>
#include <chrono>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class RapsPiPeripheralNode : public rclcpp::Node {
public:
    RapsPiPeripheralNode() : Node("pi_peripheral_node") {
        this->declare_parameter("power_path", "/sys/class/hwmon/hwmon3/in0_lcrit_alarm");
        this->declare_parameter("led_path",   "/sys/class/leds/ACT/");
        this->declare_parameter("delay_ms",   500);

        std::string power_path = this->get_parameter("power_path").as_string();
        std::string led_path   = this->get_parameter("led_path").as_string();
        is_rasp_pi_ = std::filesystem::exists(led_path);
        if (is_rasp_pi_) {
            RCLCPP_INFO(get_logger(), "RapsPiPeripheralNode:constructor(): raspberry pi "
                                      "power alarm peripheral detected at %s, led peripherial detected at %s", 
                                      power_path.c_str(), led_path.c_str());
        } else {
            RCLCPP_WARN(get_logger(), "RapsPiPeripheralNode:constructor(): not on a raspberry pi (or no LED access)");
        }
        
        timer_ = this->create_wall_timer(std::chrono::seconds(1), 
                                         std::bind(&RapsPiPeripheralNode::checkHardwarePower, this));
    }
    bool is_valid() const { 
        return is_rasp_pi_; 
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
    void checkHardwarePower() {
        if (!is_rasp_pi_) return;

        std::string power_path = this->get_parameter("power_path").as_string();
        std::ifstream power_file(power_path);
        if (power_file.is_open()) {
            int power_alarm;
            power_file >> power_alarm;

            if (power_alarm == 1) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(),  5000, "RapsPiPeripheralNode:checkHardwarePower(): "
                                      "HARDWARE LOW POWER, TIME TO CHARGE BATTERY");
                setLedTrigger("timer");
            } else {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 60000, "RapsPiPeripheralNode:checkHardwarePower(): "
                                      "hardware power normal");
                setLedTrigger("none");
            }
        }
    }
    void setLedTrigger(const std::string &trigger) {
        if (current_trigger_ == trigger) return;

        std::string led_path = this->get_parameter("led_path").as_string();
        int delay_ms = this->get_parameter("delay_ms").as_int();
        
        std::ofstream led_trigger_file(led_path+"/trigger");
        if (led_trigger_file.is_open()) {
            led_trigger_file << trigger;
            current_trigger_ = trigger;
            RCLCPP_INFO(get_logger(), "RapsPiPeripheralNode:setLedTrigger(): "
                                      "LED trigger set to: %s", trigger.c_str());
        }
        if (trigger == "timer") {
            std::ofstream led_on_file( led_path+"/delay_on");
            std::ofstream led_off_file(led_path+"/delay_off");
            if (led_on_file.is_open() && led_off_file.is_open()) {
                led_on_file << delay_ms;
                led_off_file << delay_ms;
            }
        }
    }

    bool                         is_rasp_pi_;
    std::string                  current_trigger_;
    rclcpp::TimerBase::SharedPtr timer_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RapsPiPeripheralNode>();
    if (node->is_valid()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
