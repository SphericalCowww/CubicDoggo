#include <iostream>
#include <cstring>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
char getKey() {
    char buf = 0;
    struct termios old;
    std::memset(&old, 0, sizeof(struct termios));

    if (tcgetattr(0, &old) < 0) perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0) perror ("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) perror ("tcsetattr ~ICANON");
    return (buf);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class CubicDoggoTeleopKey : public rclcpp::Node {
public:
    CubicDoggoTeleopKey() : Node("cubic_doggo_teleop_key") {
        publisher_ = this->create_publisher<example_interfaces::msg::String>("/leg_set_named", 10);
        
        std::cout << "Reading from keyboard\n";
        std::cout << "---------------------------\n";
        std::cout << "Press 's' to STAND\n";
        std::cout << "Press 'q' to QUIT\n";
    }

    void run() {
        while (rclcpp::ok()) {
            char keyPressed = getKey();
            auto msg = example_interfaces::msg::String();

            if (keyPressed == 's') {
                msg.data = "stand";
                publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Sent 'stand' command");
            } 
            else if (keyPressed == 'q') {
                break;
            }
        }
    }

private:
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CubicDoggoTeleopKey>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
