#include <rclcpp/rclcpp.hpp>
#include <gpiod.hpp>   // libgpiod C++ binding
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class MotorDriverNode : public rclcpp::Node
{
public:
    MotorDriverNode() : Node("motor_driver_node")
    {
        // GPIO chip, usually "gpiochip0" for Raspberry Pi
        chip = gpiod::chip("gpiochip0");

        // Your GPIO pin numbers (BCM numbering)
        in1_pin = 23;  // Physical 16
        in2_pin = 18;  // Physical 12
        in3_pin = 15;  // Physical 10
        in4_pin = 14;  // Physical 8

        // Request lines as outputs, default LOW
        in1 = chip.get_line(in1_pin);
        in2 = chip.get_line(in2_pin);
        in3 = chip.get_line(in3_pin);
        in4 = chip.get_line(in4_pin);

        in1.request({"motor_driver", gpiod::line_request::DIRECTION_OUTPUT, 0});
        in2.request({"motor_driver", gpiod::line_request::DIRECTION_OUTPUT, 0});
        in3.request({"motor_driver", gpiod::line_request::DIRECTION_OUTPUT, 0});
        in4.request({"motor_driver", gpiod::line_request::DIRECTION_OUTPUT, 0});

        RCLCPP_INFO(this->get_logger(), "driving forward...");
        driveForward();
        std::this_thread::sleep_for(2s);
        stopMotors();
        RCLCPP_INFO(this->get_logger(), "Motors stopped.");

	RCLCPP_INFO(this->get_logger(), "Now, here's the full set of tests.");
	testDirections();
    }

private:
    gpiod::chip chip;
    gpiod::line in1, in2, in3, in4;
    int in1_pin, in2_pin, in3_pin, in4_pin;

    void driveForward()
    {
        in1.set_value(1);
        in2.set_value(0);
        in3.set_value(0);
        in4.set_value(1);
    }
    void driveBackward()
    {
        in1.set_value(0);
        in2.set_value(1);
        in3.set_value(1);
        in4.set_value(0);
    }
    void turnLeft()
    {
        in1.set_value(0);
        in2.set_value(1);
        in3.set_value(0);
        in4.set_value(1);
    }
    void turnRight()
    {
        in1.set_value(1);
        in2.set_value(0);
        in3.set_value(1);
        in4.set_value(0);
    }
    void stopMotors()
    {
        in1.set_value(0);
        in2.set_value(0);
        in3.set_value(0);
        in4.set_value(0);
    }
    void testDirections(){
    	// Go Forward
	driveForward();
	std::this_thread::sleep_for(2s);
	stopMotors();
	std::this_thread::sleep_for(1s);

	// Go Backward
	driveBackward();
	std::this_thread::sleep_for(2s);
	stopMotors();
	std::this_thread::sleep_for(1s);
	
	// Go Left
	turnLeft();
        std::this_thread::sleep_for(1s);
        stopMotors();
        std::this_thread::sleep_for(500ms);
        driveForward();
        std::this_thread::sleep_for(1s);
        stopMotors();
        std::this_thread::sleep_for(1s);

	// Go Right
        turnRight();
        std::this_thread::sleep_for(1s);
        stopMotors();
        std::this_thread::sleep_for(500ms);
        driveForward();
        std::this_thread::sleep_for(1s);
        stopMotors();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

