#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gpiod.hpp>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

class MotorController : public rclcpp::Node
{
public:
    MotorController() : Node("motor_controller_node")
    {
        // Initialize GPIO chip (gpiochip0 is standard for Raspberry Pi)
        chip = gpiod::chip("gpiochip0");

        // Set BCM GPIO pin numbers used to control motor driver inputs
        in1_pin = 23;  // Motor A direction input 1
        in2_pin = 18;  // Motor A direction input 2
        in3_pin = 15;  // Motor B direction input 1
        in4_pin = 14;  // Motor B direction input 2

        // Get handles to GPIO lines
        in1 = chip.get_line(in1_pin);
        in2 = chip.get_line(in2_pin);
        in3 = chip.get_line(in3_pin);
        in4 = chip.get_line(in4_pin);

        // Request each line as output with initial value LOW (0)
        // The request() function takes:
        //   - a consumer name ("motor_ctrl" helps identify who owns the pin)
        //   - a flag that this is a direction OUTPUT
        //   - initial value (0 = LOW)
        // If this fails, it throws an exception (e.g., if already in use)
        in1.request({"motor_ctrl", gpiod::line_request::DIRECTION_OUTPUT, 0});
        in2.request({"motor_ctrl", gpiod::line_request::DIRECTION_OUTPUT, 0});
        in3.request({"motor_ctrl", gpiod::line_request::DIRECTION_OUTPUT, 0});
        in4.request({"motor_ctrl", gpiod::line_request::DIRECTION_OUTPUT, 0});

        // Subscribe to the /cmd_vel topic (standard ROS topic for velocity commands)
        // Queue size of 10; uses std::bind to attach the callback method
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&MotorController::cmdVelCallback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Motor controller node started. Listening to /cmd_vel...");
    }

private:
    gpiod::chip chip;
    gpiod::line in1, in2, in3, in4;
    int in1_pin, in2_pin, in3_pin, in4_pin;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

    // This callback is called every time a new message is received on /cmd_vel
    // Twist messages contain:
    //   - linear.x for forward/backward velocity
    //   - angular.z for rotation (turning left/right)
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
	// A minor change in value causes big robot change, so don't react so quick.
    	//rclcpp::Time now = this->get_clock()->now();
   	//if ((now - last_cmd_time_).seconds() < 0.2) return;
    	//last_cmd_time_ = now;

        double lin = msg->linear.x;     // forward/backward command
        double ang = msg->angular.z;    // turning command

	 // Threshold for detecting meaningful changes
	  const double linear_threshold = 0.01;
	  const double angular_threshold = 0.01;

//	  if (std::abs(lin - last_linear_x_) < linear_threshold &&
//	    std::abs(ang - last_angular_z_) < angular_threshold) {
	    // Command not changed significantly, skip processing
//	    return;
//	  }

	RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear.x=%.6f angular.z=%.7f", lin, ang);


    	// Prevent robot from spinning; Clamp angular velocity to safe range
    	ang = std::clamp(ang, -1.0, 1.0);

    // Deadzone threshold
    const double threshold = 0.0002; // make bigger so we turn less.  if error is low enough, angle is low, less than threshold, so goes straight.

    //if (std::abs(lin) < threshold && std::abs(ang) < threshold) {
      //  stopMotors();
       // return;
    // }

    if (lin > 0) {
        if (ang > threshold) {
            // Curve Left Forward
            curveLeftForward();
        } else if (ang < -threshold) {
            // Curve Right Forward
            curveRightForward();
        } else {
            driveForward();
        }
    } else if (lin < 0) {
        if (ang > threshold) {
            // Curve Left Backward
            curveRightBackward();
        } else if (ang < -threshold) {
            // Curve Right Backward
            curveLeftBackward();
        } else {
            driveBackward();
        }
    } else {
        if (ang > threshold) {
            turnLeft();
        } else if (ang < -threshold) {
            turnRight();
        } else {
	    
            stopMotors();
        }

    }
    // Only Pulse Motors
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    	stopMotors();
    }

    // These functions directly control motor GPIO lines

    void driveForward()
    {
        in1.set_value(1);  // Motor A: forward
        in2.set_value(0);
        in3.set_value(0);  // Motor B: forward
        in4.set_value(1);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Moving forward");
    }

    void driveBackward()
    {
        in1.set_value(0);  // Motor A: backward
        in2.set_value(1);
        in3.set_value(1);  // Motor B: backward
        in4.set_value(0);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Moving backward");
    }

    void turnLeft()
    {
        in1.set_value(0);  // Motor A: backward
        in2.set_value(1);
        in3.set_value(0);  // Motor B: forward
        in4.set_value(1);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Turning left");
    }

    void turnRight()
    {
        in1.set_value(1);  // Motor A: forward
        in2.set_value(0);
        in3.set_value(1);  // Motor B: backward
        in4.set_value(0);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Turning right");
    }

    void stopMotors()
    {
        in1.set_value(0);
        in2.set_value(0);
        in3.set_value(0);
        in4.set_value(0);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Stopping motors");
    }
	void curveLeftForward()
	{
	    in1.set_value(1);
	    in2.set_value(0);
	    in3.set_value(0);
	    in4.set_value(0); // slow/stop right motor to curve left
	    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Curving left forward");
	}

	void curveRightForward()
	{
	    in1.set_value(0);
	    in2.set_value(0); // slow/stop left motor to curve right
	    in3.set_value(0);
	    in4.set_value(1);
	    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Curving right forward");
	}

	void curveLeftBackward()
	{
	    in1.set_value(0);
	    in2.set_value(1);
	    in3.set_value(0); // slow/stop right motor to curve left backward
	    in4.set_value(0);
	    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Curving left backward");
	}

	void curveRightBackward()
	{
	    in1.set_value(0);
	    in2.set_value(0); // slow/stop left motor to curve right backward
	    in3.set_value(1);
	    in4.set_value(0);
	    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Curving right backward");
	}

	double last_linear_x_ = 0.0;
	double last_angular_z_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

