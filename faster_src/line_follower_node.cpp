#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class LineFollower : public rclcpp::Node
{
public:
  LineFollower()
  : Node("line_follower")
  {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/filtered/image_raw", 10,
      std::bind(&LineFollower::image_callback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "LineFollower node started — using preprocessed binary image.");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  int frame_count_ = 0;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    frame_count_++;
    if (frame_count_ % 3 != 0 ){
    	return;
    }
    cv::Mat binary;
    try {
      binary = cv_bridge::toCvCopy(msg, "mono8")->image;
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    if (binary.empty()) return;

    int width = binary.cols;

    // Compute centroid using image moments
    cv::Moments M = cv::moments(binary, true);
    geometry_msgs::msg::Twist cmd;

    // If total number of white pixels is greater than 0, a line exists!
    if (M.m00 > 0) {
      // centroid x (sum of x coordinates / number of white pixels)
      int cx = static_cast<int>(M.m10 / M.m00);

      // error tells how far our centroid is from center of the image
      int error = cx - width / 2;

      cmd.linear.x = 0.15;
      cmd.angular.z = -0.000001 * error;

      RCLCPP_INFO(this->get_logger(), "width=%d, sum_of_x_coords=%.2f, total_white_pixels=%.2f, cx=%d, error=%d, angular=%.7f",width,M.m10,M.m00, cx, error, cmd.angular.z);
    } else {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      RCLCPP_WARN(this->get_logger(), "Line not found. Rotating...");
    }

    cmd_pub_->publish(cmd);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineFollower>());
  rclcpp::shutdown();
  return 0;
}
