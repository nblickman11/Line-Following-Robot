#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageFilterNode : public rclcpp::Node
{
public:
  ImageFilterNode()
  : Node("image_filter_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw",
      10,
      std::bind(&ImageFilterNode::image_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/filtered/image_raw", 10);

    RCLCPP_INFO(this->get_logger(), "ImageFilterNode started b.");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      // Convert ROS Image to OpenCV Mat (BGR8)
      cv::Mat cv_image = cv_bridge::toCvCopy(msg, "bgr8")->image;

    // ROI: bottom slice of image
    int height = cv_image.rows;
      cv::Mat roi = cv_image(cv::Range(height - 50, height), cv::Range::all());

      // Convert to grayscale
      cv::Mat gray;
      cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);

double minVal, maxVal;
cv::minMaxLoc(gray, &minVal, &maxVal);
RCLCPP_INFO(this->get_logger(), "Grayscale intensity: min = %.1f, max = %.1f", minVal, maxVal);

    // Threshold to isolate white areas
    cv::Mat binary;
    cv::threshold(gray, binary, 140, 255, cv::THRESH_BINARY);

      // Convert back to ROS Image message (mono8)
      auto filtered_msg = cv_bridge::CvImage(msg->header, "mono8", binary).toImageMsg();

      // Publish
      publisher_->publish(*filtered_msg);
    }
    catch (const cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

