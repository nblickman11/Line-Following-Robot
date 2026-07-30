#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <opencv2/opencv.hpp>

class GstCameraNode : public rclcpp::Node {
public:
  GstCameraNode() : Node("gst_camera_node") {
    publisher_ = image_transport::create_publisher(this, "camera/image_raw");
    
    // Initialize GStreamer
    gst_init(nullptr, nullptr);

    // Create the GStreamer pipeline
    std::string pipeline_desc = "libcamerasrc ! video/x-raw,width=640,height=480,framerate=30/1,format=RGB ! appsink name=sink";

    GError *error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_desc.c_str(), &error);
    if (!pipeline_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create GStreamer pipeline: %s", error->message);
      return;
    }

    // Retrieve the appsink
    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
    gst_app_sink_set_emit_signals((GstAppSink *)appsink_, true);
    gst_app_sink_set_max_buffers((GstAppSink *)appsink_, 1);
    gst_app_sink_set_drop((GstAppSink *)appsink_, true);

    // Start the pipeline
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);

    // Start a timer to poll frames
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(80),  // Poll at approximately 12.5 Hz
       std::bind(&GstCameraNode::capture_frame, this)
    );
  }

  ~GstCameraNode() {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
  }

private:
  void capture_frame() {
    GstSample *sample = gst_app_sink_try_pull_sample((GstAppSink *)appsink_, 1000000);  // 1ms timeout

    if (!sample) {
      RCLCPP_WARN(this->get_logger(), "No sample received from appsink.");
      return;
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *s = gst_caps_get_structure(caps, 0);

    int width, height;
    gst_structure_get_int(s, "width", &width);
    gst_structure_get_int(s, "height", &height);

    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);

    // Create OpenCV Mat from raw data
    cv::Mat frame(height, width, CV_8UC3, (char *)map.data);

    // Convert to ROS Image
    auto msg = cv_bridge::CvImage(
      std_msgs::msg::Header(),
      "bgr8",
      frame
    ).toImageMsg();

    auto now = this->get_clock()->now();
    msg->header.stamp = now;
    msg->header.frame_id = "camera_frame";

    RCLCPP_INFO(this->get_logger(), "Publishing image: %dx%d", msg->width, msg->height);

    publisher_.publish(msg);
    
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
  }

  GstElement *pipeline_;
  GstElement *appsink_; 
  image_transport::Publisher publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GstCameraNode>());
  rclcpp::shutdown();
  return 0;
}
