#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&MinimalSubscriber::imageCallback, this, _1));

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("topic", 10);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr image_msg) {
      // Convert ROS 2 image to OpenCV image
      cv::Mat cv_image;
      try {
          cv_image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
      } catch (cv_bridge::Exception& e) {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
          return;
      }

      // Draw a circle
      cv::Point center(cv_image.cols/2, cv_image.rows/2);
      int radius = 50;
      cv::Scalar color(0, 255, 0); // Green circle
      int thickness = 2;

      cv::circle(cv_image, center, radius, color, thickness);

      // Convert OpenCV image back to ROS 2 image message
      auto output_image_msg = cv_bridge::CvImage(image_msg->header, "bgr8", cv_image).toImageMsg();

      // Publish the modified image
      publisher_->publish(*output_image_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}