#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class ImageRotateNode : public rclcpp::Node
{
public:
  ImageRotateNode()
  : Node("image_rotate_node")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/picam/image_raw");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/picam/image_rotated");
    rotation_degrees_ = declare_parameter<int>("rotation_degrees", 90);

    pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic, 10);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic, 10, std::bind(&ImageRotateNode::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Rotating images from %s to %s, rotation=%d degrees",
      input_topic.c_str(), output_topic.c_str(), rotation_degrees_);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    const auto bytes_per_pixel = get_bytes_per_pixel(msg->encoding);
    if (bytes_per_pixel == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Unsupported image encoding '%s'. Supported: rgb8, bgr8, mono8.",
        msg->encoding.c_str());
      return;
    }

    if (msg->step < msg->width * bytes_per_pixel) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Invalid image step");
      return;
    }

    sensor_msgs::msg::Image rotated = *msg;
    rotate_image(*msg, rotated, bytes_per_pixel);
    pub_->publish(rotated);
  }

  static uint32_t get_bytes_per_pixel(const std::string & encoding)
  {
    if (encoding == "rgb8" || encoding == "bgr8") {
      return 3;
    }
    if (encoding == "mono8") {
      return 1;
    }
    return 0;
  }

  void rotate_image(
    const sensor_msgs::msg::Image & src,
    sensor_msgs::msg::Image & dst,
    const uint32_t bytes_per_pixel)
  {
    if (rotation_degrees_ == 0) {
      dst = src;
      return;
    }

    if (rotation_degrees_ == 180) {
      dst.height = src.height;
      dst.width = src.width;
    } else if (rotation_degrees_ == 90 || rotation_degrees_ == 270) {
      dst.height = src.width;
      dst.width = src.height;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Unsupported rotation_degrees=%d. Use 0, 90, 180, or 270.",
        rotation_degrees_);
      dst = src;
      return;
    }

    dst.step = dst.width * bytes_per_pixel;
    dst.data.assign(dst.step * dst.height, 0);

    for (uint32_t y = 0; y < src.height; ++y) {
      for (uint32_t x = 0; x < src.width; ++x) {
        uint32_t target_x = x;
        uint32_t target_y = y;

        if (rotation_degrees_ == 90) {
          target_x = src.height - 1 - y;
          target_y = x;
        } else if (rotation_degrees_ == 180) {
          target_x = src.width - 1 - x;
          target_y = src.height - 1 - y;
        } else if (rotation_degrees_ == 270) {
          target_x = y;
          target_y = src.width - 1 - x;
        }

        const auto src_offset = y * src.step + x * bytes_per_pixel;
        const auto dst_offset = target_y * dst.step + target_x * bytes_per_pixel;
        std::copy_n(src.data.begin() + src_offset, bytes_per_pixel, dst.data.begin() + dst_offset);
      }
    }
  }

  int rotation_degrees_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageRotateNode>());
  rclcpp::shutdown();
  return 0;
}
