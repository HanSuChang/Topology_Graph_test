#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

class ImageRotateCompressNode : public rclcpp::Node
{
public:
  ImageRotateCompressNode()
  : Node("image_rotate_compress_node")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/picam/image_raw");
    const auto output_topic =
      declare_parameter<std::string>("output_topic", "/picam/image_rotated/compressed");
    rotation_degrees_ = declare_parameter<int>("rotation_degrees", 90);
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 80);
    output_width_ = declare_parameter<int>("output_width", 0);
    output_height_ = declare_parameter<int>("output_height", 0);
    resize_mode_ = declare_parameter<std::string>("resize_mode", "none");

    jpeg_quality_ = std::max(1, std::min(jpeg_quality_, 100));

    pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(output_topic, 10);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic, 10,
      std::bind(&ImageRotateCompressNode::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Rotating and compressing %s to %s, rotation=%d degrees, jpeg_quality=%d, output=%dx%d, resize_mode=%s",
      input_topic.c_str(), output_topic.c_str(), rotation_degrees_, jpeg_quality_, output_width_,
      output_height_, resize_mode_.c_str());
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat frame;
    if (!image_msg_to_mat(*msg, frame)) {
      return;
    }

    rotate_frame(frame);
    resize_frame(frame);

    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    std::vector<uchar> encoded;
    if (!cv::imencode(".jpg", frame, encoded, params)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "JPEG encoding failed");
      return;
    }

    sensor_msgs::msg::CompressedImage out;
    out.header = msg->header;
    out.format = "jpeg";
    out.data.assign(encoded.begin(), encoded.end());
    pub_->publish(out);
  }

  bool image_msg_to_mat(const sensor_msgs::msg::Image & msg, cv::Mat & frame)
  {
    if (msg.encoding == "rgb8" || msg.encoding == "bgr8") {
      if (msg.step < msg.width * 3) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Invalid rgb/bgr image step");
        return false;
      }

      cv::Mat source(msg.height, msg.width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()), msg.step);
      if (msg.encoding == "rgb8") {
        cv::cvtColor(source, frame, cv::COLOR_RGB2BGR);
      } else {
        frame = source.clone();
      }
      return true;
    }

    if (msg.encoding == "mono8") {
      if (msg.step < msg.width) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Invalid mono image step");
        return false;
      }

      cv::Mat source(msg.height, msg.width, CV_8UC1, const_cast<uint8_t *>(msg.data.data()), msg.step);
      frame = source.clone();
      return true;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Unsupported image encoding '%s'. Supported: rgb8, bgr8, mono8.", msg.encoding.c_str());
    return false;
  }

  void rotate_frame(cv::Mat & frame)
  {
    cv::Mat rotated;
    switch (rotation_degrees_) {
      case 0:
        return;
      case 90:
        cv::rotate(frame, rotated, cv::ROTATE_90_CLOCKWISE);
        frame = rotated;
        return;
      case 180:
        cv::rotate(frame, rotated, cv::ROTATE_180);
        frame = rotated;
        return;
      case 270:
        cv::rotate(frame, rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
        frame = rotated;
        return;
      default:
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Unsupported rotation_degrees=%d. Use 0, 90, 180, or 270.", rotation_degrees_);
        return;
    }
  }

  void resize_frame(cv::Mat & frame)
  {
    if (output_width_ <= 0 || output_height_ <= 0 || resize_mode_ == "none") {
      return;
    }

    cv::Mat resized;
    if (resize_mode_ == "stretch") {
      cv::resize(frame, resized, cv::Size(output_width_, output_height_), 0.0, 0.0, cv::INTER_AREA);
      frame = resized;
      return;
    }

    if (resize_mode_ != "crop") {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Unsupported resize_mode='%s'. Use none, crop, or stretch.", resize_mode_.c_str());
      return;
    }

    const double source_ratio = static_cast<double>(frame.cols) / static_cast<double>(frame.rows);
    const double target_ratio = static_cast<double>(output_width_) / static_cast<double>(output_height_);
    cv::Rect roi;

    if (source_ratio > target_ratio) {
      const int crop_width = static_cast<int>(frame.rows * target_ratio);
      roi = cv::Rect((frame.cols - crop_width) / 2, 0, crop_width, frame.rows);
    } else {
      const int crop_height = static_cast<int>(frame.cols / target_ratio);
      roi = cv::Rect(0, (frame.rows - crop_height) / 2, frame.cols, crop_height);
    }

    cv::resize(frame(roi), resized, cv::Size(output_width_, output_height_), 0.0, 0.0, cv::INTER_AREA);
    frame = resized;
  }

  int rotation_degrees_;
  int jpeg_quality_;
  int output_width_;
  int output_height_;
  std::string resize_mode_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageRotateCompressNode>());
  rclcpp::shutdown();
  return 0;
}
