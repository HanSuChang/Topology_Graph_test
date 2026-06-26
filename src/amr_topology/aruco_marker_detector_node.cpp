#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

class ArucoMarkerDetectorNode : public rclcpp::Node
{
public:
  ArucoMarkerDetectorNode()
  : Node("aruco_marker_detector_node")
  {
    video_device_ = declare_parameter<std::string>(
      "video_device", "/dev/v4l/by-id/usb-BY-5640-200408_Webcam_01.00.00-video-index0");
    output_topic_ = declare_parameter<std::string>("output_topic", "/aruco_marker/target");
    enable_topic_ = declare_parameter<std::string>("enable_topic", "/aruco_marker/enable");
    dictionary_name_ = declare_parameter<std::string>("dictionary", "4X4");
    marker_id_ = declare_parameter<int>("marker_id", 175);
    marker_size_m_ = declare_parameter<double>("marker_size_m", 0.095);
    focal_length_px_ = declare_parameter<double>("focal_length_px", 620.0);
    camera_width_ = declare_parameter<int>("camera_width", 640);
    camera_height_ = declare_parameter<int>("camera_height", 480);
    camera_fps_ = declare_parameter<double>("camera_fps", 10.0);
    pixel_format_ = declare_parameter<std::string>("pixel_format", "YUYV");

    dictionaries_ = make_dictionaries(dictionary_name_);

    marker_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(output_topic_, 10);
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic_, 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        set_detection_enabled(msg->data);
      });

    RCLCPP_INFO(
      get_logger(),
      "ArUco detector ready: device=%s dictionary=%s marker_id=%d marker_size=%.3fm",
      video_device_.c_str(), dictionary_name_.c_str(), marker_id_, marker_size_m_);
  }

  ~ArucoMarkerDetectorNode() override
  {
    close_camera();
  }

private:
  void set_detection_enabled(bool enabled)
  {
    if (enabled == detection_enabled_) {
      return;
    }

    detection_enabled_ = enabled;
    if (detection_enabled_) {
      open_camera();
      if (camera_.isOpened()) {
        const auto period = std::chrono::duration<double>(1.0 / std::max(camera_fps_, 1.0));
        timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::milliseconds>(period),
          std::bind(&ArucoMarkerDetectorNode::detect_once, this));
        RCLCPP_INFO(get_logger(), "ArUco camera enabled");
      } else {
        detection_enabled_ = false;
      }
    } else {
      timer_.reset();
      close_camera();
      RCLCPP_INFO(get_logger(), "ArUco camera disabled");
    }
  }

  void open_camera()
  {
    close_camera();
    camera_.open(video_device_, cv::CAP_V4L2);
    if (!camera_.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Failed to open ArUco camera: %s", video_device_.c_str());
      return;
    }

    camera_.set(cv::CAP_PROP_FRAME_WIDTH, camera_width_);
    camera_.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height_);
    camera_.set(cv::CAP_PROP_FPS, camera_fps_);
    if (pixel_format_ == "MJPG") {
      camera_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    } else if (pixel_format_ == "YUYV") {
      camera_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    }
  }

  void close_camera()
  {
    if (camera_.isOpened()) {
      camera_.release();
    }
  }

  static std::vector<cv::Ptr<cv::aruco::Dictionary>> make_dictionaries(const std::string & name)
  {
    if (name == "4X4" || name == "4X4_ALL") {
      return {
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50),
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100),
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250),
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000),
      };
    }
    if (name == "4X4_50") {
      return {cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50)};
    }
    if (name == "4X4_100") {
      return {cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100)};
    }
    if (name == "4X4_250") {
      return {cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250)};
    }
    if (name == "4X4_1000") {
      return {cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000)};
    }
    return {cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250)};
  }

  void detect_once()
  {
    if (!detection_enabled_ || !camera_.isOpened()) {
      return;
    }

    cv::Mat frame;
    if (!camera_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to read ArUco camera frame");
      return;
    }

    for (const auto & dictionary : dictionaries_) {
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(frame, dictionary, corners, ids);

      for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] != marker_id_) {
          continue;
        }

        const auto & marker_corners = corners[i];
        const cv::Point2f center =
          (marker_corners[0] + marker_corners[1] + marker_corners[2] + marker_corners[3]) * 0.25f;
        const double side_px =
          (cv::norm(marker_corners[0] - marker_corners[1]) +
          cv::norm(marker_corners[1] - marker_corners[2]) +
          cv::norm(marker_corners[2] - marker_corners[3]) +
          cv::norm(marker_corners[3] - marker_corners[0])) * 0.25;
        if (side_px <= 1.0) {
          return;
        }

        geometry_msgs::msg::PointStamped target;
        target.header.stamp = this->now();
        target.header.frame_id = "aruco_camera";
        target.point.x = (static_cast<double>(center.x) - static_cast<double>(frame.cols) * 0.5) /
          (static_cast<double>(frame.cols) * 0.5);
        target.point.y = marker_size_m_ * focal_length_px_ / side_px;
        target.point.z = static_cast<double>(marker_id_);
        marker_pub_->publish(target);
        return;
      }
    }
  }

  bool detection_enabled_{false};
  int marker_id_{175};
  int camera_width_{640};
  int camera_height_{480};
  double marker_size_m_{0.095};
  double focal_length_px_{620.0};
  double camera_fps_{10.0};
  std::string video_device_;
  std::string output_topic_;
  std::string enable_topic_;
  std::string dictionary_name_;
  std::string pixel_format_;
  std::vector<cv::Ptr<cv::aruco::Dictionary>> dictionaries_;
  cv::VideoCapture camera_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoMarkerDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
