#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <memory>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unordered_map>
#include <unistd.h>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct LeaderPoseSample
{
  Pose2D pose;
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

struct PwmCommand
{
  int left{0};
  int right{0};
};

struct PathTarget
{
  Pose2D pose;
  bool valid{false};
};

struct PathProjection
{
  double progress{0.0};
  size_t segment_index{0};
  double segment_ratio{0.0};
  bool valid{false};
};

double normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw * 0.5);
  q.z = std::sin(yaw * 0.5);
  return q;
}

speed_t baud_rate_to_termios(int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    default:
      return B115200;
  }
}

class SerialPort
{
public:
  ~SerialPort()
  {
    close_port();
  }

  bool open_port(const std::string & device, int baud_rate)
  {
    close_port();

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      last_error_ = std::strerror(errno);
      return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      last_error_ = std::strerror(errno);
      close_port();
      return false;
    }

    cfmakeraw(&tty);
    const speed_t speed = baud_rate_to_termios(baud_rate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      last_error_ = std::strerror(errno);
      close_port();
      return false;
    }

    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  void close_port()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool is_open() const
  {
    return fd_ >= 0;
  }

  bool write_line(const std::string & line)
  {
    if (fd_ < 0) {
      last_error_ = "serial port is not open";
      return false;
    }

    const std::string payload = line + "\n";
    const ssize_t written = ::write(fd_, payload.data(), payload.size());
    if (written < 0 || static_cast<size_t>(written) != payload.size()) {
      last_error_ = written < 0 ? std::strerror(errno) : "partial serial write";
      close_port();
      return false;
    }
    return true;
  }

  std::string read_available()
  {
    if (fd_ < 0) {
      return {};
    }

    std::string data;
    char buffer[256];
    while (true) {
      const ssize_t count = ::read(fd_, buffer, sizeof(buffer));
      if (count > 0) {
        data.append(buffer, static_cast<size_t>(count));
        continue;
      }
      if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        break;
      }
      if (count < 0) {
        last_error_ = std::strerror(errno);
        close_port();
      }
      break;
    }
    return data;
  }

  const std::string & last_error() const
  {
    return last_error_;
  }

private:
  int fd_{-1};
  std::string last_error_;
};

enum class FollowerMode
{
  Stop,
  Follow,
  Return,
  Align,
  TurnPrepareLeft,
  SlotWait
};

enum class SlotWaitTask
{
  None,
  A,
  B
};

enum class SlotWaitStage
{
  DriveToIntersection,
  AlignToStaging,
  DriveToStaging,
  Arrived
};

}  // namespace

class RcCarFollowerNode : public rclcpp::Node
{
public:
  RcCarFollowerNode()
  : Node("rc_car_follower_node")
  {
    const auto default_topology =
      ament_index_cpp::get_package_share_directory("amr_topology") + "/config/topology.yaml";

    this->declare_parameter<std::string>("leader_pose_topic", "/turtlebot/pose");
    this->declare_parameter<std::string>("leader_pose_type", "pose_stamped");
    this->declare_parameter<std::string>("leader_cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<std::string>("follower_odom_topic", "/rc_car/odom");
    this->declare_parameter<std::string>("mode_topic", "/rc_car/follower_mode");
    this->declare_parameter<std::string>("command_topic", "/rc_car/command");
    this->declare_parameter<std::string>("encoder_ticks_topic", "/rc_car/encoder_ticks");
    this->declare_parameter<std::string>("serial_debug_topic", "/rc_car/serial_debug");
    this->declare_parameter<std::string>("slot_wait_status_topic", "/rc_car/slot_wait_status");
    this->declare_parameter<std::string>("arduino_port", "auto");
    this->declare_parameter<int>("arduino_baud_rate", 115200);
    this->declare_parameter<double>("target_distance", 0.70);
    this->declare_parameter<double>("distance_deadband", 0.08);
    this->declare_parameter<double>("too_close_distance", 0.45);
    this->declare_parameter<double>("emergency_stop_distance", 0.25);
    this->declare_parameter<double>("collision_stop_distance", 0.08);
    this->declare_parameter<double>("heading_deadband", 0.18);
    this->declare_parameter<double>("pose_timeout_seconds", 0.70);
    this->declare_parameter<double>("control_rate_hz", 10.0);
    this->declare_parameter<double>("drive_duty_cycle", 1.0);
    this->declare_parameter<double>("drive_cycle_seconds", 0.20);
    this->declare_parameter<double>("follow_start_delay_seconds", 2.0);
    this->declare_parameter<bool>("start_in_follow_mode", false);
    this->declare_parameter<bool>("publish_encoder_odom", true);
    this->declare_parameter<bool>("auto_initialize_odom_from_leader", true);
    this->declare_parameter<double>("initial_rc_x", 0.0);
    this->declare_parameter<double>("initial_rc_y", 0.0);
    this->declare_parameter<double>("initial_rc_yaw", 0.0);
    this->declare_parameter<std::string>("odom_frame_id", "map");
    this->declare_parameter<std::string>("base_frame_id", "rc_car_base_link");
    this->declare_parameter<double>("initial_follow_distance", 0.70);
    this->declare_parameter<double>("wheel_base", 0.16);
    this->declare_parameter<double>("left_meters_per_tick", 0.0005882353);
    this->declare_parameter<double>("right_meters_per_tick", 0.0005882353);
    this->declare_parameter<bool>("enable_command_odom_fallback", true);
    this->declare_parameter<double>("encoder_timeout_seconds", 0.50);
    this->declare_parameter<int>("encoder_baseline_samples", 3);
    this->declare_parameter<int>("max_encoder_tick_delta_per_update", 1000);
    this->declare_parameter<double>("max_odom_translation_delta_per_update", 0.50);
    this->declare_parameter<double>("max_odom_yaw_delta_per_update", 1.20);
    this->declare_parameter<double>("odom_jump_command_hold_seconds", 2.0);
    this->declare_parameter<double>("max_pwm_wheel_speed", 0.24);
    this->declare_parameter<double>("odom_publish_rate_hz", 20.0);
    this->declare_parameter<double>("leader_turn_assist_heading", 0.35);
    this->declare_parameter<double>("command_resend_period_seconds", 0.25);
    this->declare_parameter<double>("leader_history_max_seconds", 60.0);
    this->declare_parameter<double>("leader_history_min_spacing", 0.01);
    this->declare_parameter<double>("leader_path_follow_distance", 0.70);
    this->declare_parameter<double>("leader_path_goal_tolerance", 0.18);
    this->declare_parameter<bool>("use_topology_path", false);
    this->declare_parameter<std::string>("topology_file", default_topology);
    this->declare_parameter<std::vector<std::string>>(
      "topology_route", {"loading", "intersection_1", "a_entry", "a_leader_slot"});
    this->declare_parameter<double>("topology_path_spacing", 0.05);
    this->declare_parameter<double>("topology_follow_gap_distance", 0.70);
    this->declare_parameter<double>("topology_min_gap_distance", 0.35);
    this->declare_parameter<double>("topology_lookahead_distance", 0.40);
    this->declare_parameter<int>("min_forward_pwm", 85);
    this->declare_parameter<int>("slow_forward_pwm", 65);
    this->declare_parameter<int>("min_turn_forward_pwm", 70);
    this->declare_parameter<int>("base_forward_pwm", 115);
    this->declare_parameter<int>("max_forward_pwm", 190);
    this->declare_parameter<int>("reverse_pwm", 90);
    this->declare_parameter<int>("turn_pwm_delta", 45);
    this->declare_parameter<int>("near_turn_pwm_delta", 15);
    this->declare_parameter<int>("straight_pwm_trim", 0);
    this->declare_parameter<bool>("enable_encoder_balance", false);
    this->declare_parameter<double>("encoder_balance_gain_pwm_per_meter", 400.0);
    this->declare_parameter<int>("encoder_balance_max_trim", 30);
    this->declare_parameter<int>("encoder_balance_update_pwm_tolerance", 12);
    this->declare_parameter<double>("encoder_balance_decay", 0.98);
    this->declare_parameter<int>("in_place_turn_pwm", 105);
    this->declare_parameter<double>("leader_speed_gain_pwm", 260.0);
    this->declare_parameter<double>("leader_moving_speed_threshold", 0.03);
    this->declare_parameter<double>("leader_cmd_vel_timeout_seconds", 0.50);
    this->declare_parameter<double>("leader_cmd_vel_speed_scale", 1.0);
    this->declare_parameter<double>("leader_cmd_vel_weight", 0.75);
    this->declare_parameter<double>("distance_gain_pwm", 55.0);
    this->declare_parameter<double>("turn_slowdown_heading", 0.75);
    this->declare_parameter<double>("turn_prepare_duty_cycle", 0.45);
    this->declare_parameter<double>("turn_prepare_close_distance", 0.62);
    this->declare_parameter<double>("turn_prepare_turn_delay_seconds", 2.0);
    this->declare_parameter<double>("intersection_1_x", 2.442805051803589);
    this->declare_parameter<double>("intersection_1_y", -0.015692830085754395);
    this->declare_parameter<double>("intersection_1_turn_radius", 0.35);
    this->declare_parameter<double>("intersection_1_turn_seconds", 1.20);
    this->declare_parameter<std::string>("intersection_1_left_turn_command", "3");
    this->declare_parameter<double>("slot_staging_ratio", 0.75);
    this->declare_parameter<bool>("slot_a_staging_override", true);
    this->declare_parameter<double>("slot_a_staging_x", 0.8180313110351562);
    this->declare_parameter<double>("slot_a_staging_y", 0.2924773097038269);
    this->declare_parameter<double>("slot_intersection_tolerance", 0.25);
    this->declare_parameter<double>("slot_staging_tolerance", 0.25);
    this->declare_parameter<double>("slot_yaw_tolerance", 0.16);
    this->declare_parameter<double>("slot_drive_heading_limit", 0.35);
    this->declare_parameter<int>("slot_forward_pwm", 80);
    this->declare_parameter<int>("slot_turn_pwm", 95);

    target_distance_ = this->get_parameter("target_distance").as_double();
    distance_deadband_ = this->get_parameter("distance_deadband").as_double();
    too_close_distance_ = this->get_parameter("too_close_distance").as_double();
    emergency_stop_distance_ = std::max(
      0.0, this->get_parameter("emergency_stop_distance").as_double());
    collision_stop_distance_ = std::max(
      0.0, this->get_parameter("collision_stop_distance").as_double());
    heading_deadband_ = this->get_parameter("heading_deadband").as_double();
    pose_timeout_seconds_ = this->get_parameter("pose_timeout_seconds").as_double();
    drive_duty_cycle_ = std::clamp(
      this->get_parameter("drive_duty_cycle").as_double(), 0.10, 1.0);
    drive_cycle_seconds_ = std::max(
      0.50, this->get_parameter("drive_cycle_seconds").as_double());
    follow_start_delay_seconds_ = std::max(
      0.0, this->get_parameter("follow_start_delay_seconds").as_double());
    publish_encoder_odom_ = this->get_parameter("publish_encoder_odom").as_bool();
    auto_initialize_odom_from_leader_ =
      this->get_parameter("auto_initialize_odom_from_leader").as_bool();
    initial_rc_x_ = this->get_parameter("initial_rc_x").as_double();
    initial_rc_y_ = this->get_parameter("initial_rc_y").as_double();
    initial_rc_yaw_ = this->get_parameter("initial_rc_yaw").as_double();
    odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    initial_follow_distance_ = this->get_parameter("initial_follow_distance").as_double();
    wheel_base_ = std::max(0.01, this->get_parameter("wheel_base").as_double());
    left_meters_per_tick_ = this->get_parameter("left_meters_per_tick").as_double();
    right_meters_per_tick_ = this->get_parameter("right_meters_per_tick").as_double();
    enable_command_odom_fallback_ =
      this->get_parameter("enable_command_odom_fallback").as_bool();
    encoder_timeout_seconds_ = std::max(
      0.0, this->get_parameter("encoder_timeout_seconds").as_double());
    encoder_baseline_samples_ = std::clamp(
      static_cast<int>(this->get_parameter("encoder_baseline_samples").as_int()), 1, 20);
    max_encoder_tick_delta_per_update_ = std::max(
      1, static_cast<int>(this->get_parameter("max_encoder_tick_delta_per_update").as_int()));
    max_odom_translation_delta_per_update_ = std::max(
      0.01, this->get_parameter("max_odom_translation_delta_per_update").as_double());
    max_odom_yaw_delta_per_update_ = std::max(
      0.01, this->get_parameter("max_odom_yaw_delta_per_update").as_double());
    odom_jump_command_hold_seconds_ = std::max(
      0.0, this->get_parameter("odom_jump_command_hold_seconds").as_double());
    max_pwm_wheel_speed_ = std::max(
      0.01, this->get_parameter("max_pwm_wheel_speed").as_double());
    leader_turn_assist_heading_ = this->get_parameter("leader_turn_assist_heading").as_double();
    command_resend_period_seconds_ = std::max(
      0.0, this->get_parameter("command_resend_period_seconds").as_double());
    leader_history_max_seconds_ = std::max(
      1.0, this->get_parameter("leader_history_max_seconds").as_double());
    leader_history_min_spacing_ = std::max(
      0.0, this->get_parameter("leader_history_min_spacing").as_double());
    leader_path_follow_distance_ = std::max(
      0.0, this->get_parameter("leader_path_follow_distance").as_double());
    leader_path_goal_tolerance_ = std::max(
      0.03, this->get_parameter("leader_path_goal_tolerance").as_double());
    use_topology_path_ = this->get_parameter("use_topology_path").as_bool();
    topology_file_ = this->get_parameter("topology_file").as_string();
    topology_route_ = this->get_parameter("topology_route").as_string_array();
    topology_path_spacing_ = std::max(
      0.01, this->get_parameter("topology_path_spacing").as_double());
    topology_follow_gap_distance_ = std::max(
      0.0, this->get_parameter("topology_follow_gap_distance").as_double());
    topology_min_gap_distance_ = std::max(
      0.0, this->get_parameter("topology_min_gap_distance").as_double());
    topology_lookahead_distance_ = std::max(
      0.05, this->get_parameter("topology_lookahead_distance").as_double());
    min_forward_pwm_ = std::clamp(
      static_cast<int>(this->get_parameter("min_forward_pwm").as_int()), 0, 255);
    slow_forward_pwm_ = std::clamp(
      static_cast<int>(this->get_parameter("slow_forward_pwm").as_int()), 0, 255);
    min_turn_forward_pwm_ = std::clamp(
      static_cast<int>(this->get_parameter("min_turn_forward_pwm").as_int()), 0, 255);
    base_forward_pwm_ = std::clamp(
      static_cast<int>(this->get_parameter("base_forward_pwm").as_int()), 0, 255);
    max_forward_pwm_ = std::clamp(
      static_cast<int>(this->get_parameter("max_forward_pwm").as_int()), 0, 255);
    reverse_pwm_ = std::clamp(
      static_cast<int>(this->get_parameter("reverse_pwm").as_int()), 0, 255);
    turn_pwm_delta_ = std::clamp(
      static_cast<int>(this->get_parameter("turn_pwm_delta").as_int()), 0, 255);
    near_turn_pwm_delta_ = std::clamp(
      static_cast<int>(this->get_parameter("near_turn_pwm_delta").as_int()), 0, 255);
    straight_pwm_trim_ = std::clamp(
      static_cast<int>(this->get_parameter("straight_pwm_trim").as_int()), -100, 100);
    enable_encoder_balance_ = this->get_parameter("enable_encoder_balance").as_bool();
    encoder_balance_gain_pwm_per_meter_ =
      this->get_parameter("encoder_balance_gain_pwm_per_meter").as_double();
    encoder_balance_max_trim_ = std::clamp(
      static_cast<int>(this->get_parameter("encoder_balance_max_trim").as_int()), 0, 100);
    encoder_balance_update_pwm_tolerance_ = std::clamp(
      static_cast<int>(this->get_parameter("encoder_balance_update_pwm_tolerance").as_int()), 0, 255);
    encoder_balance_decay_ = std::clamp(
      this->get_parameter("encoder_balance_decay").as_double(), 0.0, 1.0);
    in_place_turn_pwm_ = std::clamp(
      static_cast<int>(this->get_parameter("in_place_turn_pwm").as_int()), 0, 255);
    leader_speed_gain_pwm_ = this->get_parameter("leader_speed_gain_pwm").as_double();
    leader_moving_speed_threshold_ = std::max(
      0.0, this->get_parameter("leader_moving_speed_threshold").as_double());
    leader_cmd_vel_timeout_seconds_ = std::max(
      0.0, this->get_parameter("leader_cmd_vel_timeout_seconds").as_double());
    leader_cmd_vel_speed_scale_ = std::max(
      0.0, this->get_parameter("leader_cmd_vel_speed_scale").as_double());
    leader_cmd_vel_weight_ = std::clamp(
      this->get_parameter("leader_cmd_vel_weight").as_double(), 0.0, 1.0);
    distance_gain_pwm_ = this->get_parameter("distance_gain_pwm").as_double();
    turn_slowdown_heading_ = std::max(
      heading_deadband_, this->get_parameter("turn_slowdown_heading").as_double());
    turn_prepare_duty_cycle_ = std::clamp(
      this->get_parameter("turn_prepare_duty_cycle").as_double(), 0.10, 1.0);
    turn_prepare_close_distance_ =
      this->get_parameter("turn_prepare_close_distance").as_double();
    turn_prepare_turn_delay_seconds_ = std::max(
      0.0, this->get_parameter("turn_prepare_turn_delay_seconds").as_double());
    intersection_1_x_ = this->get_parameter("intersection_1_x").as_double();
    intersection_1_y_ = this->get_parameter("intersection_1_y").as_double();
    intersection_1_turn_radius_ =
      this->get_parameter("intersection_1_turn_radius").as_double();
    intersection_1_turn_seconds_ =
      this->get_parameter("intersection_1_turn_seconds").as_double();
    intersection_1_left_turn_command_ =
      this->get_parameter("intersection_1_left_turn_command").as_string();
    slot_staging_ratio_ = std::clamp(
      this->get_parameter("slot_staging_ratio").as_double(), 0.0, 1.0);
    slot_a_staging_override_ = this->get_parameter("slot_a_staging_override").as_bool();
    slot_a_staging_x_ = this->get_parameter("slot_a_staging_x").as_double();
    slot_a_staging_y_ = this->get_parameter("slot_a_staging_y").as_double();
    slot_intersection_tolerance_ = std::max(
      0.05, this->get_parameter("slot_intersection_tolerance").as_double());
    slot_staging_tolerance_ = std::max(
      0.05, this->get_parameter("slot_staging_tolerance").as_double());
    slot_yaw_tolerance_ = std::max(
      0.02, this->get_parameter("slot_yaw_tolerance").as_double());
    slot_drive_heading_limit_ = std::max(
      slot_yaw_tolerance_, this->get_parameter("slot_drive_heading_limit").as_double());
    slot_forward_pwm_ = std::clamp(
      static_cast<int>(this->get_parameter("slot_forward_pwm").as_int()), 0, 255);
    slot_turn_pwm_ = std::clamp(
      static_cast<int>(this->get_parameter("slot_turn_pwm").as_int()), 0, 255);

    load_topology_nodes();
    if (use_topology_path_) {
      load_topology_path();
    }
    mode_ = this->get_parameter("start_in_follow_mode").as_bool() ?
      FollowerMode::Follow : FollowerMode::Stop;

    const auto leader_pose_topic = this->get_parameter("leader_pose_topic").as_string();
    leader_pose_type_ = this->get_parameter("leader_pose_type").as_string();
    const auto leader_cmd_vel_topic = this->get_parameter("leader_cmd_vel_topic").as_string();
    const auto follower_odom_topic = this->get_parameter("follower_odom_topic").as_string();
    const auto mode_topic = this->get_parameter("mode_topic").as_string();
    const auto command_topic = this->get_parameter("command_topic").as_string();
    const auto encoder_ticks_topic = this->get_parameter("encoder_ticks_topic").as_string();
    const auto serial_debug_topic = this->get_parameter("serial_debug_topic").as_string();
    const auto slot_wait_status_topic = this->get_parameter("slot_wait_status_topic").as_string();
    arduino_port_ = this->get_parameter("arduino_port").as_string();
    arduino_baud_rate_ = this->get_parameter("arduino_baud_rate").as_int();

    if (leader_pose_type_ == "pose_with_covariance_stamped") {
      const auto amcl_qos = rclcpp::QoS(10).reliable().transient_local();
      leader_pose_cov_sub_ =
        this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        leader_pose_topic,
        amcl_qos,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          Pose2D pose;
          pose.x = msg->pose.pose.position.x;
          pose.y = msg->pose.pose.position.y;
          pose.yaw = yaw_from_quaternion(msg->pose.pose.orientation);
          update_leader_pose(pose, this->now());
        });
    } else {
      leader_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        leader_pose_topic,
        10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          Pose2D pose;
          pose.x = msg->pose.position.x;
          pose.y = msg->pose.position.y;
          pose.yaw = yaw_from_quaternion(msg->pose.orientation);
          update_leader_pose(pose, this->now());
        });
    }

    leader_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      leader_cmd_vel_topic,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        leader_cmd_speed_ = std::clamp(
          msg->linear.x * leader_cmd_vel_speed_scale_, 0.0, 0.5);
        last_leader_cmd_vel_stamp_ = this->now();
      });

    if (!publish_encoder_odom_) {
      follower_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        follower_odom_topic,
        10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          Pose2D pose;
          pose.x = msg->pose.pose.position.x;
          pose.y = msg->pose.pose.position.y;
          pose.yaw = yaw_from_quaternion(msg->pose.pose.orientation);
          const rclcpp::Time now = this->now();
          if (reject_odom_jump_if_needed(pose, now, "external /rc_car/odom")) {
            return;
          }
          follower_pose_ = pose;
          last_follower_stamp_ = now;
        });
    }

    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      mode_topic,
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        set_mode(msg->data);
      });

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(follower_odom_topic, 10);
    command_pub_ = this->create_publisher<std_msgs::msg::String>(command_topic, 10);
    slot_wait_status_pub_ =
      this->create_publisher<std_msgs::msg::String>(
      slot_wait_status_topic,
      rclcpp::QoS(1).reliable().transient_local());
    encoder_ticks_pub_ =
      this->create_publisher<std_msgs::msg::Int64MultiArray>(encoder_ticks_topic, 10);
    serial_debug_pub_ = this->create_publisher<std_msgs::msg::String>(serial_debug_topic, 10);

    try_open_serial_port();

    const double control_rate_hz = std::max(1.0, this->get_parameter("control_rate_hz").as_double());
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / control_rate_hz)),
      [this]() {
        control_step();
      });

    const double odom_rate_hz = std::max(
      1.0,
      this->get_parameter("odom_publish_rate_hz").as_double());
    odom_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / odom_rate_hz)),
      [this]() {
        serial_odom_step();
      });

    serial_retry_timer_ = this->create_wall_timer(
      1s,
      [this]() {
        if (!serial_.is_open()) {
          try_open_serial_port();
        }
      });
  }

  ~RcCarFollowerNode() override
  {
    send_pwm_command(PwmCommand{0, 0});
  }

private:
  void try_open_serial_port()
  {
    const std::string port = resolve_serial_port();
    if (!serial_.open_port(port, arduino_baud_rate_)) {
      const std::string candidates = describe_serial_candidates();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Failed to open Arduino serial port %s: %s. Available candidates: %s",
        port.c_str(),
        serial_.last_error().c_str(),
        candidates.c_str());
      return;
    }

    last_command_.clear();
    last_sent_pwm_command_ = PwmCommand{0, 0};
    last_command_sent_at_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    reset_encoder_baseline();
    RCLCPP_INFO(
      this->get_logger(),
      "Arduino serial connected: %s @ %d",
      port.c_str(),
      arduino_baud_rate_);
  }

  void reset_encoder_baseline()
  {
    serial_rx_buffer_.clear();
    last_left_ticks_.reset();
    last_right_ticks_.reset();
    last_encoder_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    last_encoder_motion_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    pending_encoder_baseline_samples_ = encoder_baseline_samples_;
  }

  std::string resolve_serial_port() const
  {
    if (arduino_port_ != "auto" && std::filesystem::exists(arduino_port_)) {
      return arduino_port_;
    }

    const auto candidates = serial_port_candidates();
    if (!candidates.empty()) {
      return candidates.front();
    }

    if (arduino_port_ == "auto") {
      return "/dev/ttyACM0";
    }
    return arduino_port_;
  }

  std::vector<std::string> serial_port_candidates() const
  {
    std::vector<std::string> candidates;

    const std::filesystem::path by_id_dir{"/dev/serial/by-id"};
    if (std::filesystem::exists(by_id_dir)) {
      for (const auto & entry : std::filesystem::directory_iterator(
          by_id_dir, std::filesystem::directory_options::skip_permission_denied))
      {
        const std::string path = entry.path().string();
        if (path.find("Arduino") != std::string::npos || path.find("arduino") != std::string::npos ||
          path.find("USB_Serial") != std::string::npos || path.find("usb-1a86") != std::string::npos)
        {
          candidates.push_back(path);
        }
      }
    }

    for (const auto & prefix : {"/dev/ttyACM", "/dev/ttyUSB"}) {
      for (int i = 0; i < 16; ++i) {
        const std::string candidate = std::string(prefix) + std::to_string(i);
        if (std::filesystem::exists(candidate)) {
          candidates.push_back(candidate);
        }
      }
    }

    std::sort(candidates.begin(), candidates.end());
    candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
    return candidates;
  }

  std::string describe_serial_candidates() const
  {
    const auto candidates = serial_port_candidates();
    if (candidates.empty()) {
      return "none";
    }

    std::ostringstream out;
    for (size_t i = 0; i < candidates.size(); ++i) {
      if (i > 0) {
        out << ", ";
      }
      out << candidates[i];
    }
    return out.str();
  }

  void set_mode(const std::string & mode)
  {
    const FollowerMode previous_mode = mode_;
    if (mode == "follow") {
      mode_ = FollowerMode::Follow;
      if (previous_mode != FollowerMode::Follow) {
        follow_mode_started_at_ = this->now();
      }
    } else if (mode == "return") {
      mode_ = FollowerMode::Return;
      clear_intersection_1_left_turn();
    } else if (mode == "align") {
      mode_ = FollowerMode::Align;
      clear_intersection_1_left_turn();
    } else if (mode == "turn_prepare_left") {
      mode_ = FollowerMode::TurnPrepareLeft;
      clear_intersection_1_left_turn();
    } else if (mode == "slot_wait_a") {
      mode_ = FollowerMode::SlotWait;
      if (previous_mode != FollowerMode::SlotWait || slot_wait_task_ != SlotWaitTask::A) {
        start_slot_wait_task(SlotWaitTask::A);
      }
    } else if (mode == "slot_wait_b") {
      mode_ = FollowerMode::SlotWait;
      if (previous_mode != FollowerMode::SlotWait || slot_wait_task_ != SlotWaitTask::B) {
        start_slot_wait_task(SlotWaitTask::B);
      }
    } else {
      mode_ = FollowerMode::Stop;
      slot_wait_task_ = SlotWaitTask::None;
      slot_wait_stage_ = SlotWaitStage::DriveToIntersection;
      clear_intersection_1_left_turn();
      send_pwm_command(PwmCommand{0, 0});
    }

    if (mode_ != previous_mode) {
      RCLCPP_INFO(this->get_logger(), "Mode changed: %s", mode.c_str());
    }
  }

  void start_slot_wait_task(SlotWaitTask task)
  {
    clear_intersection_1_left_turn();
    slot_wait_task_ = task;
    slot_wait_stage_ = SlotWaitStage::DriveToIntersection;
    slot_wait_arrival_published_ = false;

    publish_slot_wait_status(slot_wait_task_key() + "_active");
    RCLCPP_INFO(
      this->get_logger(),
      "Starting RC slot wait task: %s",
      slot_wait_task_key().c_str());
  }

  void control_step()
  {
    if (mode_ == FollowerMode::Stop) {
      send_pwm_command(PwmCommand{0, 0});
      return;
    }

    if (odom_jump_guard_active()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Holding RC car command at 0,0 after odom jump");
      send_pwm_command(PwmCommand{0, 0});
      return;
    }

    if (mode_ == FollowerMode::SlotWait) {
      const bool follower_fresh =
        follower_pose_.has_value() &&
        last_follower_stamp_.nanoseconds() > 0 &&
        (this->now() - last_follower_stamp_).seconds() <= pose_timeout_seconds_;
      if (!follower_fresh) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "Waiting for fresh follower pose in slot wait mode");
        send_pwm_command(PwmCommand{0, 0});
        return;
      }

      send_pwm_command(apply_drive_duty(make_slot_wait_command(follower_pose_.value())));
      return;
    }

    if (!poses_are_fresh()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Waiting for fresh leader/follower poses");
      send_pwm_command(PwmCommand{0, 0});
      return;
    }

    if (
      (mode_ == FollowerMode::Follow || mode_ == FollowerMode::TurnPrepareLeft) &&
      follow_start_delay_seconds_ > 0.0 &&
      follow_mode_started_at_.nanoseconds() > 0 &&
      (this->now() - follow_mode_started_at_).seconds() < follow_start_delay_seconds_)
    {
      send_pwm_command(PwmCommand{0, 0});
      return;
    }

    const Pose2D leader = leader_pose_.value();
    const Pose2D follower = follower_pose_.value();
    const PathTarget path_target = use_topology_path_ ?
      select_topology_path_target(follower, leader) : select_leader_path_target(follower);
    const Pose2D follow_target = path_target.valid ? path_target.pose : leader;
    const double dx = follow_target.x - follower.x;
    const double dy = follow_target.y - follower.y;
    const double distance = std::hypot(dx, dy);
    const double leader_distance = std::hypot(leader.x - follower.x, leader.y - follower.y);
    const double target_heading = std::atan2(dy, dx);
    const double heading_error = normalize_angle(target_heading - follower.yaw);
    const double target_yaw_error = normalize_angle(follow_target.yaw - follower.yaw);

    PwmCommand command;
    if (mode_ == FollowerMode::Align) {
      command = make_align_command(heading_error);
    } else if (mode_ == FollowerMode::Return) {
      const double return_dx = leader.x - follower.x;
      const double return_dy = leader.y - follower.y;
      const double return_distance = std::hypot(return_dx, return_dy);
      const double return_heading_error =
        normalize_angle(std::atan2(return_dy, return_dx) - follower.yaw);
      command = make_return_command(return_distance, return_heading_error);
    } else if (mode_ == FollowerMode::TurnPrepareLeft) {
      command = make_path_follow_command(
        distance, leader_distance, heading_error, target_yaw_error);
    } else {
      command = make_path_follow_command(
        distance, leader_distance, heading_error, target_yaw_error);
    }

    send_pwm_command(apply_drive_duty(command));
  }

  PwmCommand make_slot_wait_command(const Pose2D & follower)
  {
    if (slot_wait_task_ == SlotWaitTask::None) {
      return PwmCommand{0, 0};
    }

    if (slot_wait_stage_ == SlotWaitStage::Arrived) {
      publish_slot_wait_arrived_once();
      return PwmCommand{0, 0};
    }

    const Pose2D intersection = slot_wait_intersection();
    const Pose2D staging = slot_wait_staging_target();
    const Pose2D target =
      slot_wait_stage_ == SlotWaitStage::DriveToIntersection ? intersection : staging;
    const double dx = target.x - follower.x;
    const double dy = target.y - follower.y;
    const double distance = std::hypot(dx, dy);
    const double heading_error =
      normalize_angle(std::atan2(dy, dx) - follower.yaw);

    if (slot_wait_stage_ == SlotWaitStage::DriveToIntersection) {
      if (distance <= slot_intersection_tolerance_) {
        slot_wait_stage_ = SlotWaitStage::AlignToStaging;
        RCLCPP_INFO(
          this->get_logger(),
          "RC reached %s; aligning toward staging point",
          slot_wait_task_ == SlotWaitTask::A ? "intersection_1" : "intersection_2");
        return PwmCommand{0, 0};
      }

      if (std::abs(heading_error) > slot_drive_heading_limit_) {
        return make_slot_turn_command(heading_error);
      }
      return apply_turn_to_pwm(slot_forward_pwm_, heading_error, near_turn_pwm_delta_);
    }

    if (slot_wait_stage_ == SlotWaitStage::AlignToStaging) {
      if (std::abs(heading_error) <= slot_yaw_tolerance_) {
        slot_wait_stage_ = SlotWaitStage::DriveToStaging;
        RCLCPP_INFO(this->get_logger(), "RC aligned; driving to slot staging point");
        return PwmCommand{0, 0};
      }
      return make_slot_turn_command(heading_error);
    }

    if (slot_wait_stage_ == SlotWaitStage::DriveToStaging) {
      if (distance <= slot_staging_tolerance_) {
        slot_wait_stage_ = SlotWaitStage::Arrived;
        publish_slot_wait_arrived_once();
        return PwmCommand{0, 0};
      }

      if (std::abs(heading_error) > slot_drive_heading_limit_) {
        return make_slot_turn_command(heading_error);
      }
      return apply_turn_to_pwm(slot_forward_pwm_, heading_error, near_turn_pwm_delta_);
    }

    return PwmCommand{0, 0};
  }

  PwmCommand make_slot_turn_command(double heading_error) const
  {
    if (std::abs(heading_error) <= slot_yaw_tolerance_) {
      return PwmCommand{0, 0};
    }

    return heading_error > 0.0 ?
      PwmCommand{-slot_turn_pwm_, slot_turn_pwm_} :
      PwmCommand{slot_turn_pwm_, -slot_turn_pwm_};
  }

  Pose2D slot_wait_intersection() const
  {
    const std::string node_name =
      slot_wait_task_ == SlotWaitTask::B ? "intersection_2" : "intersection_1";
    return topology_nodes_.at(node_name);
  }

  Pose2D slot_wait_staging_target() const
  {
    const std::string entry_name =
      slot_wait_task_ == SlotWaitTask::B ? "b_entry" : "a_entry";
    const std::string precision_name =
      slot_wait_task_ == SlotWaitTask::B ? "b_leader_slot_precision" :
      "a_leader_slot_precision";
    const Pose2D entry = topology_nodes_.at(entry_name);
    const Pose2D precision = topology_nodes_.at(precision_name);

    Pose2D target;
    target.x = entry.x + (precision.x - entry.x) * slot_staging_ratio_;
    target.y = entry.y + (precision.y - entry.y) * slot_staging_ratio_;
    target.yaw = std::atan2(precision.y - entry.y, precision.x - entry.x);
    if (slot_wait_task_ == SlotWaitTask::A && slot_a_staging_override_) {
      target.x = slot_a_staging_x_;
      target.y = slot_a_staging_y_;
    }
    return target;
  }

  std::string slot_wait_task_key() const
  {
    if (slot_wait_task_ == SlotWaitTask::B) {
      return "b";
    }
    if (slot_wait_task_ == SlotWaitTask::A) {
      return "a";
    }
    return "none";
  }

  void publish_slot_wait_arrived_once()
  {
    if (slot_wait_arrival_published_) {
      return;
    }

    publish_slot_wait_status(slot_wait_task_key() + "_arrived");
    slot_wait_arrival_published_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "RC reached %s slot staging point",
      slot_wait_task_key().c_str());
  }

  void publish_slot_wait_status(const std::string & status)
  {
    if (!slot_wait_status_pub_) {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = status;
    slot_wait_status_pub_->publish(msg);
  }

  void update_leader_pose(const Pose2D & pose, const rclcpp::Time & stamp)
  {
    update_leader_speed(pose, stamp);
    leader_pose_ = pose;
    last_leader_stamp_ = stamp;
    record_leader_pose(pose, stamp);
  }

  void update_leader_speed(const Pose2D & pose, const rclcpp::Time & stamp)
  {
    if (!leader_pose_.has_value() || last_leader_stamp_.nanoseconds() == 0) {
      leader_pose_speed_ = 0.0;
      return;
    }

    const double dt = (stamp - last_leader_stamp_).seconds();
    if (dt <= 1e-3) {
      return;
    }

    const Pose2D previous = leader_pose_.value();
    const double distance = std::hypot(pose.x - previous.x, pose.y - previous.y);
    leader_pose_speed_ = std::clamp(distance / dt, 0.0, 0.5);
  }

  double effective_leader_speed() const
  {
    const bool cmd_vel_is_fresh =
      leader_cmd_vel_timeout_seconds_ > 0.0 &&
      last_leader_cmd_vel_stamp_.nanoseconds() > 0 &&
      (this->now() - last_leader_cmd_vel_stamp_).seconds() <= leader_cmd_vel_timeout_seconds_;

    if (!cmd_vel_is_fresh) {
      return leader_pose_speed_;
    }

    return leader_cmd_vel_weight_ * leader_cmd_speed_ +
      (1.0 - leader_cmd_vel_weight_) * leader_pose_speed_;
  }

  void record_leader_pose(const Pose2D & pose, const rclcpp::Time & stamp)
  {
    if (!leader_history_.empty()) {
      const Pose2D & last = leader_history_.back().pose;
      const double spacing = std::hypot(pose.x - last.x, pose.y - last.y);
      if (spacing < leader_history_min_spacing_) {
        leader_history_.back() = LeaderPoseSample{pose, stamp};
        prune_leader_history(stamp);
        return;
      }
    }

    leader_history_.push_back(LeaderPoseSample{pose, stamp});
    prune_leader_history(stamp);
  }

  void prune_leader_history(const rclcpp::Time & now)
  {
    while (leader_history_.size() > 1 &&
      (now - leader_history_.front().stamp).seconds() > leader_history_max_seconds_)
    {
      leader_history_.erase(leader_history_.begin());
      if (leader_path_progress_index_ > 0) {
        --leader_path_progress_index_;
      }
    }
  }

  void load_topology_nodes()
  {
    topology_nodes_.clear();
    const YAML::Node topology = YAML::LoadFile(topology_file_);
    const YAML::Node yaml_nodes = topology["nodes"];
    if (!yaml_nodes) {
      throw std::runtime_error("topology.yaml has no nodes section");
    }

    for (YAML::const_iterator it = yaml_nodes.begin(); it != yaml_nodes.end(); ++it) {
      const std::string name = it->first.as<std::string>();
      const YAML::Node data = it->second;
      Pose2D pose;
      pose.x = data["x"].as<double>();
      pose.y = data["y"].as<double>();
      pose.yaw = data["yaw"] ? data["yaw"].as<double>() : 0.0;
      topology_nodes_[name] = pose;
    }
  }

  void load_topology_path()
  {
    topology_path_.clear();
    topology_path_progress_index_ = 0;

    if (topology_route_.size() < 2) {
      throw std::runtime_error("topology_route must contain at least two nodes");
    }

    for (const auto & name : topology_route_) {
      if (topology_nodes_.find(name) == topology_nodes_.end()) {
        throw std::runtime_error("topology_route contains unknown node: " + name);
      }
    }

    topology_path_.push_back(topology_nodes_.at(topology_route_.front()));
    for (size_t route_index = 1; route_index < topology_route_.size(); ++route_index) {
      append_resampled_topology_segment(
        topology_nodes_.at(topology_route_[route_index - 1]),
        topology_nodes_.at(topology_route_[route_index]));
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Loaded topology path from %s with %zu route nodes and %zu sampled points",
      topology_file_.c_str(),
      topology_route_.size(),
      topology_path_.size());
  }

  void append_resampled_topology_segment(const Pose2D & start, const Pose2D & end)
  {
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double segment = std::hypot(dx, dy);
    if (segment <= 1e-6) {
      topology_path_.push_back(end);
      return;
    }

    const double segment_yaw = std::atan2(dy, dx);
    const int samples = std::max(1, static_cast<int>(std::ceil(segment / topology_path_spacing_)));
    for (int i = 1; i <= samples; ++i) {
      const double ratio = static_cast<double>(i) / static_cast<double>(samples);
      Pose2D pose;
      pose.x = start.x + dx * ratio;
      pose.y = start.y + dy * ratio;
      pose.yaw = (i == samples) ? end.yaw : segment_yaw;
      topology_path_.push_back(pose);
    }
  }

  PathTarget select_topology_path_target(const Pose2D & follower, const Pose2D & leader)
  {
    if (topology_path_.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "topology path is empty; falling back to leader pose");
      return PathTarget{leader, true};
    }

    if (topology_path_.size() == 1) {
      return PathTarget{topology_path_.back(), true};
    }

    const PathProjection follower_projection = project_pose_to_path(
      topology_path_, follower, topology_path_progress_index_);
    const PathProjection leader_projection = project_pose_to_path(
      topology_path_, leader, 0);

    if (!follower_projection.valid || !leader_projection.valid) {
      return PathTarget{topology_path_.back(), true};
    }

    topology_path_progress_index_ =
      std::max(topology_path_progress_index_, follower_projection.segment_index);

    const double gap = leader_projection.progress - follower_projection.progress;
    if (gap <= topology_min_gap_distance_) {
      return PathTarget{follower, true};
    }

    const double follower_target_progress =
      follower_projection.progress + topology_lookahead_distance_;
    const double leader_limited_progress =
      std::max(0.0, leader_projection.progress - topology_follow_gap_distance_);
    const double target_progress =
      std::min(follower_target_progress, leader_limited_progress);

    if (target_progress <= follower_projection.progress + 1e-3) {
      return PathTarget{follower, true};
    }

    return PathTarget{sample_path_at_progress(topology_path_, target_progress), true};
  }

  PathProjection project_pose_to_path(
    const std::vector<Pose2D> & path,
    const Pose2D & pose,
    size_t start_index) const
  {
    if (path.size() < 2) {
      return PathProjection{};
    }

    if (start_index + 1 >= path.size()) {
      start_index = path.size() - 2;
    }

    double progress_before_segment = path_progress_before_index(path, start_index);
    double best_distance_sq = std::numeric_limits<double>::max();
    PathProjection best;

    for (size_t i = start_index; i + 1 < path.size(); ++i) {
      const Pose2D & start = path[i];
      const Pose2D & end = path[i + 1];
      const double vx = end.x - start.x;
      const double vy = end.y - start.y;
      const double segment_sq = vx * vx + vy * vy;
      if (segment_sq <= 1e-12) {
        continue;
      }

      const double wx = pose.x - start.x;
      const double wy = pose.y - start.y;
      const double ratio = std::clamp((wx * vx + wy * vy) / segment_sq, 0.0, 1.0);
      const double closest_x = start.x + vx * ratio;
      const double closest_y = start.y + vy * ratio;
      const double dx = pose.x - closest_x;
      const double dy = pose.y - closest_y;
      const double distance_sq = dx * dx + dy * dy;
      const double segment = std::sqrt(segment_sq);

      if (distance_sq < best_distance_sq) {
        best_distance_sq = distance_sq;
        best.progress = progress_before_segment + segment * ratio;
        best.segment_index = i;
        best.segment_ratio = ratio;
        best.valid = true;
      }

      progress_before_segment += segment;
    }

    return best;
  }

  double path_progress_before_index(const std::vector<Pose2D> & path, size_t index) const
  {
    double progress = 0.0;
    const size_t end = std::min(index, path.size() > 0 ? path.size() - 1 : 0);
    for (size_t i = 0; i < end; ++i) {
      progress += std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
    }
    return progress;
  }

  Pose2D sample_path_at_progress(const std::vector<Pose2D> & path, double progress) const
  {
    if (path.empty()) {
      return Pose2D{};
    }
    if (path.size() == 1 || progress <= 0.0) {
      return path.front();
    }

    double remaining = progress;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
      const Pose2D & start = path[i];
      const Pose2D & end = path[i + 1];
      const double segment = std::hypot(end.x - start.x, end.y - start.y);
      if (segment <= 1e-6) {
        continue;
      }

      if (remaining <= segment) {
        const double ratio = std::clamp(remaining / segment, 0.0, 1.0);
        Pose2D target;
        target.x = start.x + (end.x - start.x) * ratio;
        target.y = start.y + (end.y - start.y) * ratio;
        target.yaw = std::atan2(end.y - start.y, end.x - start.x);
        return target;
      }

      remaining -= segment;
    }

    return path.back();
  }

  PathTarget select_pose_path_target(
    const std::vector<Pose2D> & path,
    size_t & progress_index,
    const Pose2D & follower,
    const char * source_name)
  {
    if (path.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "%s is empty; falling back to leader pose",
        source_name);
      return PathTarget{leader_pose_.value_or(Pose2D{}), leader_pose_.has_value()};
    }

    if (leader_path_follow_distance_ <= 0.0 || path.size() == 1) {
      return PathTarget{path.back(), true};
    }

    if (progress_index + 1 >= path.size()) {
      progress_index = path.size() - 2;
    }

    double best_distance_sq = std::numeric_limits<double>::max();
    size_t best_segment_index = progress_index;
    double best_segment_ratio = 0.0;
    bool found_segment = false;

    for (size_t i = progress_index; i + 1 < path.size(); ++i) {
      const Pose2D & start = path[i];
      const Pose2D & end = path[i + 1];
      const double vx = end.x - start.x;
      const double vy = end.y - start.y;
      const double segment_sq = vx * vx + vy * vy;
      if (segment_sq <= 1e-12) {
        continue;
      }

      const double wx = follower.x - start.x;
      const double wy = follower.y - start.y;
      const double ratio = std::clamp((wx * vx + wy * vy) / segment_sq, 0.0, 1.0);
      const double closest_x = start.x + vx * ratio;
      const double closest_y = start.y + vy * ratio;
      const double dx = follower.x - closest_x;
      const double dy = follower.y - closest_y;
      const double distance_sq = dx * dx + dy * dy;

      if (distance_sq < best_distance_sq) {
        best_distance_sq = distance_sq;
        best_segment_index = i;
        best_segment_ratio = ratio;
        found_segment = true;
      }
    }

    if (!found_segment) {
      return PathTarget{path.back(), true};
    }

    progress_index = std::max(progress_index, best_segment_index);
    if (best_segment_ratio >= 0.95 && progress_index + 2 < path.size()) {
      ++progress_index;
      best_segment_index = progress_index;
      best_segment_ratio = 0.0;
    }

    double remaining = leader_path_follow_distance_;
    for (size_t i = best_segment_index; i + 1 < path.size(); ++i) {
      const Pose2D & start = path[i];
      const Pose2D & end = path[i + 1];
      const double segment = std::hypot(end.x - start.x, end.y - start.y);
      if (segment <= 1e-6) {
        continue;
      }

      const double start_ratio = (i == best_segment_index) ? best_segment_ratio : 0.0;
      const double available = segment * (1.0 - start_ratio);
      if (remaining <= available) {
        const double ratio = start_ratio + remaining / segment;
        Pose2D target;
        target.x = start.x + (end.x - start.x) * ratio;
        target.y = start.y + (end.y - start.y) * ratio;
        target.yaw = std::atan2(end.y - start.y, end.x - start.x);
        return PathTarget{target, true};
      }

      remaining -= available;
    }

    return PathTarget{path.back(), true};
  }

  PathTarget select_leader_path_target(const Pose2D & follower)
  {
    if (leader_history_.empty()) {
      return PathTarget{leader_pose_.value_or(Pose2D{}), leader_pose_.has_value()};
    }

    if (leader_path_follow_distance_ <= 0.0 || leader_history_.size() == 1) {
      return PathTarget{leader_history_.back().pose, true};
    }

    if (leader_path_progress_index_ + 1 >= leader_history_.size()) {
      leader_path_progress_index_ = leader_history_.size() - 2;
    }

    double best_distance_sq = std::numeric_limits<double>::max();
    size_t best_segment_index = leader_path_progress_index_;
    double best_segment_ratio = 0.0;
    bool found_segment = false;

    for (size_t i = leader_path_progress_index_; i + 1 < leader_history_.size(); ++i) {
      const Pose2D & start = leader_history_[i].pose;
      const Pose2D & end = leader_history_[i + 1].pose;
      const double vx = end.x - start.x;
      const double vy = end.y - start.y;
      const double segment_sq = vx * vx + vy * vy;
      if (segment_sq <= 1e-12) {
        continue;
      }

      const double wx = follower.x - start.x;
      const double wy = follower.y - start.y;
      const double ratio = std::clamp((wx * vx + wy * vy) / segment_sq, 0.0, 1.0);
      const double closest_x = start.x + vx * ratio;
      const double closest_y = start.y + vy * ratio;
      const double dx = follower.x - closest_x;
      const double dy = follower.y - closest_y;
      const double distance_sq = dx * dx + dy * dy;

      if (distance_sq < best_distance_sq) {
        best_distance_sq = distance_sq;
        best_segment_index = i;
        best_segment_ratio = ratio;
        found_segment = true;
      }
    }

    if (!found_segment) {
      return PathTarget{leader_history_.back().pose, true};
    }

    leader_path_progress_index_ = std::max(leader_path_progress_index_, best_segment_index);
    if (best_segment_ratio >= 0.95 && leader_path_progress_index_ + 2 < leader_history_.size()) {
      ++leader_path_progress_index_;
      best_segment_index = leader_path_progress_index_;
      best_segment_ratio = 0.0;
    }

    double remaining = leader_path_follow_distance_;
    for (size_t i = best_segment_index; i + 1 < leader_history_.size(); ++i) {
      const Pose2D & start = leader_history_[i].pose;
      const Pose2D & end = leader_history_[i + 1].pose;
      const double segment = std::hypot(end.x - start.x, end.y - start.y);
      if (segment <= 1e-6) {
        continue;
      }

      const double start_ratio = (i == best_segment_index) ? best_segment_ratio : 0.0;
      const double available = segment * (1.0 - start_ratio);
      if (remaining <= available) {
        const double ratio = start_ratio + remaining / segment;
        Pose2D target;
        target.x = start.x + (end.x - start.x) * ratio;
        target.y = start.y + (end.y - start.y) * ratio;
        target.yaw = std::atan2(end.y - start.y, end.x - start.x);
        return PathTarget{target, true};
      }

      remaining -= available;
    }

    return PathTarget{leader_history_.back().pose, true};
  }

  void serial_odom_step()
  {
    if (!publish_encoder_odom_) {
      return;
    }

    serial_rx_buffer_ += serial_.read_available();
    size_t newline_pos = std::string::npos;
    while ((newline_pos = serial_rx_buffer_.find('\n')) != std::string::npos) {
      const std::string line = serial_rx_buffer_.substr(0, newline_pos);
      serial_rx_buffer_.erase(0, newline_pos + 1);
      publish_serial_debug_line(line);
      long left_ticks = 0;
      long right_ticks = 0;
      if (parse_tick_line(line, left_ticks, right_ticks)) {
        publish_encoder_ticks(left_ticks, right_ticks);
        update_encoder_odom(left_ticks, right_ticks);
      } else if (!line.empty()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Ignoring unrecognized encoder line: '%s'",
          line.c_str());
      }
    }

    update_command_odom_fallback();
    publish_current_odom();
  }

  void publish_encoder_ticks(long left_ticks, long right_ticks)
  {
    std_msgs::msg::Int64MultiArray msg;
    msg.data = {left_ticks, right_ticks};
    encoder_ticks_pub_->publish(msg);
  }

  void publish_serial_debug_line(const std::string & line)
  {
    std_msgs::msg::String msg;
    msg.data = line;
    serial_debug_pub_->publish(msg);
  }

  bool parse_tick_line(const std::string & line, long & left_ticks, long & right_ticks) const
  {
    const auto left_pos = line.find("L_Tick:");
    const auto right_pos = line.find("R_Tick:");
    if (left_pos != std::string::npos && right_pos != std::string::npos) {
      try {
        const std::string left_text = line.substr(left_pos + 7, right_pos - (left_pos + 7));
        const std::string right_text = line.substr(right_pos + 7);
        left_ticks = std::stol(left_text);
        right_ticks = std::stol(right_text);
        return true;
      } catch (const std::exception &) {
        return false;
      }
    }

    const auto comma_pos = line.find(',');
    if (comma_pos == std::string::npos) {
      return false;
    }
    try {
      const std::string left_text = line.substr(0, comma_pos);
      const std::string right_text = line.substr(comma_pos + 1);
      left_ticks = std::stol(left_text);
      right_ticks = std::stol(right_text);
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  void initialize_odom_from_leader()
  {
    if (odom_initialized_) {
      return;
    }

    const rclcpp::Time now = this->now();
    if (!auto_initialize_odom_from_leader_) {
      odom_pose_.x = initial_rc_x_;
      odom_pose_.y = initial_rc_y_;
      odom_pose_.yaw = normalize_angle(initial_rc_yaw_);
      follower_pose_ = odom_pose_;
      last_follower_stamp_ = now;
      last_odom_integrated_stamp_ = now;
      odom_initialized_ = true;

      RCLCPP_INFO(
        this->get_logger(),
        "Initialized RC car odom from initial parameters: x=%.2f y=%.2f yaw=%.2f",
        odom_pose_.x,
        odom_pose_.y,
        odom_pose_.yaw);
      return;
    }

    if (!leader_pose_.has_value()) {
      return;
    }

    const Pose2D leader = leader_pose_.value();
    odom_pose_.x = leader.x - std::cos(leader.yaw) * initial_follow_distance_;
    odom_pose_.y = leader.y - std::sin(leader.yaw) * initial_follow_distance_;
    odom_pose_.yaw = leader.yaw;
    follower_pose_ = odom_pose_;
    last_follower_stamp_ = now;
    last_odom_integrated_stamp_ = now;
    odom_initialized_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Initialized RC car odom from leader pose: x=%.2f y=%.2f yaw=%.2f",
      odom_pose_.x,
      odom_pose_.y,
      odom_pose_.yaw);
  }

  void update_encoder_odom(long left_ticks, long right_ticks)
  {
    initialize_odom_from_leader();
    if (!odom_initialized_) {
      return;
    }

    if (!last_left_ticks_.has_value() || !last_right_ticks_.has_value()) {
      last_left_ticks_ = left_ticks;
      last_right_ticks_ = right_ticks;
      last_encoder_stamp_ = this->now();
      last_odom_integrated_stamp_ = last_encoder_stamp_;
      return;
    }

    if (pending_encoder_baseline_samples_ > 0) {
      last_left_ticks_ = left_ticks;
      last_right_ticks_ = right_ticks;
      last_encoder_stamp_ = this->now();
      last_odom_integrated_stamp_ = last_encoder_stamp_;
      --pending_encoder_baseline_samples_;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Stabilizing encoder baseline: left=%ld right=%ld remaining=%d",
        left_ticks,
        right_ticks,
        pending_encoder_baseline_samples_);
      return;
    }

    const long delta_left_ticks = left_ticks - last_left_ticks_.value();
    const long delta_right_ticks = right_ticks - last_right_ticks_.value();
    const rclcpp::Time now = this->now();

    if (
      std::abs(delta_left_ticks) > max_encoder_tick_delta_per_update_ ||
      std::abs(delta_right_ticks) > max_encoder_tick_delta_per_update_)
    {
      last_left_ticks_ = left_ticks;
      last_right_ticks_ = right_ticks;
      last_encoder_stamp_ = now;
      last_odom_integrated_stamp_ = now;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Ignoring implausible encoder delta: left_delta=%ld right_delta=%ld limit=%d",
        delta_left_ticks,
        delta_right_ticks,
        max_encoder_tick_delta_per_update_);
      return;
    }

    last_left_ticks_ = left_ticks;
    last_right_ticks_ = right_ticks;
    last_encoder_stamp_ = now;
    if (delta_left_ticks != 0 || delta_right_ticks != 0) {
      last_encoder_motion_stamp_ = now;
    }

    const double delta_left = static_cast<double>(delta_left_ticks) * left_meters_per_tick_;
    const double delta_right = static_cast<double>(delta_right_ticks) * right_meters_per_tick_;
    const double delta_distance = 0.5 * (delta_left + delta_right);
    const double delta_yaw = (delta_right - delta_left) / wheel_base_;
    if (reject_encoder_odom_jump_if_needed(delta_distance, delta_yaw, now)) {
      return;
    }

    update_encoder_balance(delta_left, delta_right);
    const double mid_yaw = odom_pose_.yaw + 0.5 * delta_yaw;

    odom_pose_.x += delta_distance * std::cos(mid_yaw);
    odom_pose_.y += delta_distance * std::sin(mid_yaw);
    odom_pose_.yaw = normalize_angle(odom_pose_.yaw + delta_yaw);
    follower_pose_ = odom_pose_;
    last_follower_stamp_ = now;
    last_odom_integrated_stamp_ = last_follower_stamp_;
  }

  void update_encoder_balance(double delta_left, double delta_right)
  {
    if (!enable_encoder_balance_) {
      return;
    }

    const int left_command = last_sent_pwm_command_.left;
    const int right_command = last_sent_pwm_command_.right;
    if (left_command <= 0 && right_command <= 0) {
      encoder_balance_trim_pwm_ *= encoder_balance_decay_;
      return;
    }

    if (std::abs(left_command - right_command) > encoder_balance_update_pwm_tolerance_) {
      return;
    }

    const double left_distance = std::abs(delta_left);
    const double right_distance = std::abs(delta_right);
    const double traveled = 0.5 * (left_distance + right_distance);
    if (traveled < 1e-5) {
      return;
    }

    const double distance_error = left_distance - right_distance;
    encoder_balance_trim_pwm_ += encoder_balance_gain_pwm_per_meter_ * distance_error;
    encoder_balance_trim_pwm_ = std::clamp(
      encoder_balance_trim_pwm_,
      -static_cast<double>(encoder_balance_max_trim_),
      static_cast<double>(encoder_balance_max_trim_));
  }

  void update_command_odom_fallback()
  {
    if (!enable_command_odom_fallback_) {
      return;
    }

    initialize_odom_from_leader();
    if (!odom_initialized_) {
      return;
    }

    const rclcpp::Time now = this->now();
    if (last_sent_pwm_command_.left == 0 && last_sent_pwm_command_.right == 0) {
      last_odom_integrated_stamp_ = now;
      return;
    }

    const bool encoder_motion_is_fresh =
      last_encoder_motion_stamp_.nanoseconds() > 0 &&
      (now - last_encoder_motion_stamp_).seconds() <= encoder_timeout_seconds_;
    if (encoder_motion_is_fresh) {
      last_odom_integrated_stamp_ = now;
      return;
    }

    if (last_odom_integrated_stamp_.nanoseconds() == 0) {
      last_odom_integrated_stamp_ = now;
      return;
    }

    const double dt = (now - last_odom_integrated_stamp_).seconds();
    last_odom_integrated_stamp_ = now;
    if (dt <= 0.0 || dt > 1.0) {
      return;
    }

    const double left_speed = pwm_to_wheel_speed(last_sent_pwm_command_.left);
    const double right_speed = pwm_to_wheel_speed(last_sent_pwm_command_.right);
    const double linear = 0.5 * (left_speed + right_speed);
    const double angular = (right_speed - left_speed) / wheel_base_;
    const double delta_yaw = angular * dt;
    const double mid_yaw = odom_pose_.yaw + 0.5 * delta_yaw;

    odom_pose_.x += linear * dt * std::cos(mid_yaw);
    odom_pose_.y += linear * dt * std::sin(mid_yaw);
    odom_pose_.yaw = normalize_angle(odom_pose_.yaw + delta_yaw);
    follower_pose_ = odom_pose_;
    last_follower_stamp_ = now;

    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      3000,
      "Encoder tick motion is stale; estimating /rc_car/odom from PWM command %d,%d",
      last_sent_pwm_command_.left,
      last_sent_pwm_command_.right);
  }

  double pwm_to_wheel_speed(int pwm) const
  {
    if (pwm == 0 || max_forward_pwm_ <= 0) {
      return 0.0;
    }
    const double normalized =
      std::clamp(static_cast<double>(pwm) / static_cast<double>(max_forward_pwm_), -1.0, 1.0);
    return normalized * max_pwm_wheel_speed_;
  }

  void publish_current_odom()
  {
    initialize_odom_from_leader();
    if (!odom_initialized_) {
      return;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = odom_frame_id_;
    odom.header.stamp = this->now();
    odom.child_frame_id = base_frame_id_;
    odom.pose.pose.position.x = odom_pose_.x;
    odom.pose.pose.position.y = odom_pose_.y;
    odom.pose.pose.orientation = quaternion_from_yaw(odom_pose_.yaw);
    odom_pub_->publish(odom);
  }

  bool odom_jump_guard_active() const
  {
    return odom_jump_guard_until_.nanoseconds() > 0 && this->now() < odom_jump_guard_until_;
  }

  void start_odom_jump_guard(const rclcpp::Time & now)
  {
    if (odom_jump_command_hold_seconds_ <= 0.0) {
      odom_jump_guard_until_ = now;
      return;
    }

    odom_jump_guard_until_ =
      now + rclcpp::Duration::from_seconds(odom_jump_command_hold_seconds_);
  }

  bool reject_odom_jump_if_needed(
    const Pose2D & candidate,
    const rclcpp::Time & now,
    const char * source)
  {
    if (!follower_pose_.has_value()) {
      return false;
    }

    const Pose2D previous = follower_pose_.value();
    const double translation = std::hypot(candidate.x - previous.x, candidate.y - previous.y);
    const double yaw_delta = std::abs(normalize_angle(candidate.yaw - previous.yaw));
    if (
      translation <= max_odom_translation_delta_per_update_ &&
      yaw_delta <= max_odom_yaw_delta_per_update_)
    {
      return false;
    }

    start_odom_jump_guard(now);
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Rejecting %s jump: translation=%.3f yaw_delta=%.3f limits=(%.3f, %.3f)",
      source,
      translation,
      yaw_delta,
      max_odom_translation_delta_per_update_,
      max_odom_yaw_delta_per_update_);
    send_pwm_command(PwmCommand{0, 0});
    return true;
  }

  bool reject_encoder_odom_jump_if_needed(
    double delta_distance,
    double delta_yaw,
    const rclcpp::Time & now)
  {
    const double translation = std::abs(delta_distance);
    const double yaw_delta = std::abs(delta_yaw);
    if (
      translation <= max_odom_translation_delta_per_update_ &&
      yaw_delta <= max_odom_yaw_delta_per_update_)
    {
      return false;
    }

    start_odom_jump_guard(now);
    last_odom_integrated_stamp_ = now;
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Rejecting encoder odom jump: translation=%.3f yaw_delta=%.3f limits=(%.3f, %.3f)",
      translation,
      yaw_delta,
      max_odom_translation_delta_per_update_,
      max_odom_yaw_delta_per_update_);
    send_pwm_command(PwmCommand{0, 0});
    return true;
  }

  bool poses_are_fresh() const
  {
    if (!leader_pose_.has_value() || !follower_pose_.has_value()) {
      return false;
    }

    const rclcpp::Time now = this->now();
    const double leader_age = (now - last_leader_stamp_).seconds();
    const double follower_age = (now - last_follower_stamp_).seconds();
    return leader_age <= pose_timeout_seconds_ && follower_age <= pose_timeout_seconds_;
  }

  PwmCommand make_follow_command(
    double distance,
    double heading_error,
    double leader_yaw_error) const
  {
    double turn_error = heading_error;
    if (std::abs(turn_error) <= heading_deadband_ &&
      std::abs(leader_yaw_error) > leader_turn_assist_heading_)
    {
      turn_error = leader_yaw_error;
    }

    if (distance < too_close_distance_) {
      if (distance <= emergency_stop_distance_) {
        return PwmCommand{0, 0};
      }
      return apply_turn_to_pwm(slow_forward_pwm_, turn_error, near_turn_pwm_delta_);
    }

    if (std::abs(distance - target_distance_) <= distance_deadband_) {
      return apply_turn_to_pwm(slow_forward_pwm_, turn_error, near_turn_pwm_delta_);
    }

    if (distance < target_distance_) {
      return apply_turn_to_pwm(slow_forward_pwm_, turn_error, near_turn_pwm_delta_);
    }

    return apply_turn_to_pwm(calculate_forward_pwm(distance - target_distance_), turn_error);
  }

  PwmCommand make_path_follow_command(
    double path_target_distance,
    double leader_distance,
    double heading_error,
    double target_yaw_error) const
  {
    double turn_error = heading_error;
    if (std::abs(turn_error) <= heading_deadband_ &&
      std::abs(target_yaw_error) > leader_turn_assist_heading_)
    {
      turn_error = target_yaw_error;
    }

    const bool leader_moving = is_leader_moving();

    if (leader_distance <= collision_stop_distance_) {
      return PwmCommand{0, 0};
    }

    if (leader_distance <= emergency_stop_distance_) {
      if (!leader_moving) {
        return PwmCommand{0, 0};
      }
      return apply_turn_to_pwm(
        scale_forward_for_turn(slow_forward_pwm_, turn_error), turn_error, near_turn_pwm_delta_);
    }

    if (leader_distance < too_close_distance_) {
      if (!leader_moving) {
        return PwmCommand{0, 0};
      }
      return apply_turn_to_pwm(
        scale_forward_for_turn(slow_forward_pwm_, turn_error), turn_error, near_turn_pwm_delta_);
    }

    if (leader_distance < target_distance_ - distance_deadband_) {
      if (!leader_moving) {
        return PwmCommand{0, 0};
      }
      return apply_turn_to_pwm(
        scale_forward_for_turn(calculate_forward_pwm(0.0), turn_error),
        turn_error,
        near_turn_pwm_delta_);
    }

    if (path_target_distance <= leader_path_goal_tolerance_) {
      if (!leader_moving) {
        return PwmCommand{0, 0};
      }
      return apply_turn_to_pwm(
        scale_forward_for_turn(calculate_forward_pwm(0.0), turn_error),
        turn_error,
        near_turn_pwm_delta_);
    }

    const int forward_pwm =
      scale_forward_for_turn(calculate_forward_pwm(leader_distance - target_distance_), turn_error);
    return apply_turn_to_pwm(forward_pwm, turn_error);
  }

  PwmCommand make_return_command(double distance, double heading_error) const
  {
    if (distance <= target_distance_) {
      return PwmCommand{0, 0};
    }
    return apply_turn_to_pwm(calculate_forward_pwm(distance - target_distance_), heading_error);
  }

  int calculate_forward_pwm(double distance_error) const
  {
    const double pwm =
      static_cast<double>(base_forward_pwm_) +
      leader_speed_gain_pwm_ * effective_leader_speed() +
      distance_gain_pwm_ * distance_error;
    return std::clamp(static_cast<int>(std::lround(pwm)), min_forward_pwm_, max_forward_pwm_);
  }

  bool is_leader_moving() const
  {
    return effective_leader_speed() >= leader_moving_speed_threshold_;
  }

  int scale_forward_for_turn(int forward_pwm, double heading_error) const
  {
    if (std::abs(heading_error) <= heading_deadband_) {
      return forward_pwm;
    }

    const double ratio = std::clamp(
      (std::abs(heading_error) - heading_deadband_) /
      std::max(1e-6, turn_slowdown_heading_ - heading_deadband_),
      0.0,
      1.0);
    const int limited_pwm = static_cast<int>(std::lround(
      static_cast<double>(forward_pwm) * (1.0 - 0.55 * ratio)));
    return std::clamp(limited_pwm, min_turn_forward_pwm_, forward_pwm);
  }

  PwmCommand make_in_place_turn_command(double heading_error) const
  {
    if (std::abs(heading_error) <= heading_deadband_) {
      return PwmCommand{0, 0};
    }

    if (heading_error > 0.0) {
      return PwmCommand{-in_place_turn_pwm_, in_place_turn_pwm_};
    }
    return PwmCommand{in_place_turn_pwm_, -in_place_turn_pwm_};
  }

  PwmCommand apply_turn_to_pwm(int forward_pwm, double heading_error) const
  {
    return apply_turn_to_pwm(forward_pwm, heading_error, turn_pwm_delta_);
  }

  PwmCommand apply_turn_to_pwm(int forward_pwm, double heading_error, int max_turn_delta) const
  {
    int left = forward_pwm;
    int right = forward_pwm;

    if (std::abs(heading_error) > heading_deadband_) {
      const int turn_delta = std::clamp(
        static_cast<int>(std::lround(
          turn_pwm_delta_ * std::min(std::abs(heading_error), 1.2) / 1.2)),
        0,
        std::clamp(max_turn_delta, 0, turn_pwm_delta_));

      if (heading_error > 0.0) {
        left -= turn_delta;
        right += turn_delta;
      } else {
        left += turn_delta;
        right -= turn_delta;
      }
    }

    const int total_straight_trim =
      straight_pwm_trim_ + static_cast<int>(std::lround(encoder_balance_trim_pwm_));
    if (forward_pwm > 0 && total_straight_trim != 0) {
      left -= total_straight_trim;
      right += total_straight_trim;
    }

    const int command_limit = std::max(max_forward_pwm_, reverse_pwm_);
    return PwmCommand{
      std::clamp(left, -command_limit, max_forward_pwm_),
      std::clamp(right, -command_limit, max_forward_pwm_)};
  }

  bool handle_intersection_1_left_turn(const Pose2D & follower)
  {
    (void)follower;

    if (intersection_1_left_turn_active_) {
      if (this->now() < intersection_1_left_turn_end_at_) {
        send_pwm_command(PwmCommand{-in_place_turn_pwm_, in_place_turn_pwm_});
        return true;
      }

      intersection_1_left_turn_active_ = false;
      intersection_1_left_turn_pending_ = false;
      intersection_1_left_turn_completed_ = true;
      mode_ = FollowerMode::Follow;
      follow_mode_started_at_ = this->now();
      send_pwm_command(PwmCommand{0, 0});
      RCLCPP_INFO(this->get_logger(), "Intersection_1 left turn complete; resuming follow");
      return true;
    }

    if (!intersection_1_left_turn_pending_ || intersection_1_left_turn_completed_) {
      return false;
    }

    const double elapsed_since_request =
      intersection_1_left_turn_requested_at_.nanoseconds() > 0 ?
      (this->now() - intersection_1_left_turn_requested_at_).seconds() : 0.0;
    if (elapsed_since_request < turn_prepare_turn_delay_seconds_) {
      return false;
    }

    intersection_1_left_turn_active_ = true;
    intersection_1_left_turn_end_at_ =
      this->now() + rclcpp::Duration::from_seconds(intersection_1_turn_seconds_);
    RCLCPP_INFO(
      this->get_logger(),
      "Starting intersection_1 left turn after %.2f seconds: pwm=%d,%d duration=%.2f",
      elapsed_since_request,
      -in_place_turn_pwm_,
      in_place_turn_pwm_,
      intersection_1_turn_seconds_);
    send_pwm_command(PwmCommand{-in_place_turn_pwm_, in_place_turn_pwm_});
    return true;
  }

  void clear_intersection_1_left_turn()
  {
    intersection_1_left_turn_pending_ = false;
    intersection_1_left_turn_active_ = false;
  }

  PwmCommand make_align_command(double heading_error) const
  {
    if (std::abs(heading_error) <= heading_deadband_) {
      return PwmCommand{0, 0};
    }
    return make_in_place_turn_command(heading_error);
  }

  PwmCommand apply_drive_duty(const PwmCommand & command) const
  {
    return apply_drive_duty(command, drive_duty_cycle_);
  }

  PwmCommand apply_drive_duty(const PwmCommand & command, double duty_cycle) const
  {
    if ((command.left == 0 && command.right == 0) || duty_cycle >= 0.999) {
      return command;
    }

    duty_cycle = std::clamp(duty_cycle, 0.0, 1.0);
    const double cycle_position =
      std::fmod(this->now().seconds(), drive_cycle_seconds_) / drive_cycle_seconds_;
    return cycle_position < duty_cycle ? command : PwmCommand{0, 0};
  }

  std::string format_pwm_command(const PwmCommand & command) const
  {
    return std::to_string(std::clamp(command.left, -255, 255)) + "," +
      std::to_string(std::clamp(command.right, -255, 255));
  }

  void send_pwm_command(const PwmCommand & pwm_command)
  {
    const std::string command = format_pwm_command(pwm_command);
    const rclcpp::Time now = this->now();
    if (command == last_command_ && command_resend_period_seconds_ > 0.0 &&
      last_command_sent_at_.nanoseconds() > 0 &&
      (now - last_command_sent_at_).seconds() < command_resend_period_seconds_)
    {
      return;
    }

    if (!serial_.write_line(command)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Failed to send Arduino command %s: %s",
        command.c_str(),
        serial_.last_error().c_str());
      return;
    }

    last_command_ = command;
    last_sent_pwm_command_ = pwm_command;
    last_command_sent_at_ = now;
    std_msgs::msg::String msg;
    msg.data = command;
    command_pub_->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "Arduino command: %s", command.c_str());
  }

  SerialPort serial_;
  FollowerMode mode_{FollowerMode::Stop};

  std::string leader_pose_type_{"pose_stamped"};
  std::string arduino_port_{"auto"};
  int arduino_baud_rate_{115200};
  double target_distance_{0.70};
  double distance_deadband_{0.08};
  double too_close_distance_{0.45};
  double emergency_stop_distance_{0.25};
  double collision_stop_distance_{0.08};
  double heading_deadband_{0.18};
  double pose_timeout_seconds_{0.70};
  double drive_duty_cycle_{1.0};
  double drive_cycle_seconds_{0.20};
  double follow_start_delay_seconds_{2.0};
  bool publish_encoder_odom_{true};
  bool auto_initialize_odom_from_leader_{true};
  double initial_rc_x_{0.0};
  double initial_rc_y_{0.0};
  double initial_rc_yaw_{0.0};
  std::string odom_frame_id_{"map"};
  std::string base_frame_id_{"rc_car_base_link"};
  double initial_follow_distance_{0.70};
  double wheel_base_{0.16};
  double left_meters_per_tick_{0.0005882353};
  double right_meters_per_tick_{0.0005882353};
  bool enable_command_odom_fallback_{true};
  double encoder_timeout_seconds_{0.50};
  int encoder_baseline_samples_{3};
  int max_encoder_tick_delta_per_update_{1000};
  double max_odom_translation_delta_per_update_{0.50};
  double max_odom_yaw_delta_per_update_{1.20};
  double odom_jump_command_hold_seconds_{2.0};
  double max_pwm_wheel_speed_{0.24};
  double leader_turn_assist_heading_{0.35};
  double command_resend_period_seconds_{0.25};
  double leader_history_max_seconds_{60.0};
  double leader_history_min_spacing_{0.01};
  double leader_path_follow_distance_{0.70};
  double leader_path_goal_tolerance_{0.18};
  bool use_topology_path_{false};
  std::string topology_file_;
  std::vector<std::string> topology_route_;
  double topology_path_spacing_{0.05};
  double topology_follow_gap_distance_{0.70};
  double topology_min_gap_distance_{0.35};
  double topology_lookahead_distance_{0.40};
  int min_forward_pwm_{85};
  int slow_forward_pwm_{65};
  int min_turn_forward_pwm_{70};
  int base_forward_pwm_{115};
  int max_forward_pwm_{190};
  int reverse_pwm_{90};
  int turn_pwm_delta_{45};
  int near_turn_pwm_delta_{15};
  int straight_pwm_trim_{0};
  bool enable_encoder_balance_{false};
  double encoder_balance_gain_pwm_per_meter_{400.0};
  int encoder_balance_max_trim_{30};
  int encoder_balance_update_pwm_tolerance_{12};
  double encoder_balance_decay_{0.98};
  int in_place_turn_pwm_{105};
  double leader_speed_gain_pwm_{260.0};
  double leader_moving_speed_threshold_{0.03};
  double leader_cmd_vel_timeout_seconds_{0.50};
  double leader_cmd_vel_speed_scale_{1.0};
  double leader_cmd_vel_weight_{0.75};
  double distance_gain_pwm_{55.0};
  double turn_slowdown_heading_{0.75};
  double turn_prepare_duty_cycle_{0.45};
  double turn_prepare_close_distance_{0.62};
  double turn_prepare_turn_delay_seconds_{2.0};
  double intersection_1_x_{2.442805051803589};
  double intersection_1_y_{-0.015692830085754395};
  double intersection_1_turn_radius_{0.35};
  double intersection_1_turn_seconds_{1.20};
  std::string intersection_1_left_turn_command_{"3"};
  double slot_staging_ratio_{0.75};
  bool slot_a_staging_override_{true};
  double slot_a_staging_x_{0.8180313110351562};
  double slot_a_staging_y_{0.2924773097038269};
  double slot_intersection_tolerance_{0.25};
  double slot_staging_tolerance_{0.25};
  double slot_yaw_tolerance_{0.16};
  double slot_drive_heading_limit_{0.35};
  int slot_forward_pwm_{80};
  int slot_turn_pwm_{95};

  std::optional<Pose2D> leader_pose_;
  std::optional<Pose2D> follower_pose_;
  std::vector<LeaderPoseSample> leader_history_;
  std::unordered_map<std::string, Pose2D> topology_nodes_;
  std::vector<Pose2D> topology_path_;
  size_t leader_path_progress_index_{0};
  size_t topology_path_progress_index_{0};
  double leader_pose_speed_{0.0};
  double leader_cmd_speed_{0.0};
  Pose2D odom_pose_;
  bool odom_initialized_{false};
  std::optional<long> last_left_ticks_;
  std::optional<long> last_right_ticks_;
  int pending_encoder_baseline_samples_{0};
  rclcpp::Time last_leader_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_leader_cmd_vel_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_follower_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time follow_mode_started_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_sent_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_encoder_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_encoder_motion_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_integrated_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odom_jump_guard_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time intersection_1_left_turn_end_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time intersection_1_left_turn_requested_at_{0, 0, RCL_ROS_TIME};
  bool intersection_1_left_turn_pending_{false};
  bool intersection_1_left_turn_active_{false};
  bool intersection_1_left_turn_completed_{false};
  SlotWaitTask slot_wait_task_{SlotWaitTask::None};
  SlotWaitStage slot_wait_stage_{SlotWaitStage::DriveToIntersection};
  bool slot_wait_arrival_published_{false};
  PwmCommand last_sent_pwm_command_;
  double encoder_balance_trim_pwm_{0.0};
  std::string last_command_;
  std::string serial_rx_buffer_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr encoder_ticks_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_debug_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr slot_wait_status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr leader_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    leader_pose_cov_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr leader_cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr follower_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::TimerBase::SharedPtr serial_retry_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RcCarFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
