#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
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
  Align
};

}  // namespace

class RcCarFollowerNode : public rclcpp::Node
{
public:
  RcCarFollowerNode()
  : Node("rc_car_follower_node")
  {
    this->declare_parameter<std::string>("leader_pose_topic", "/turtlebot/pose");
    this->declare_parameter<std::string>("follower_odom_topic", "/rc_car/odom");
    this->declare_parameter<std::string>("mode_topic", "/rc_car/follower_mode");
    this->declare_parameter<std::string>("arduino_port", "/dev/ttyACM0");
    this->declare_parameter<int>("arduino_baud_rate", 115200);
    this->declare_parameter<double>("target_distance", 0.70);
    this->declare_parameter<double>("distance_deadband", 0.08);
    this->declare_parameter<double>("too_close_distance", 0.45);
    this->declare_parameter<double>("heading_deadband", 0.18);
    this->declare_parameter<double>("pose_timeout_seconds", 0.70);
    this->declare_parameter<double>("control_rate_hz", 10.0);
    this->declare_parameter<double>("drive_duty_cycle", 0.90);
    this->declare_parameter<double>("drive_cycle_seconds", 0.20);
    this->declare_parameter<bool>("start_in_follow_mode", false);
    this->declare_parameter<bool>("publish_encoder_odom", true);
    this->declare_parameter<bool>("auto_initialize_odom_from_leader", true);
    this->declare_parameter<std::string>("odom_frame_id", "map");
    this->declare_parameter<std::string>("base_frame_id", "rc_car_base_link");
    this->declare_parameter<double>("initial_follow_distance", 0.70);
    this->declare_parameter<double>("wheel_base", 0.16);
    this->declare_parameter<double>("left_meters_per_tick", 0.0005);
    this->declare_parameter<double>("right_meters_per_tick", 0.0005);
    this->declare_parameter<double>("odom_publish_rate_hz", 20.0);

    target_distance_ = this->get_parameter("target_distance").as_double();
    distance_deadband_ = this->get_parameter("distance_deadband").as_double();
    too_close_distance_ = this->get_parameter("too_close_distance").as_double();
    heading_deadband_ = this->get_parameter("heading_deadband").as_double();
    pose_timeout_seconds_ = this->get_parameter("pose_timeout_seconds").as_double();
    drive_duty_cycle_ = std::clamp(
      this->get_parameter("drive_duty_cycle").as_double(), 0.10, 1.0);
    drive_cycle_seconds_ = std::max(
      0.20, this->get_parameter("drive_cycle_seconds").as_double());
    publish_encoder_odom_ = this->get_parameter("publish_encoder_odom").as_bool();
    auto_initialize_odom_from_leader_ =
      this->get_parameter("auto_initialize_odom_from_leader").as_bool();
    odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    initial_follow_distance_ = this->get_parameter("initial_follow_distance").as_double();
    wheel_base_ = this->get_parameter("wheel_base").as_double();
    left_meters_per_tick_ = this->get_parameter("left_meters_per_tick").as_double();
    right_meters_per_tick_ = this->get_parameter("right_meters_per_tick").as_double();
    mode_ = this->get_parameter("start_in_follow_mode").as_bool() ?
      FollowerMode::Follow : FollowerMode::Stop;

    const auto leader_pose_topic = this->get_parameter("leader_pose_topic").as_string();
    const auto follower_odom_topic = this->get_parameter("follower_odom_topic").as_string();
    const auto mode_topic = this->get_parameter("mode_topic").as_string();
    arduino_port_ = this->get_parameter("arduino_port").as_string();
    arduino_baud_rate_ = this->get_parameter("arduino_baud_rate").as_int();

    leader_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      leader_pose_topic,
      10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        Pose2D pose;
        pose.x = msg->pose.position.x;
        pose.y = msg->pose.position.y;
        pose.yaw = yaw_from_quaternion(msg->pose.orientation);
        leader_pose_ = pose;
        last_leader_stamp_ = this->now();
      });

    follower_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      follower_odom_topic,
      10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        Pose2D pose;
        pose.x = msg->pose.pose.position.x;
        pose.y = msg->pose.pose.position.y;
        pose.yaw = yaw_from_quaternion(msg->pose.pose.orientation);
        follower_pose_ = pose;
        last_follower_stamp_ = this->now();
      });

    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      mode_topic,
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        set_mode(msg->data);
      });

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(follower_odom_topic, 10);

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
    send_command("7");
  }

private:
  void try_open_serial_port()
  {
    const std::string port = resolve_serial_port();
    if (!serial_.open_port(port, arduino_baud_rate_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Failed to open Arduino serial port %s: %s",
        port.c_str(),
        serial_.last_error().c_str());
      return;
    }

    last_command_.clear();
    RCLCPP_INFO(
      this->get_logger(),
      "Arduino serial connected: %s @ %d",
      port.c_str(),
      arduino_baud_rate_);
  }

  std::string resolve_serial_port() const
  {
    if (std::filesystem::exists(arduino_port_)) {
      return arduino_port_;
    }

    const std::filesystem::path by_id_dir{"/dev/serial/by-id"};
    if (std::filesystem::exists(by_id_dir)) {
      for (const auto & entry : std::filesystem::directory_iterator(by_id_dir)) {
        const std::string path = entry.path().string();
        if (path.find("Arduino") != std::string::npos || path.find("arduino") != std::string::npos) {
          return path;
        }
      }
    }

    for (const auto & prefix : {"/dev/ttyACM", "/dev/ttyUSB"}) {
      for (int i = 0; i < 4; ++i) {
        const std::string candidate = std::string(prefix) + std::to_string(i);
        if (std::filesystem::exists(candidate)) {
          return candidate;
        }
      }
    }

    return arduino_port_;
  }

  void set_mode(const std::string & mode)
  {
    const FollowerMode previous_mode = mode_;
    if (mode == "follow") {
      mode_ = FollowerMode::Follow;
    } else if (mode == "return") {
      mode_ = FollowerMode::Return;
    } else if (mode == "align") {
      mode_ = FollowerMode::Align;
    } else {
      mode_ = FollowerMode::Stop;
      send_command("7");
    }

    if (mode_ != previous_mode) {
      RCLCPP_INFO(this->get_logger(), "Mode changed: %s", mode.c_str());
    }
  }

  void control_step()
  {
    if (mode_ == FollowerMode::Stop) {
      send_command("7");
      return;
    }

    if (!poses_are_fresh()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Waiting for fresh leader/follower poses");
      send_command("7");
      return;
    }

    const Pose2D leader = leader_pose_.value();
    const Pose2D follower = follower_pose_.value();
    const double dx = leader.x - follower.x;
    const double dy = leader.y - follower.y;
    const double distance = std::hypot(dx, dy);
    const double target_heading = std::atan2(dy, dx);
    const double heading_error = normalize_angle(target_heading - follower.yaw);

    std::string command = "7";
    if (mode_ == FollowerMode::Align) {
      command = make_align_command(heading_error);
    } else if (mode_ == FollowerMode::Return) {
      command = make_return_command(distance, heading_error);
    } else {
      command = make_follow_command(distance, heading_error);
    }

    send_command(apply_drive_duty(command));
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
      long left_ticks = 0;
      long right_ticks = 0;
      if (parse_tick_line(line, left_ticks, right_ticks)) {
        update_encoder_odom(left_ticks, right_ticks);
      }
    }

    publish_current_odom();
  }

  bool parse_tick_line(const std::string & line, long & left_ticks, long & right_ticks) const
  {
    const auto left_pos = line.find("L_Tick:");
    const auto right_pos = line.find("R_Tick:");
    if (left_pos == std::string::npos || right_pos == std::string::npos) {
      return false;
    }

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

  void initialize_odom_from_leader()
  {
    if (!auto_initialize_odom_from_leader_ || odom_initialized_ || !leader_pose_.has_value()) {
      return;
    }

    const Pose2D leader = leader_pose_.value();
    odom_pose_.x = leader.x - std::cos(leader.yaw) * initial_follow_distance_;
    odom_pose_.y = leader.y - std::sin(leader.yaw) * initial_follow_distance_;
    odom_pose_.yaw = leader.yaw;
    follower_pose_ = odom_pose_;
    last_follower_stamp_ = this->now();
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
      return;
    }

    const long delta_left_ticks = left_ticks - last_left_ticks_.value();
    const long delta_right_ticks = right_ticks - last_right_ticks_.value();
    last_left_ticks_ = left_ticks;
    last_right_ticks_ = right_ticks;

    const double delta_left = static_cast<double>(delta_left_ticks) * left_meters_per_tick_;
    const double delta_right = static_cast<double>(delta_right_ticks) * right_meters_per_tick_;
    const double delta_distance = 0.5 * (delta_left + delta_right);
    const double delta_yaw = (delta_right - delta_left) / wheel_base_;
    const double mid_yaw = odom_pose_.yaw + 0.5 * delta_yaw;

    odom_pose_.x += delta_distance * std::cos(mid_yaw);
    odom_pose_.y += delta_distance * std::sin(mid_yaw);
    odom_pose_.yaw = normalize_angle(odom_pose_.yaw + delta_yaw);
    follower_pose_ = odom_pose_;
    last_follower_stamp_ = this->now();
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

  std::string make_follow_command(double distance, double heading_error) const
  {
    if (distance < too_close_distance_) {
      if (heading_error > heading_deadband_) {
        return "6";
      }
      if (heading_error < -heading_deadband_) {
        return "5";
      }
      return "2";
    }

    if (std::abs(distance - target_distance_) <= distance_deadband_) {
      return "7";
    }

    if (heading_error > heading_deadband_) {
      return "3";
    }
    if (heading_error < -heading_deadband_) {
      return "4";
    }
    return "1";
  }

  std::string make_return_command(double distance, double heading_error) const
  {
    if (distance <= target_distance_) {
      return "7";
    }
    if (heading_error > heading_deadband_) {
      return "3";
    }
    if (heading_error < -heading_deadband_) {
      return "4";
    }
    return "1";
  }

  std::string make_align_command(double heading_error) const
  {
    if (std::abs(heading_error) <= heading_deadband_) {
      return "7";
    }
    return heading_error > 0.0 ? "3" : "4";
  }

  std::string apply_drive_duty(const std::string & command) const
  {
    if (command == "7" || drive_duty_cycle_ >= 0.999) {
      return command;
    }

    const double cycle_position =
      std::fmod(this->now().seconds(), drive_cycle_seconds_) / drive_cycle_seconds_;
    return cycle_position < drive_duty_cycle_ ? command : "7";
  }

  void send_command(const std::string & command)
  {
    if (command == last_command_) {
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
    RCLCPP_DEBUG(this->get_logger(), "Arduino command: %s", command.c_str());
  }

  SerialPort serial_;
  FollowerMode mode_{FollowerMode::Stop};

  std::string arduino_port_{"/dev/ttyACM0"};
  int arduino_baud_rate_{115200};
  double target_distance_{0.70};
  double distance_deadband_{0.08};
  double too_close_distance_{0.45};
  double heading_deadband_{0.18};
  double pose_timeout_seconds_{0.70};
  double drive_duty_cycle_{0.90};
  double drive_cycle_seconds_{0.20};
  bool publish_encoder_odom_{true};
  bool auto_initialize_odom_from_leader_{true};
  std::string odom_frame_id_{"map"};
  std::string base_frame_id_{"rc_car_base_link"};
  double initial_follow_distance_{0.70};
  double wheel_base_{0.16};
  double left_meters_per_tick_{0.0005};
  double right_meters_per_tick_{0.0005};

  std::optional<Pose2D> leader_pose_;
  std::optional<Pose2D> follower_pose_;
  Pose2D odom_pose_;
  bool odom_initialized_{false};
  std::optional<long> last_left_ticks_;
  std::optional<long> last_right_ticks_;
  rclcpp::Time last_leader_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_follower_stamp_{0, 0, RCL_ROS_TIME};
  std::string last_command_;
  std::string serial_rx_buffer_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr leader_pose_sub_;
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
