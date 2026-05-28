#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

struct MissionNode
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct RobotPose2D
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

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

class MissionLoop : public rclcpp::Node
{
public:
  MissionLoop()
  : Node("mission_loop"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    const auto default_topology =
      ament_index_cpp::get_package_share_directory("amr_topology") + "/config/topology.yaml";

    this->declare_parameter<std::string>("topology_file", default_topology);
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("base_frame", "base_footprint");
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<int>("repeat_count", 2);
    this->declare_parameter<double>("wait_seconds", 3.0);
    this->declare_parameter<double>("precision_wait_seconds", 5.0);
    this->declare_parameter<double>("goal_tolerance", 0.08);
    this->declare_parameter<double>("precision_goal_tolerance", 0.04);
    this->declare_parameter<double>("waypoint_tolerance", 0.22);
    this->declare_parameter<double>("yaw_tolerance", 0.08);
    this->declare_parameter<double>("max_linear_speed", 0.14);
    this->declare_parameter<double>("max_angular_speed", 0.45);
    this->declare_parameter<double>("linear_gain", 0.70);
    this->declare_parameter<double>("angular_gain", 1.1);
    this->declare_parameter<double>("drive_heading_limit", 1.75);
    this->declare_parameter<double>("curve_min_linear_speed", 0.03);
    this->declare_parameter<double>("slot_departure_linear_speed", 0.065);
    this->declare_parameter<bool>("enable_lidar_safety", true);
    this->declare_parameter<double>("front_stop_distance", 0.28);
    this->declare_parameter<double>("front_sector_angle", 0.70);
    this->declare_parameter<double>("rear_stop_distance", 0.18);
    this->declare_parameter<double>("rear_sector_angle", 0.70);
    this->declare_parameter<double>("dock_tolerance", 0.04);
    this->declare_parameter<double>("dock_lateral_tolerance", 0.08);
    this->declare_parameter<double>("dock_longitudinal_tolerance", 0.04);
    this->declare_parameter<double>("dock_max_reverse_speed", 0.055);
    this->declare_parameter<double>("dock_max_reverse_angular_speed", 0.22);
    this->declare_parameter<double>("dock_reverse_start_distance", 0.75);
    this->declare_parameter<double>("dock_parking_lateral_offset", 0.15);
    this->declare_parameter<double>("dock_linear_gain", 0.30);
    this->declare_parameter<double>("dock_angular_gain", 0.8);
    this->declare_parameter<bool>("enable_parking_prep", true);
    this->declare_parameter<int>("l_parking_iterations", 0);
    this->declare_parameter<double>("parking_prep_duration", 1.4);
    this->declare_parameter<double>("parking_prep_linear_speed", 0.045);
    this->declare_parameter<double>("parking_prep_angular_speed", 0.22);
    this->declare_parameter<double>("side_sector_angle", 0.70);

    topology_file_ = this->get_parameter("topology_file").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    repeat_count_ = this->get_parameter("repeat_count").as_int();
    wait_seconds_ = this->get_parameter("wait_seconds").as_double();
    precision_wait_seconds_ = this->get_parameter("precision_wait_seconds").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    precision_goal_tolerance_ = this->get_parameter("precision_goal_tolerance").as_double();
    waypoint_tolerance_ = this->get_parameter("waypoint_tolerance").as_double();
    yaw_tolerance_ = this->get_parameter("yaw_tolerance").as_double();
    max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
    max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
    linear_gain_ = this->get_parameter("linear_gain").as_double();
    angular_gain_ = this->get_parameter("angular_gain").as_double();
    drive_heading_limit_ = this->get_parameter("drive_heading_limit").as_double();
    curve_min_linear_speed_ = this->get_parameter("curve_min_linear_speed").as_double();
    slot_departure_linear_speed_ = this->get_parameter("slot_departure_linear_speed").as_double();
    enable_lidar_safety_ = this->get_parameter("enable_lidar_safety").as_bool();
    front_stop_distance_ = this->get_parameter("front_stop_distance").as_double();
    front_sector_angle_ = this->get_parameter("front_sector_angle").as_double();
    rear_stop_distance_ = this->get_parameter("rear_stop_distance").as_double();
    rear_sector_angle_ = this->get_parameter("rear_sector_angle").as_double();
    dock_tolerance_ = this->get_parameter("dock_tolerance").as_double();
    dock_lateral_tolerance_ = this->get_parameter("dock_lateral_tolerance").as_double();
    dock_longitudinal_tolerance_ = this->get_parameter("dock_longitudinal_tolerance").as_double();
    dock_max_reverse_speed_ = this->get_parameter("dock_max_reverse_speed").as_double();
    dock_max_reverse_angular_speed_ =
      this->get_parameter("dock_max_reverse_angular_speed").as_double();
    dock_reverse_start_distance_ = this->get_parameter("dock_reverse_start_distance").as_double();
    dock_parking_lateral_offset_ = this->get_parameter("dock_parking_lateral_offset").as_double();
    dock_linear_gain_ = this->get_parameter("dock_linear_gain").as_double();
    dock_angular_gain_ = this->get_parameter("dock_angular_gain").as_double();
    enable_parking_prep_ = this->get_parameter("enable_parking_prep").as_bool();
    l_parking_iterations_ = this->get_parameter("l_parking_iterations").as_int();
    parking_prep_duration_ = this->get_parameter("parking_prep_duration").as_double();
    parking_prep_linear_speed_ = this->get_parameter("parking_prep_linear_speed").as_double();
    parking_prep_angular_speed_ = this->get_parameter("parking_prep_angular_speed").as_double();
    side_sector_angle_ = this->get_parameter("side_sector_angle").as_double();

    load_topology();

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      this->get_parameter("cmd_vel_topic").as_string(), 10);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      this->get_parameter("scan_topic").as_string(),
      rclcpp::SensorDataQoS(),
      std::bind(&MissionLoop::scan_callback, this, std::placeholders::_1));

    mission_command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "mission_command",
      10,
      std::bind(
        &MissionLoop::handle_mission_command,
        this,
        std::placeholders::_1));
  }

  void run()
  {
    RCLCPP_INFO(this->get_logger(), "Starting A/B return mission loop");

    for (int cycle = 1; cycle <= repeat_count_ && rclcpp::ok(); ++cycle) {
      RCLCPP_INFO(this->get_logger(), "Cycle %d/%d: moving to A", cycle, repeat_count_);
      go_path({"loading", "intersection_1", "a_entry", "a_leader_slot"});
      wait_at_slot("A", "a_entry");
      run_precision_slot_mission("A", "a_leader_slot_precision", "a_entry");
      return_to_loading({"a_leader_slot_precision", "a_entry", "intersection_1", "loading"});

      RCLCPP_INFO(this->get_logger(), "Cycle %d/%d: moving to B", cycle, repeat_count_);
      go_path({"loading", "intersection_2", "b_entry", "b_leader_slot"});
      wait_at_slot("B", "b_entry");
      run_precision_slot_mission("B", "b_leader_slot_precision", "b_entry");
      return_to_loading({"b_leader_slot_precision", "b_entry", "intersection_2", "loading"});
    }

    wait_for_charger_request();
  }

private:
  void go_path(const std::vector<std::string> & path)
  {
    if (path.empty()) {
      return;
    }

    size_t target_index = 0;
    rclcpp::Rate rate(20.0);

    while (rclcpp::ok() && target_index < path.size()) {
      rclcpp::spin_some(this->get_node_base_interface());
      const auto pose = lookup_robot_pose();
      if (!pose.has_value()) {
        rate.sleep();
        continue;
      }

      const bool is_final = target_index == path.size() - 1;
      const auto source_name = target_index > 0 ? path[target_index - 1] : std::string{};
      const auto & target_name = path[target_index];
      const auto & target = nodes_.at(target_name);
      const double distance = distance_to_target(pose.value(), target);
      const double tolerance = is_final ? goal_tolerance_ : waypoint_tolerance_;

      if (distance <= tolerance) {
        if (is_final) {
          stop();
          if (target_name != "charger_entry") {
            rotate_to_node_yaw(target_name);
          }
          RCLCPP_INFO(this->get_logger(), "Reached %s", target_name.c_str());
          return;
        }

        RCLCPP_INFO(this->get_logger(), "Passed waypoint %s", target_name.c_str());
        ++target_index;
        continue;
      }

      const bool leaving_leader_slot =
        is_leader_slot(source_name) && is_entry_node(target_name);
      auto command = leaving_leader_slot ?
        make_slot_departure_command() :
        make_drive_command(pose.value(), target, is_final);
      cmd_pub_->publish(apply_lidar_safety(command));
      rate.sleep();
    }
  }

  geometry_msgs::msg::Twist make_drive_command(
    const RobotPose2D & pose,
    const MissionNode & target,
    bool is_final)
  {
    const double dx = target.x - pose.x;
    const double dy = target.y - pose.y;
    const double distance = std::hypot(dx, dy);
    const double target_heading = std::atan2(dy, dx);
    const double heading_error = normalize_angle(target_heading - pose.yaw);

    geometry_msgs::msg::Twist command;
    command.angular.z = clamp(
      angular_gain_ * heading_error,
      -max_angular_speed_,
      max_angular_speed_);

    const double speed_scale = std::abs(heading_error) <= drive_heading_limit_ ?
      std::max(0.25, std::cos(heading_error)) : 0.25;
    const double min_speed = is_final && distance < 0.25 ? 0.015 : curve_min_linear_speed_;
    command.linear.x = clamp(
      linear_gain_ * distance * speed_scale,
      min_speed,
      max_linear_speed_);

    return command;
  }

  geometry_msgs::msg::Twist make_slot_departure_command() const
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = slot_departure_linear_speed_;
    return command;
  }

  bool is_leader_slot(const std::string & node_name) const
  {
    return node_name == "a_leader_slot" || node_name == "b_leader_slot" ||
      node_name == "a_leader_slot_precision" || node_name == "b_leader_slot_precision";
  }

  bool is_entry_node(const std::string & node_name) const
  {
    return node_name == "a_entry" || node_name == "b_entry";
  }

  void return_to_loading(const std::vector<std::string> & path)
  {
    go_path(path);
    wait_stopped("loading", precision_wait_seconds_);
  }

  void wait_for_charger_request()
  {
    stop();
    RCLCPP_INFO(
      this->get_logger(),
      "Mission loop complete. Waiting at loading for /mission_command start_charger_parking.");

    rclcpp::Rate rate(10.0);
    while (rclcpp::ok() && !charger_parking_requested_) {
      rclcpp::spin_some(this->get_node_base_interface());
      stop();
      rate.sleep();
    }

    if (!rclcpp::ok()) {
      return;
    }

    charger_parking_requested_ = false;
    RCLCPP_INFO(this->get_logger(), "Charger parking requested. Moving to charger_entry");
    go_path({"loading", "charger_entry"});
    l_shaped_charger_parking("charger_front");
    stop();
    RCLCPP_INFO(this->get_logger(), "Charger parking complete");
  }

  void handle_mission_command(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "start_charger_parking") {
      charger_parking_requested_ = true;
      RCLCPP_INFO(this->get_logger(), "Received charger parking command");
    }
  }

  void run_precision_slot_mission(
    const std::string & slot_name,
    const std::string & precision_node_name,
    const std::string & exit_node_name)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Moving to %s precision slot %s",
      slot_name.c_str(),
      precision_node_name.c_str());

    go_to_precision_slot(precision_node_name);
    rotate_to_node_yaw(precision_node_name);
    wait_stopped(slot_name + " precision slot", precision_wait_seconds_);
    rotate_to_face(exit_node_name);
  }

  void go_to_precision_slot(const std::string & target_node_name)
  {
    const auto & target = nodes_.at(target_node_name);
    rclcpp::Rate rate(20.0);

    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      const auto pose = lookup_robot_pose();
      if (!pose.has_value()) {
        rate.sleep();
        continue;
      }

      const double distance = distance_to_target(pose.value(), target);
      if (distance <= precision_goal_tolerance_) {
        stop();
        RCLCPP_INFO(this->get_logger(), "Reached %s precisely", target_node_name.c_str());
        return;
      }

      cmd_pub_->publish(make_drive_command(pose.value(), target, true));
      rate.sleep();
    }
  }

  void reverse_dock(const std::string & dock_node_name)
  {
    const auto & dock_target = nodes_.at(dock_node_name);
    RCLCPP_INFO(this->get_logger(), "Reverse docking to %s", dock_node_name.c_str());

    rclcpp::Rate rate(20.0);
    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      const auto pose = lookup_robot_pose();
      if (!pose.has_value()) {
        rate.sleep();
        continue;
      }

      const double dx = dock_target.x - pose->x;
      const double dy = dock_target.y - pose->y;
      const double distance = std::hypot(dx, dy);
      if (distance <= dock_tolerance_) {
        stop();
        RCLCPP_INFO(this->get_logger(), "Docking target reached");
        return;
      }

      const double rear_target_heading = std::atan2(dy, dx);
      const double robot_target_yaw = normalize_angle(rear_target_heading + M_PI);
      const double yaw_error = normalize_angle(robot_target_yaw - pose->yaw);

      if (std::abs(yaw_error) > yaw_tolerance_) {
        geometry_msgs::msg::Twist rotate_command;
        rotate_command.angular.z = clamp(
          dock_angular_gain_ * yaw_error,
          -max_angular_speed_,
          max_angular_speed_);
        cmd_pub_->publish(rotate_command);
        rate.sleep();
        continue;
      }

      geometry_msgs::msg::Twist command;
      command.linear.x = -clamp(
        dock_linear_gain_ * distance,
        0.015,
        dock_max_reverse_speed_);
      command.angular.z = clamp(
        dock_angular_gain_ * yaw_error,
        -max_angular_speed_,
        max_angular_speed_);

      cmd_pub_->publish(apply_rear_lidar_safety(command));
      rate.sleep();
    }
  }

  void l_shaped_charger_parking(const std::string & dock_node_name)
  {
    RCLCPP_INFO(this->get_logger(), "Starting L-shaped charger parking");

    const auto & dock_target = nodes_.at(dock_node_name);
    const double cos_yaw = std::cos(dock_target.yaw);
    const double sin_yaw = std::sin(dock_target.yaw);

    MissionNode reverse_start;
    reverse_start.x = dock_target.x + cos_yaw * dock_reverse_start_distance_;
    reverse_start.y = dock_target.y + sin_yaw * dock_reverse_start_distance_;
    reverse_start.yaw = dock_target.yaw;

    MissionNode parking_corner = reverse_start;
    parking_corner.x += -sin_yaw * dock_parking_lateral_offset_;
    parking_corner.y += cos_yaw * dock_parking_lateral_offset_;

    const auto pose = lookup_robot_pose();
    if (!pose.has_value()) {
      return;
    }
    parking_corner.yaw = std::atan2(parking_corner.y - pose->y, parking_corner.x - pose->x);

    rotate_to_yaw(parking_corner.yaw, "charger_parking_roof");
    drive_forward_straight_to_pose(parking_corner, "charger_parking_corner");
    reverse_arc_to_pose(reverse_start, dock_target.yaw, "charger_reverse_start");
    reverse_dock_to_pose(dock_node_name);
  }

  void drive_forward_straight_to_pose(
    const MissionNode & target,
    const std::string & label)
  {
    RCLCPP_INFO(this->get_logger(), "Moving to %s", label.c_str());

    rclcpp::Rate rate(20.0);
    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      const auto pose = lookup_robot_pose();
      if (!pose.has_value()) {
        rate.sleep();
        continue;
      }

      const double distance = distance_to_target(pose.value(), target);
      if (distance <= goal_tolerance_) {
        stop();
        RCLCPP_INFO(this->get_logger(), "Reached %s", label.c_str());
        return;
      }

      const double heading = std::atan2(target.y - pose->y, target.x - pose->x);
      const double heading_error = normalize_angle(heading - pose->yaw);

      geometry_msgs::msg::Twist command;
      command.linear.x = clamp(linear_gain_ * distance, 0.025, 0.08);
      command.angular.z = clamp(
        angular_gain_ * heading_error,
        -0.18,
        0.18);

      cmd_pub_->publish(apply_lidar_safety(command));
      rate.sleep();
    }
  }

  void reverse_arc_to_pose(
    const MissionNode & target,
    double final_yaw,
    const std::string & label)
  {
    RCLCPP_INFO(this->get_logger(), "Reverse arc to %s", label.c_str());

    rclcpp::Rate rate(20.0);
    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      const auto pose = lookup_robot_pose();
      if (!pose.has_value()) {
        rate.sleep();
        continue;
      }

      const double distance = distance_to_target(pose.value(), target);
      const double yaw_error = normalize_angle(final_yaw - pose->yaw);
      if (distance <= goal_tolerance_) {
        stop();
        RCLCPP_INFO(this->get_logger(), "Reached %s", label.c_str());
        return;
      }

      const double heading = std::atan2(target.y - pose->y, target.x - pose->x);
      const double rear_heading_error = normalize_angle(heading + M_PI - pose->yaw);

      geometry_msgs::msg::Twist command;
      command.linear.x = -clamp(linear_gain_ * distance, 0.025, dock_max_reverse_speed_);
      command.angular.z = clamp(
        0.30 * rear_heading_error + 0.15 * yaw_error,
        -0.12,
        0.12);

      cmd_pub_->publish(apply_rear_lidar_safety(command));
      rate.sleep();
    }
  }

  void reverse_dock_to_pose(const std::string & dock_node_name)
  {
    const auto & dock_target = nodes_.at(dock_node_name);
    RCLCPP_INFO(this->get_logger(), "Reverse docking straight to %s", dock_node_name.c_str());

    rclcpp::Rate rate(20.0);
    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      const auto pose = lookup_robot_pose();
      if (!pose.has_value()) {
        rate.sleep();
        continue;
      }

      const double dx = dock_target.x - pose->x;
      const double dy = dock_target.y - pose->y;
      const double cos_yaw = std::cos(dock_target.yaw);
      const double sin_yaw = std::sin(dock_target.yaw);
      const double longitudinal_error = dx * cos_yaw + dy * sin_yaw;
      const double reverse_remaining = -longitudinal_error;
      const double lateral_error = dx * -sin_yaw + dy * cos_yaw;
      const double yaw_error = normalize_angle(dock_target.yaw - pose->yaw);

      if (
        reverse_remaining <= dock_longitudinal_tolerance_ &&
        std::abs(lateral_error) <= dock_lateral_tolerance_ &&
        std::abs(yaw_error) <= yaw_tolerance_)
      {
        stop();
        RCLCPP_INFO(this->get_logger(), "Docking target reached");
        return;
      }

      if (reverse_remaining <= 0.0) {
        stop();
        RCLCPP_WARN(
          this->get_logger(),
          "Docking target passed. Stop reverse. longitudinal=%.3f lateral=%.3f yaw_error=%.3f",
          reverse_remaining,
          lateral_error,
          yaw_error);
        return;
      }

      geometry_msgs::msg::Twist command;
      command.linear.x = -clamp(
        dock_linear_gain_ * reverse_remaining,
        0.012,
        dock_max_reverse_speed_);
      command.angular.z = clamp(
        dock_angular_gain_ * yaw_error,
        -dock_max_reverse_angular_speed_,
        dock_max_reverse_angular_speed_);

      cmd_pub_->publish(apply_rear_lidar_safety(command));
      rate.sleep();
    }
  }

  int choose_parking_turn_sign() const
  {
    if (std::isfinite(left_avg_range_) && std::isfinite(right_avg_range_)) {
      return left_avg_range_ >= right_avg_range_ ? 1 : -1;
    }
    if (std::isfinite(left_avg_range_)) {
      return 1;
    }
    if (std::isfinite(right_avg_range_)) {
      return -1;
    }
    return 1;
  }

  void publish_timed_motion(
    double linear_x,
    double angular_z,
    double duration,
    bool forward_safety)
  {
    const auto end_time = this->now() + rclcpp::Duration::from_seconds(duration);
    rclcpp::Rate rate(20.0);
    while (rclcpp::ok() && this->now() < end_time) {
      rclcpp::spin_some(this->get_node_base_interface());
      geometry_msgs::msg::Twist command;
      command.linear.x = linear_x;
      command.angular.z = angular_z;
      cmd_pub_->publish(forward_safety ? apply_lidar_safety(command) : apply_rear_lidar_safety(command));
      rate.sleep();
    }

    stop();
    rclcpp::sleep_for(200ms);
  }

  bool is_dock_reached(const std::string & dock_node_name)
  {
    const auto pose = lookup_robot_pose();
    if (!pose.has_value()) {
      return false;
    }

    const auto & dock_target = nodes_.at(dock_node_name);
    const double yaw_error = normalize_angle(dock_target.yaw - pose->yaw);
    if (
      distance_to_target(pose.value(), dock_target) > dock_tolerance_ ||
      std::abs(yaw_error) > yaw_tolerance_)
    {
      return false;
    }

    stop();
    RCLCPP_INFO(this->get_logger(), "Docking target reached");
    return true;
  }

  void reverse_dock_with_steering(const std::string & dock_node_name)
  {
    const auto & dock_target = nodes_.at(dock_node_name);
    RCLCPP_INFO(this->get_logger(), "Final reverse parking with steering to %s", dock_node_name.c_str());

    rclcpp::Rate rate(20.0);
    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      const auto pose = lookup_robot_pose();
      if (!pose.has_value()) {
        rate.sleep();
        continue;
      }

      const double dx = dock_target.x - pose->x;
      const double dy = dock_target.y - pose->y;
      const double distance = std::hypot(dx, dy);
      const double yaw_error = normalize_angle(dock_target.yaw - pose->yaw);
      if (distance <= dock_tolerance_ && std::abs(yaw_error) <= yaw_tolerance_) {
        stop();
        RCLCPP_INFO(this->get_logger(), "Docking target reached");
        return;
      }

      geometry_msgs::msg::Twist command;
      command.linear.x = -clamp(
        dock_linear_gain_ * distance,
        0.015,
        dock_max_reverse_speed_);
      command.angular.z = clamp(
        dock_angular_gain_ * yaw_error,
        -dock_max_reverse_angular_speed_,
        dock_max_reverse_angular_speed_);

      cmd_pub_->publish(apply_rear_lidar_safety(command));
      rate.sleep();
    }
  }

  void prepare_reverse_parking()
  {
    if (!enable_parking_prep_) {
      return;
    }

    const double left_clearance = left_avg_range_;
    const double right_clearance = right_avg_range_;
    int turn_sign = 1;
    if (std::isfinite(left_clearance) && std::isfinite(right_clearance)) {
      turn_sign = left_clearance >= right_clearance ? 1 : -1;
      RCLCPP_INFO(
        this->get_logger(),
        "Parking prep: left %.2fm, right %.2fm, turning %s",
        left_clearance,
        right_clearance,
        turn_sign > 0 ? "left" : "right");
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Parking prep: side clearance unavailable. Using default left arc.");
    }

    const auto end_time = this->now() + rclcpp::Duration::from_seconds(parking_prep_duration_);
    rclcpp::Rate rate(20.0);
    while (rclcpp::ok() && this->now() < end_time) {
      rclcpp::spin_some(this->get_node_base_interface());
      geometry_msgs::msg::Twist command;
      command.linear.x = parking_prep_linear_speed_;
      command.angular.z = static_cast<double>(turn_sign) * parking_prep_angular_speed_;
      cmd_pub_->publish(apply_lidar_safety(command));
      rate.sleep();
    }

    stop();
    rclcpp::sleep_for(200ms);
  }

  double distance_to_target(const RobotPose2D & pose, const MissionNode & target) const
  {
    return std::hypot(target.x - pose.x, target.y - pose.y);
  }

  std::optional<RobotPose2D> lookup_robot_pose()
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      RobotPose2D pose;
      pose.x = transform.transform.translation.x;
      pose.y = transform.transform.translation.y;
      pose.yaw = yaw_from_quaternion(transform.transform.rotation);
      return pose;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for TF %s -> %s: %s",
        map_frame_.c_str(), base_frame_.c_str(), ex.what());
      return std::nullopt;
    }
  }

  geometry_msgs::msg::Twist apply_lidar_safety(geometry_msgs::msg::Twist command)
  {
    if (!enable_lidar_safety_ || command.linear.x <= 0.0) {
      return command;
    }

    if (!std::isfinite(front_min_range_)) {
      command.linear.x = 0.0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "No valid front laser range. Blocking forward motion.");
      return command;
    }

    if (front_min_range_ <= front_stop_distance_) {
      command.linear.x = 0.0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Front obstacle %.2fm <= %.2fm. Blocking forward motion.",
        front_min_range_, front_stop_distance_);
    }

    return command;
  }

  geometry_msgs::msg::Twist apply_rear_lidar_safety(geometry_msgs::msg::Twist command)
  {
    if (!enable_lidar_safety_ || command.linear.x >= 0.0) {
      return command;
    }

    if (!std::isfinite(rear_min_range_)) {
      command.linear.x = 0.0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "No valid rear laser range. Blocking reverse motion.");
      return command;
    }

    if (rear_min_range_ <= rear_stop_distance_) {
      command.linear.x = 0.0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Rear obstacle %.2fm <= %.2fm. Blocking reverse motion.",
        rear_min_range_, rear_stop_distance_);
    }

    return command;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double front_min_range = std::numeric_limits<double>::infinity();
    double rear_min_range = std::numeric_limits<double>::infinity();
    double left_sum = 0.0;
    double right_sum = 0.0;
    int left_count = 0;
    int right_count = 0;

    for (size_t index = 0; index < msg->ranges.size(); ++index) {
      const double angle = msg->angle_min + static_cast<double>(index) * msg->angle_increment;
      const double range = msg->ranges[index];
      if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max) {
        continue;
      }

      if (std::abs(angle) <= front_sector_angle_ * 0.5) {
        front_min_range = std::min(front_min_range, range);
      }

      const double rear_error = std::abs(normalize_angle(angle - M_PI));
      if (rear_error <= rear_sector_angle_ * 0.5) {
        rear_min_range = std::min(rear_min_range, range);
      }

      const double left_error = std::abs(normalize_angle(angle - M_PI_2));
      if (left_error <= side_sector_angle_ * 0.5) {
        left_sum += range;
        ++left_count;
      }

      const double right_error = std::abs(normalize_angle(angle + M_PI_2));
      if (right_error <= side_sector_angle_ * 0.5) {
        right_sum += range;
        ++right_count;
      }
    }

    front_min_range_ = front_min_range;
    rear_min_range_ = rear_min_range;
    left_avg_range_ = left_count > 0 ? left_sum / static_cast<double>(left_count) :
      std::numeric_limits<double>::infinity();
    right_avg_range_ = right_count > 0 ? right_sum / static_cast<double>(right_count) :
      std::numeric_limits<double>::infinity();
  }

  void wait_at_slot(
    const std::string & slot_name,
    const std::string & exit_node_name)
  {
    wait_stopped(slot_name, wait_seconds_);
    rotate_to_face(exit_node_name);
  }

  void wait_stopped(const std::string & label, double seconds)
  {
    stop();
    RCLCPP_INFO(this->get_logger(), "Waiting at %s for %.1f seconds", label.c_str(), seconds);
    const auto end_time = this->now() + rclcpp::Duration::from_seconds(seconds);
    while (rclcpp::ok() && this->now() < end_time) {
      rclcpp::spin_some(this->get_node_base_interface());
      stop();
      rclcpp::sleep_for(100ms);
    }
  }

  void rotate_to_face(const std::string & target_node_name)
  {
    const auto & target = nodes_.at(target_node_name);
    RCLCPP_INFO(this->get_logger(), "Rotating in place toward %s", target_node_name.c_str());

    rclcpp::Rate rate(20.0);
    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      const auto pose = lookup_robot_pose();
      if (!pose.has_value()) {
        rate.sleep();
        continue;
      }

      const double target_heading = std::atan2(target.y - pose->y, target.x - pose->x);
      const double yaw_error = normalize_angle(target_heading - pose->yaw);
      if (std::abs(yaw_error) <= yaw_tolerance_) {
        stop();
        return;
      }

      geometry_msgs::msg::Twist command;
      command.angular.z = clamp(
        angular_gain_ * yaw_error,
        -max_angular_speed_,
        max_angular_speed_);
      cmd_pub_->publish(command);
      rate.sleep();
    }
  }

  void rotate_to_node_yaw(const std::string & target_node_name)
  {
    const auto & target = nodes_.at(target_node_name);
    RCLCPP_INFO(this->get_logger(), "Rotating in place to %s yaw", target_node_name.c_str());
    rotate_to_yaw(target.yaw, target_node_name);
  }

  void rotate_to_yaw(double target_yaw, const std::string & label)
  {
    RCLCPP_INFO(this->get_logger(), "Rotating in place to %s yaw", label.c_str());

    rclcpp::Rate rate(20.0);
    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      const auto pose = lookup_robot_pose();
      if (!pose.has_value()) {
        rate.sleep();
        continue;
      }

      const double yaw_error = normalize_angle(target_yaw - pose->yaw);
      if (std::abs(yaw_error) <= yaw_tolerance_) {
        stop();
        return;
      }

      geometry_msgs::msg::Twist command;
      command.angular.z = clamp(
        angular_gain_ * yaw_error,
        -max_angular_speed_,
        max_angular_speed_);
      cmd_pub_->publish(command);
      rate.sleep();
    }
  }

  void load_topology()
  {
    const YAML::Node topology = YAML::LoadFile(topology_file_);
    const YAML::Node yaml_nodes = topology["nodes"];
    if (!yaml_nodes) {
      throw std::runtime_error("topology.yaml has no nodes section");
    }

    for (const auto & item : yaml_nodes) {
      const auto name = item.first.as<std::string>();
      const auto data = item.second;
      MissionNode node;
      node.x = data["x"].as<double>();
      node.y = data["y"].as<double>();
      if (data["yaw"]) {
        node.yaw = data["yaw"].as<double>();
      }
      nodes_[name] = node;
    }
  }

  void stop()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }

  std::string topology_file_;
  std::string map_frame_;
  std::string base_frame_;
  std::unordered_map<std::string, MissionNode> nodes_;

  int repeat_count_{2};
  double wait_seconds_{3.0};
  double precision_wait_seconds_{5.0};
  double goal_tolerance_{0.08};
  double precision_goal_tolerance_{0.04};
  double waypoint_tolerance_{0.22};
  double yaw_tolerance_{0.08};
  double max_linear_speed_{0.14};
  double max_angular_speed_{0.45};
  double linear_gain_{0.70};
  double angular_gain_{1.1};
  double drive_heading_limit_{1.75};
  double curve_min_linear_speed_{0.03};
  double slot_departure_linear_speed_{0.065};
  bool enable_lidar_safety_{true};
  double front_stop_distance_{0.28};
  double front_sector_angle_{0.70};
  double rear_stop_distance_{0.18};
  double rear_sector_angle_{0.70};
  double dock_tolerance_{0.04};
  double dock_lateral_tolerance_{0.08};
  double dock_longitudinal_tolerance_{0.04};
  double dock_max_reverse_speed_{0.055};
  double dock_max_reverse_angular_speed_{0.22};
  double dock_reverse_start_distance_{0.75};
  double dock_parking_lateral_offset_{0.15};
  double dock_linear_gain_{0.30};
  double dock_angular_gain_{0.8};
  bool enable_parking_prep_{true};
  int l_parking_iterations_{0};
  bool charger_parking_requested_{false};
  double parking_prep_duration_{1.4};
  double parking_prep_linear_speed_{0.045};
  double parking_prep_angular_speed_{0.22};
  double side_sector_angle_{0.70};
  double front_min_range_{std::numeric_limits<double>::infinity()};
  double rear_min_range_{std::numeric_limits<double>::infinity()};
  double left_avg_range_{std::numeric_limits<double>::infinity()};
  double right_avg_range_{std::numeric_limits<double>::infinity()};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_command_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionLoop>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
