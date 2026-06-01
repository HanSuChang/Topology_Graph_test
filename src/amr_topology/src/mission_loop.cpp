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
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include "amr_topology/obstacle_avoidance.hpp"

using namespace std::chrono_literals;
using amr_topology::angle_in_sector;
using amr_topology::clamp;
using amr_topology::DwaPlannerConfig;
using amr_topology::MissionNode;
using amr_topology::normalize_angle;
using amr_topology::RobotPose2D;
using amr_topology::ScanPoint2D;
using amr_topology::yaw_from_quaternion;

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
    this->declare_parameter<std::string>("map_topic", "/map");
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
    this->declare_parameter<double>("front_emergency_stop_distance", 0.54);
    this->declare_parameter<double>("front_stop_distance", 0.60);
    this->declare_parameter<double>("front_clear_distance", 0.90);
    this->declare_parameter<double>("forward_collision_sector_angle", M_PI_2);
    this->declare_parameter<double>("forward_collision_stop_distance", 0.46);
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
    this->declare_parameter<bool>("enable_obstacle_avoidance", true);
    this->declare_parameter<double>("obstacle_wait_seconds", 0.0);
    this->declare_parameter<int>("obstacle_avoidance_max_attempts", 0);
    this->declare_parameter<double>("obstacle_avoidance_trigger_distance", 0.62);
    this->declare_parameter<double>("dwa_max_duration", 6.0);
    this->declare_parameter<double>("dwa_min_duration", 0.7);
    this->declare_parameter<double>("dwa_sim_time", 3.0);
    this->declare_parameter<double>("dwa_sim_step", 0.10);
    this->declare_parameter<double>("dwa_min_linear_speed", 0.04);
    this->declare_parameter<double>("dwa_max_linear_speed", 0.095);
    this->declare_parameter<double>("dwa_max_angular_speed", 0.55);
    this->declare_parameter<int>("dwa_linear_samples", 4);
    this->declare_parameter<int>("dwa_angular_samples", 11);
    this->declare_parameter<double>("dwa_robot_radius", 0.18);
    this->declare_parameter<double>("dwa_lidar_x_offset", 0.08);
    this->declare_parameter<double>("dwa_safety_margin", 0.25);
    this->declare_parameter<double>("dwa_static_map_clearance", 0.21);
    this->declare_parameter<double>("dwa_obstacle_range", 1.6);
    this->declare_parameter<double>("dwa_goal_weight", 0.9);
    this->declare_parameter<double>("dwa_clearance_weight", 2.0);
    this->declare_parameter<double>("dwa_speed_weight", 1.8);
    this->declare_parameter<double>("dwa_front_clear_hold", 0.6);
    this->declare_parameter<double>("dwa_stuck_turn_speed", 0.38);
    this->declare_parameter<double>("arc_commit_duration", 1.4);
    this->declare_parameter<double>("dynamic_contact_stop_distance", 0.28);
    this->declare_parameter<double>("dynamic_contact_escape_distance", 0.22);
    this->declare_parameter<double>("dynamic_contact_escape_speed", 0.04);
    this->declare_parameter<double>("dynamic_contact_turn_hold", 1.8);
    this->declare_parameter<double>("dynamic_path_min_clearance", 0.36);
    this->declare_parameter<bool>("ignore_mapped_front_obstacles", true);
    this->declare_parameter<double>("map_obstacle_padding", 0.08);
    this->declare_parameter<int>("map_occupied_threshold", 50);

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
    front_emergency_stop_distance_ =
      this->get_parameter("front_emergency_stop_distance").as_double();
    front_stop_distance_ = this->get_parameter("front_stop_distance").as_double();
    front_clear_distance_ = this->get_parameter("front_clear_distance").as_double();
    forward_collision_sector_angle_ =
      this->get_parameter("forward_collision_sector_angle").as_double();
    forward_collision_stop_distance_ =
      this->get_parameter("forward_collision_stop_distance").as_double();
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
    enable_obstacle_avoidance_ = this->get_parameter("enable_obstacle_avoidance").as_bool();
    obstacle_wait_seconds_ = this->get_parameter("obstacle_wait_seconds").as_double();
    obstacle_avoidance_max_attempts_ =
      this->get_parameter("obstacle_avoidance_max_attempts").as_int();
    obstacle_avoidance_trigger_distance_ =
      this->get_parameter("obstacle_avoidance_trigger_distance").as_double();
    dwa_max_duration_ = this->get_parameter("dwa_max_duration").as_double();
    dwa_min_duration_ = this->get_parameter("dwa_min_duration").as_double();
    dwa_sim_time_ = this->get_parameter("dwa_sim_time").as_double();
    dwa_sim_step_ = this->get_parameter("dwa_sim_step").as_double();
    dwa_min_linear_speed_ = this->get_parameter("dwa_min_linear_speed").as_double();
    dwa_max_linear_speed_ = this->get_parameter("dwa_max_linear_speed").as_double();
    dwa_max_angular_speed_ = this->get_parameter("dwa_max_angular_speed").as_double();
    dwa_linear_samples_ = this->get_parameter("dwa_linear_samples").as_int();
    dwa_angular_samples_ = this->get_parameter("dwa_angular_samples").as_int();
    dwa_robot_radius_ = this->get_parameter("dwa_robot_radius").as_double();
    dwa_lidar_x_offset_ = this->get_parameter("dwa_lidar_x_offset").as_double();
    dwa_safety_margin_ = this->get_parameter("dwa_safety_margin").as_double();
    dwa_static_map_clearance_ = this->get_parameter("dwa_static_map_clearance").as_double();
    dwa_obstacle_range_ = this->get_parameter("dwa_obstacle_range").as_double();
    dwa_goal_weight_ = this->get_parameter("dwa_goal_weight").as_double();
    dwa_clearance_weight_ = this->get_parameter("dwa_clearance_weight").as_double();
    dwa_speed_weight_ = this->get_parameter("dwa_speed_weight").as_double();
    dwa_front_clear_hold_ = this->get_parameter("dwa_front_clear_hold").as_double();
    dwa_stuck_turn_speed_ = this->get_parameter("dwa_stuck_turn_speed").as_double();
    arc_commit_duration_ = this->get_parameter("arc_commit_duration").as_double();
    dynamic_contact_stop_distance_ =
      this->get_parameter("dynamic_contact_stop_distance").as_double();
    dynamic_contact_escape_distance_ =
      this->get_parameter("dynamic_contact_escape_distance").as_double();
    dynamic_contact_escape_speed_ =
      this->get_parameter("dynamic_contact_escape_speed").as_double();
    dynamic_contact_turn_hold_ =
      this->get_parameter("dynamic_contact_turn_hold").as_double();
    dynamic_path_min_clearance_ =
      this->get_parameter("dynamic_path_min_clearance").as_double();
    ignore_mapped_front_obstacles_ =
      this->get_parameter("ignore_mapped_front_obstacles").as_bool();
    map_obstacle_padding_ = this->get_parameter("map_obstacle_padding").as_double();
    map_occupied_threshold_ = this->get_parameter("map_occupied_threshold").as_int();
    update_dwa_planner_config();

    load_topology();

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      this->get_parameter("cmd_vel_topic").as_string(), 10);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      this->get_parameter("scan_topic").as_string(),
      rclcpp::SensorDataQoS(),
      std::bind(&MissionLoop::scan_callback, this, std::placeholders::_1));

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      this->get_parameter("map_topic").as_string(),
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&MissionLoop::map_callback, this, std::placeholders::_1));

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
  struct DynamicObstacleRisk
  {
    double x{0.0};
    double y{0.0};
    double distance{std::numeric_limits<double>::infinity()};
  };

  void go_path(const std::vector<std::string> & path)
  {
    if (path.empty()) {
      return;
    }

    size_t target_index = 0;
    rclcpp::Rate rate(20.0);
    std::optional<rclcpp::Time> blocked_since;
    int avoidance_attempts = 0;

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
      if (is_final && distance <= tolerance) {
        stop();
        if (target_name != "charger_entry") {
          rotate_to_node_yaw(target_name);
        }
        RCLCPP_INFO(this->get_logger(), "Reached %s", target_name.c_str());
        return;
      }

      if (!is_final && distance <= tolerance) {
        RCLCPP_INFO(this->get_logger(), "Passed waypoint %s", target_name.c_str());
        ++target_index;
        avoidance_attempts = 0;
        continue;
      }

      const bool leaving_leader_slot =
        is_leader_slot(source_name) && is_entry_node(target_name);
      auto command = leaving_leader_slot ?
        make_slot_departure_command() :
        make_drive_command(pose.value(), target, is_final);
      const bool route_obstructed =
        !leaving_leader_slot &&
        (is_front_obstructed() || is_drive_path_obstructed(command));

      if (route_obstructed) {
        stop();
        if (
          obstacle_avoidance_max_attempts_ > 0 &&
          avoidance_attempts >= obstacle_avoidance_max_attempts_)
        {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Front obstacle still blocks the route after %d avoidance attempt(s). Waiting for it to clear.",
            avoidance_attempts);
          rate.sleep();
          continue;
        }
        if (!blocked_since.has_value()) {
          blocked_since = this->now();
          RCLCPP_WARN(
            this->get_logger(),
            "Obstacle on route detected. Waiting %.1f seconds before avoidance.",
            obstacle_wait_seconds_);
        }
        if ((this->now() - blocked_since.value()).seconds() >= obstacle_wait_seconds_) {
          if (try_avoid_front_obstacle(avoidance_attempts + 1, target)) {
            avoidance_attempts = 0;
            blocked_since.reset();
          } else {
            ++avoidance_attempts;
            RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 2000,
              "Obstacle avoidance unavailable. Waiting for obstacle to clear.");
          }
        }
        rate.sleep();
        continue;
      }

      if (
        (blocked_since.has_value() || avoidance_attempts > 0) &&
        is_front_clear() &&
        !is_drive_path_obstructed(command))
      {
        RCLCPP_INFO(this->get_logger(), "Obstacle route cleared. Resuming mission path.");
        blocked_since.reset();
        avoidance_attempts = 0;
      }
      cmd_pub_->publish(apply_navigation_safety(command, pose.value()));
      rate.sleep();
    }
  }

  geometry_msgs::msg::Twist make_drive_command(
    const RobotPose2D & pose,
    const MissionNode & target,
    bool is_final) const
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

  void update_dwa_planner_config()
  {
    DwaPlannerConfig config;
    config.min_linear_speed = dwa_min_linear_speed_;
    config.max_linear_speed = dwa_max_linear_speed_;
    config.max_angular_speed = dwa_max_angular_speed_;
    config.linear_samples = dwa_linear_samples_;
    config.angular_samples = dwa_angular_samples_;
    config.robot_radius = dwa_robot_radius_;
    config.safety_margin = dwa_safety_margin_;
    config.static_map_clearance = dwa_static_map_clearance_;
    config.obstacle_range = dwa_obstacle_range_;
    config.goal_weight = dwa_goal_weight_;
    config.clearance_weight = dwa_clearance_weight_;
    config.speed_weight = dwa_speed_weight_;
    config.sim_time = dwa_sim_time_;
    config.sim_step = dwa_sim_step_;
    config.front_clear_distance = front_clear_distance_;
    config.front_stop_distance = front_stop_distance_;
    config.stuck_turn_speed = dwa_stuck_turn_speed_;
    config.arc_commit_duration = arc_commit_duration_;
    dwa_planner_.set_config(config);
  }

  bool is_front_blocked() const
  {
    return enable_lidar_safety_ &&
      std::isfinite(front_min_range_) &&
      front_min_range_ <= front_stop_distance_;
  }

  bool is_front_emergency_blocked() const
  {
    return enable_lidar_safety_ &&
      std::isfinite(front_min_range_) &&
      front_min_range_ <= front_emergency_stop_distance_;
  }

  bool is_forward_collision_blocked() const
  {
    return enable_lidar_safety_ &&
      std::isfinite(forward_collision_min_range_) &&
      forward_collision_min_range_ <= forward_collision_stop_distance_;
  }

  bool is_front_clear() const
  {
    return !enable_lidar_safety_ ||
      (std::isfinite(front_min_range_) && front_min_range_ >= front_clear_distance_);
  }

  bool is_front_obstructed() const
  {
    return enable_lidar_safety_ &&
      (
        is_front_emergency_blocked() ||
        is_forward_collision_blocked() ||
        (std::isfinite(front_min_range_) &&
        front_min_range_ <= obstacle_avoidance_trigger_distance_)
      );
  }

  bool is_mapped_obstacle(double x, double y) const
  {
    if (!latest_map_) {
      return false;
    }

    const auto & info = latest_map_->info;
    if (info.resolution <= 0.0) {
      return false;
    }

    const int map_x = static_cast<int>(
      std::floor((x - info.origin.position.x) / info.resolution));
    const int map_y = static_cast<int>(
      std::floor((y - info.origin.position.y) / info.resolution));
    if (
      map_x < 0 || map_y < 0 ||
      map_x >= static_cast<int>(info.width) ||
      map_y >= static_cast<int>(info.height))
    {
      return false;
    }

    const int padding_cells =
      std::max(0, static_cast<int>(std::ceil(map_obstacle_padding_ / info.resolution)));
    for (int dy = -padding_cells; dy <= padding_cells; ++dy) {
      for (int dx = -padding_cells; dx <= padding_cells; ++dx) {
        const int nx = map_x + dx;
        const int ny = map_y + dy;
        if (
          nx < 0 || ny < 0 ||
          nx >= static_cast<int>(info.width) ||
          ny >= static_cast<int>(info.height))
        {
          continue;
        }

        const auto index = static_cast<size_t>(ny) * info.width + static_cast<size_t>(nx);
        if (latest_map_->data[index] >= map_occupied_threshold_) {
          return true;
        }
      }
    }
    return false;
  }

  bool is_static_map_collision(double x, double y, double clearance) const
  {
    if (!latest_map_ || clearance <= 0.0) {
      return false;
    }

    const auto & info = latest_map_->info;
    if (info.resolution <= 0.0) {
      return false;
    }

    const int map_x = static_cast<int>(
      std::floor((x - info.origin.position.x) / info.resolution));
    const int map_y = static_cast<int>(
      std::floor((y - info.origin.position.y) / info.resolution));
    if (
      map_x < 0 || map_y < 0 ||
      map_x >= static_cast<int>(info.width) ||
      map_y >= static_cast<int>(info.height))
    {
      return false;
    }

    const int radius_cells =
      std::max(1, static_cast<int>(std::ceil(clearance / info.resolution)));
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const int nx = map_x + dx;
        const int ny = map_y + dy;
        if (
          nx < 0 || ny < 0 ||
          nx >= static_cast<int>(info.width) ||
          ny >= static_cast<int>(info.height))
        {
          continue;
        }

        const auto index = static_cast<size_t>(ny) * info.width + static_cast<size_t>(nx);
        if (latest_map_->data[index] < map_occupied_threshold_) {
          continue;
        }

        const double cell_x = info.origin.position.x +
          (static_cast<double>(nx) + 0.5) * info.resolution;
        const double cell_y = info.origin.position.y +
          (static_cast<double>(ny) + 0.5) * info.resolution;
        if (std::hypot(cell_x - x, cell_y - y) <= clearance) {
          return true;
        }
      }
    }

    return false;
  }

  bool should_ignore_mapped_scan_point(
    const RobotPose2D & pose,
    double angle,
    double range,
    bool include_lidar_offset) const
  {
    if (!ignore_mapped_front_obstacles_ || !latest_map_) {
      return false;
    }

    const double local_x =
      (include_lidar_offset ? dwa_lidar_x_offset_ : 0.0) + range * std::cos(angle);
    const double local_y = range * std::sin(angle);
    const double point_x = pose.x + std::cos(pose.yaw) * local_x -
      std::sin(pose.yaw) * local_y;
    const double point_y = pose.y + std::sin(pose.yaw) * local_x +
      std::cos(pose.yaw) * local_y;
    return is_mapped_obstacle(point_x, point_y);
  }

  int choose_avoidance_turn_sign() const
  {
    return dwa_planner_.choose_turn_sign(left_avg_range_, right_avg_range_);
  }

  std::optional<geometry_msgs::msg::Twist> plan_dwa_command(
    const RobotPose2D & pose,
    const MissionNode & target,
    int preferred_turn_sign) const
  {
    return dwa_planner_.plan_command(
      pose,
      target,
      preferred_turn_sign,
      scan_points_,
      front_min_range_,
      [this](double x, double y, double clearance) {
        return is_static_map_collision(x, y, clearance);
      });
  }

  geometry_msgs::msg::Twist make_immediate_escape_command(
    int turn_sign,
    const RobotPose2D & pose,
    const MissionNode & target) const
  {
    return dwa_planner_.make_immediate_escape_command(
      turn_sign,
      pose,
      target,
      front_min_range_,
      is_forward_collision_blocked(),
      is_front_emergency_blocked());
  }

  bool can_resume_mission_drive(const RobotPose2D & pose, const MissionNode & target) const
  {
    const auto next_drive_command = make_drive_command(pose, target, false);

    return is_front_clear() &&
      !is_static_map_collision(pose.x, pose.y, dwa_static_map_clearance_) &&
      !will_hit_static_map(pose, next_drive_command, 1.2) &&
      !will_hit_dynamic_obstacle(next_drive_command, 1.5, dynamic_path_min_clearance_) &&
      !closest_dynamic_contact_risk().has_value() &&
      !is_front_emergency_blocked() &&
      !is_forward_collision_blocked();
  }

  bool try_avoid_front_obstacle(int attempt, const MissionNode & target)
  {
    if (!enable_obstacle_avoidance_) {
      return false;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Starting DWA obstacle avoidance attempt %d. front=%.2f raw_front=%.2f",
      attempt,
      front_min_range_,
      raw_front_min_range_);

    const auto end_time = this->now() + rclcpp::Duration::from_seconds(dwa_max_duration_);
    const auto start_time = this->now();
    const int turn_sign = choose_avoidance_turn_sign();
    std::optional<rclcpp::Time> front_clear_since;
    std::optional<geometry_msgs::msg::Twist> committed_arc;
    std::optional<rclcpp::Time> committed_arc_until;
    rclcpp::Rate rate(20.0);

    while (rclcpp::ok() && this->now() < end_time) {
      rclcpp::spin_some(this->get_node_base_interface());
      const auto pose = lookup_robot_pose();
      if (!pose.has_value()) {
        rate.sleep();
        continue;
      }

      const bool min_duration_passed =
        (this->now() - start_time).seconds() >= dwa_min_duration_;
      if (min_duration_passed && can_resume_mission_drive(pose.value(), target)) {
        if (!front_clear_since.has_value()) {
          front_clear_since = this->now();
        } else if ((this->now() - front_clear_since.value()).seconds() >= dwa_front_clear_hold_) {
          RCLCPP_INFO(this->get_logger(), "DWA avoidance cleared the front path.");
          return true;
        }
      } else {
        front_clear_since.reset();
      }

      if (
        !committed_arc.has_value() ||
        !committed_arc_until.has_value() ||
        this->now() >= committed_arc_until.value())
      {
        committed_arc = plan_dwa_command(pose.value(), target, turn_sign).value_or(
          make_immediate_escape_command(turn_sign, pose.value(), target));
        committed_arc_until =
          this->now() + rclcpp::Duration::from_seconds(arc_commit_duration_);
      }

      auto command = committed_arc.value();
      cmd_pub_->publish(apply_dwa_lidar_safety(command));
      rate.sleep();
    }

    stop();
    RCLCPP_WARN(this->get_logger(), "DWA avoidance timed out before the front path cleared.");
    return false;
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

  std::optional<RobotPose2D> lookup_robot_pose_silent()
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      RobotPose2D pose;
      pose.x = transform.transform.translation.x;
      pose.y = transform.transform.translation.y;
      pose.yaw = yaw_from_quaternion(transform.transform.rotation);
      return pose;
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  geometry_msgs::msg::Twist apply_lidar_safety(geometry_msgs::msg::Twist command)
  {
    if (!enable_lidar_safety_ || command.linear.x <= 0.0) {
      return command;
    }

    if (is_front_emergency_blocked()) {
      command.linear.x = 0.0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Front safety obstacle raw=%.2fm dynamic=%.2fm <= %.2fm. Blocking forward motion.",
        raw_front_min_range_,
        front_min_range_,
        front_emergency_stop_distance_);
      return command;
    }

    if (is_forward_collision_blocked()) {
      command.linear.x = 0.0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Forward collision zone obstacle raw=%.2fm dynamic=%.2fm <= %.2fm. Blocking forward motion.",
        raw_forward_collision_min_range_,
        forward_collision_min_range_,
        forward_collision_stop_distance_);
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

  bool will_hit_static_map(
    const RobotPose2D & pose,
    const geometry_msgs::msg::Twist & command,
    double horizon) const
  {
    if (command.linear.x <= 0.0 || horizon <= 0.0) {
      return false;
    }

    const double dt = 0.10;
    double sim_x = 0.0;
    double sim_y = 0.0;
    double sim_yaw = 0.0;
    const double cos_yaw = std::cos(pose.yaw);
    const double sin_yaw = std::sin(pose.yaw);

    for (double time = 0.0; time <= horizon; time += dt) {
      sim_x += command.linear.x * std::cos(sim_yaw) * dt;
      sim_y += command.linear.x * std::sin(sim_yaw) * dt;
      sim_yaw = normalize_angle(sim_yaw + command.angular.z * dt);

      const double map_x = pose.x + cos_yaw * sim_x - sin_yaw * sim_y;
      const double map_y = pose.y + sin_yaw * sim_x + cos_yaw * sim_y;
      if (is_static_map_collision(map_x, map_y, dwa_static_map_clearance_)) {
        return true;
      }
    }

    return false;
  }

  bool will_hit_dynamic_obstacle(
    const geometry_msgs::msg::Twist & command,
    double horizon,
    double min_clearance) const
  {
    if (command.linear.x <= 0.0 || horizon <= 0.0 || min_clearance <= 0.0) {
      return false;
    }

    const double dt = 0.10;
    double sim_x = 0.0;
    double sim_y = 0.0;
    double sim_yaw = 0.0;

    for (double time = 0.0; time <= horizon; time += dt) {
      sim_x += command.linear.x * std::cos(sim_yaw) * dt;
      sim_y += command.linear.x * std::sin(sim_yaw) * dt;
      sim_yaw = normalize_angle(sim_yaw + command.angular.z * dt);

      for (const auto & point : scan_points_) {
        if (point.x < -0.18) {
          continue;
        }
        if (std::hypot(point.x - sim_x, point.y - sim_y) <= min_clearance) {
          return true;
        }
      }
    }

    return false;
  }

  bool is_drive_path_obstructed(const geometry_msgs::msg::Twist & command) const
  {
    return enable_lidar_safety_ &&
      command.linear.x > 0.0 &&
      will_hit_dynamic_obstacle(command, 1.0, dynamic_path_min_clearance_);
  }

  std::optional<DynamicObstacleRisk> closest_dynamic_contact_risk() const
  {
    DynamicObstacleRisk closest;

    for (const auto & point : scan_points_) {
      if (point.x < -0.30 || point.x > 0.42) {
        continue;
      }

      const double distance = std::hypot(point.x, point.y);
      if (distance > dynamic_contact_stop_distance_) {
        continue;
      }

      if (distance < closest.distance) {
        closest.x = point.x;
        closest.y = point.y;
        closest.distance = distance;
      }
    }

    if (!std::isfinite(closest.distance)) {
      return std::nullopt;
    }
    return closest;
  }

  int dynamic_contact_escape_turn_sign(const DynamicObstacleRisk & risk) const
  {
    if (risk.x > 0.08) {
      return risk.y >= 0.0 ? -1 : 1;
    }
    return risk.y >= 0.0 ? 1 : -1;
  }

  bool has_front_escape_room() const
  {
    const double escape_front_limit = dynamic_contact_escape_distance_ + 0.10;
    return std::isfinite(front_min_range_) &&
      front_min_range_ > escape_front_limit &&
      std::isfinite(forward_collision_min_range_) &&
      forward_collision_min_range_ > escape_front_limit;
  }

  geometry_msgs::msg::Twist apply_dynamic_contact_safety(
    geometry_msgs::msg::Twist command,
    bool allow_reverse_escape)
  {
    const auto risk = closest_dynamic_contact_risk();
    if (!risk.has_value()) {
      if (
        contact_escape_until_.has_value() &&
        this->now() >= contact_escape_until_.value())
      {
        contact_escape_turn_sign_.reset();
        contact_escape_until_.reset();
      }
      return command;
    }

    int turn_sign = dynamic_contact_escape_turn_sign(risk.value());
    if (
      contact_escape_turn_sign_.has_value() &&
      contact_escape_until_.has_value() &&
      this->now() < contact_escape_until_.value())
    {
      turn_sign = contact_escape_turn_sign_.value();
    } else {
      contact_escape_turn_sign_ = turn_sign;
    }
    contact_escape_until_ =
      this->now() + rclcpp::Duration::from_seconds(dynamic_contact_turn_hold_);

    if (risk->distance > dynamic_contact_escape_distance_) {
      if (command.linear.x > 0.0) {
        command.linear.x = std::min(command.linear.x, 0.05);
      }
      command.angular.z = clamp(
        command.angular.z + static_cast<double>(turn_sign) * 0.12,
        -max_angular_speed_,
        max_angular_speed_);
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Dynamic obstacle near x=%.2f y=%.2f d=%.2f. Biasing arc away.",
        risk->x,
        risk->y,
        risk->distance);
      return command;
    }

    command.linear.x = 0.0;
    command.angular.z = static_cast<double>(turn_sign) * std::min(0.22, max_angular_speed_);

    if (
      allow_reverse_escape &&
      risk->x > 0.08 &&
      std::isfinite(rear_min_range_) &&
      rear_min_range_ > rear_stop_distance_ + 0.08)
    {
      command.linear.x = -dynamic_contact_escape_speed_;
      command.angular.z = static_cast<double>(turn_sign) * 0.12;
    } else if (
      allow_reverse_escape &&
      risk->x <= 0.08 &&
      has_front_escape_room())
    {
      command.linear.x = dynamic_contact_escape_speed_;
      command.angular.z = static_cast<double>(turn_sign) * 0.10;
    }

    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Dynamic obstacle contact risk x=%.2f y=%.2f d=%.2f. Escaping.",
      risk->x,
      risk->y,
      risk->distance);
    return command;
  }

  geometry_msgs::msg::Twist apply_navigation_safety(
    geometry_msgs::msg::Twist command,
    const RobotPose2D & pose)
  {
    command = apply_lidar_safety(command);
    if (command.linear.x > 0.0) {
      command = apply_dynamic_contact_safety(command, false);
    }
    if (
      command.linear.x > 0.0 &&
      will_hit_dynamic_obstacle(command, 1.2, dynamic_path_min_clearance_))
    {
      command.linear.x = 0.0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Dynamic obstacle on commanded path. Blocking forward motion.");
    }
    if (command.linear.x > 0.0 && will_hit_static_map(pose, command, 1.2)) {
      command.linear.x = 0.0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Static map obstacle on commanded path. Blocking forward motion.");
    }
    return command;
  }

  geometry_msgs::msg::Twist apply_dwa_lidar_safety(geometry_msgs::msg::Twist command)
  {
    if (!enable_lidar_safety_) {
      return command;
    }

    command = apply_dynamic_contact_safety(command, true);

    if (command.linear.x > 0.0 && is_front_emergency_blocked()) {
      command.linear.x = 0.0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "DWA front emergency obstacle raw=%.2fm dynamic=%.2fm <= %.2fm. Rotating in place.",
        raw_front_min_range_,
        front_min_range_,
        front_emergency_stop_distance_);
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
    double raw_front_min_range = std::numeric_limits<double>::infinity();
    double forward_collision_min_range = std::numeric_limits<double>::infinity();
    double raw_forward_collision_min_range = std::numeric_limits<double>::infinity();
    double rear_min_range = std::numeric_limits<double>::infinity();
    double left_sum = 0.0;
    double right_sum = 0.0;
    int left_count = 0;
    int right_count = 0;
    bool front_raw_seen = false;
    bool forward_collision_raw_seen = false;
    std::vector<ScanPoint2D> scan_points;
    const auto filter_pose = lookup_robot_pose_silent();

    for (size_t index = 0; index < msg->ranges.size(); ++index) {
      const double angle = msg->angle_min + static_cast<double>(index) * msg->angle_increment;
      const double range = msg->ranges[index];
      if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max) {
        continue;
      }

      const bool mapped_obstacle =
        filter_pose.has_value() &&
        should_ignore_mapped_scan_point(filter_pose.value(), angle, range, true);

      if (range <= dwa_obstacle_range_ && !mapped_obstacle) {
        scan_points.push_back({
          dwa_lidar_x_offset_ + range * std::cos(angle),
          range * std::sin(angle)});
      }

      if (angle_in_sector(angle, 0.0, front_sector_angle_)) {
        front_raw_seen = true;
        raw_front_min_range = std::min(raw_front_min_range, range);
        const bool mapped_front_obstacle =
          filter_pose.has_value() &&
          should_ignore_mapped_scan_point(filter_pose.value(), angle, range, false);
        if (!mapped_front_obstacle) {
          front_min_range = std::min(front_min_range, range);
        }
      }

      if (angle_in_sector(angle, 0.0, forward_collision_sector_angle_)) {
        forward_collision_raw_seen = true;
        raw_forward_collision_min_range = std::min(raw_forward_collision_min_range, range);
        const bool mapped_forward_obstacle =
          filter_pose.has_value() &&
          should_ignore_mapped_scan_point(filter_pose.value(), angle, range, false);
        if (!mapped_forward_obstacle) {
          forward_collision_min_range = std::min(forward_collision_min_range, range);
        }
      }

      if (angle_in_sector(angle, M_PI, rear_sector_angle_)) {
        rear_min_range = std::min(rear_min_range, range);
      }

      if (angle_in_sector(angle, M_PI_2, side_sector_angle_)) {
        left_sum += range;
        ++left_count;
      }

      if (angle_in_sector(angle, -M_PI_2, side_sector_angle_)) {
        right_sum += range;
        ++right_count;
      }
    }

    if (
      !std::isfinite(front_min_range) &&
      front_raw_seen &&
      ignore_mapped_front_obstacles_ &&
      latest_map_ &&
      filter_pose.has_value())
    {
      front_min_range = msg->range_max;
    }

    if (
      !std::isfinite(forward_collision_min_range) &&
      forward_collision_raw_seen &&
      ignore_mapped_front_obstacles_ &&
      latest_map_ &&
      filter_pose.has_value())
    {
      forward_collision_min_range = msg->range_max;
    }

    raw_front_min_range_ = raw_front_min_range;
    raw_forward_collision_min_range_ = raw_forward_collision_min_range;
    front_min_range_ = front_min_range;
    forward_collision_min_range_ = forward_collision_min_range;
    rear_min_range_ = rear_min_range;
    left_avg_range_ = left_count > 0 ? left_sum / static_cast<double>(left_count) :
      std::numeric_limits<double>::infinity();
    right_avg_range_ = right_count > 0 ? right_sum / static_cast<double>(right_count) :
      std::numeric_limits<double>::infinity();
    scan_points_ = std::move(scan_points);
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    latest_map_ = msg;
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
  double front_emergency_stop_distance_{0.54};
  double front_stop_distance_{0.60};
  double forward_collision_sector_angle_{M_PI_2};
  double forward_collision_stop_distance_{0.46};
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
  bool enable_obstacle_avoidance_{true};
  double obstacle_wait_seconds_{0.0};
  int obstacle_avoidance_max_attempts_{0};
  double obstacle_avoidance_trigger_distance_{0.62};
  double dwa_max_duration_{6.0};
  double dwa_min_duration_{0.7};
  double dwa_sim_time_{3.0};
  double dwa_sim_step_{0.10};
  double dwa_min_linear_speed_{0.04};
  double dwa_max_linear_speed_{0.095};
  double dwa_max_angular_speed_{0.55};
  int dwa_linear_samples_{4};
  int dwa_angular_samples_{11};
  double dwa_robot_radius_{0.18};
  double dwa_lidar_x_offset_{0.08};
  double dwa_safety_margin_{0.25};
  double dwa_static_map_clearance_{0.21};
  double dwa_obstacle_range_{1.6};
  double dwa_goal_weight_{0.9};
  double dwa_clearance_weight_{2.0};
  double dwa_speed_weight_{1.8};
  double dwa_front_clear_hold_{0.6};
  double dwa_stuck_turn_speed_{0.38};
  double arc_commit_duration_{1.4};
  double dynamic_contact_stop_distance_{0.28};
  double dynamic_contact_escape_distance_{0.22};
  double dynamic_contact_escape_speed_{0.04};
  double dynamic_contact_turn_hold_{1.8};
  double dynamic_path_min_clearance_{0.36};
  std::optional<int> contact_escape_turn_sign_;
  std::optional<rclcpp::Time> contact_escape_until_;
  bool ignore_mapped_front_obstacles_{true};
  double map_obstacle_padding_{0.08};
  int map_occupied_threshold_{50};
  amr_topology::DwaPlanner dwa_planner_;
  double raw_front_min_range_{std::numeric_limits<double>::infinity()};
  double raw_forward_collision_min_range_{std::numeric_limits<double>::infinity()};
  double front_min_range_{std::numeric_limits<double>::infinity()};
  double forward_collision_min_range_{std::numeric_limits<double>::infinity()};
  double front_clear_distance_{0.90};
  double rear_min_range_{std::numeric_limits<double>::infinity()};
  double left_avg_range_{std::numeric_limits<double>::infinity()};
  double right_avg_range_{std::numeric_limits<double>::infinity()};
  std::vector<ScanPoint2D> scan_points_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_command_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
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
