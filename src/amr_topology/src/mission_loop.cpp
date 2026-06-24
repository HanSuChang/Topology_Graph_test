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
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include "amr_topology/local_astar_pure_pursuit.hpp"

using namespace std::chrono_literals;

namespace
{

struct MissionNode
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::string type;
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
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
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

}  // namespace

class MissionLoop : public rclcpp::Node
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using FollowPathGoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

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
    this->declare_parameter<std::string>("leader_pose_topic", "/turtlebot/pose");
    this->declare_parameter<std::string>("rc_car_mode_topic", "/rc_car/follower_mode");
    this->declare_parameter<std::string>("rc_car_slot_status_topic", "/rc_car/slot_wait_status");
    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<std::string>("map_topic", "/map");
    this->declare_parameter<bool>("enable_lidar_safety", true);
    this->declare_parameter<int>("repeat_count", 2);
    this->declare_parameter<double>("wait_seconds", 3.0);
    this->declare_parameter<double>("precision_wait_seconds", 5.0);
    this->declare_parameter<double>("goal_tolerance", 0.08);
    this->declare_parameter<double>("precision_goal_tolerance", 0.04);
    this->declare_parameter<double>("waypoint_tolerance", 0.22);
    this->declare_parameter<double>("yaw_tolerance", 0.08);
    this->declare_parameter<bool>("stop_rc_car_on_a_leader_slot_ccw_yaw", false);
    this->declare_parameter<double>("rc_car_a_slot_stop_yaw_delta", M_PI / 3.0);
    this->declare_parameter<double>("max_linear_speed", 0.14);
    this->declare_parameter<double>("max_angular_speed", 0.45);
    this->declare_parameter<double>("linear_gain", 0.70);
    this->declare_parameter<double>("angular_gain", 1.1);
    this->declare_parameter<double>("drive_heading_limit", 1.75);
    this->declare_parameter<double>("curve_min_linear_speed", 0.03);
    this->declare_parameter<double>("slot_departure_linear_speed", 0.065);
    this->declare_parameter<double>("dock_tolerance", 0.04);
    this->declare_parameter<double>("dock_lateral_tolerance", 0.08);
    this->declare_parameter<double>("dock_longitudinal_tolerance", 0.04);
    this->declare_parameter<double>("dock_max_reverse_speed", 0.055);
    this->declare_parameter<double>("dock_max_reverse_angular_speed", 0.22);
    this->declare_parameter<double>("dock_reverse_start_distance", 0.75);
    this->declare_parameter<double>("dock_parking_lateral_offset", 0.15);
    this->declare_parameter<double>("dock_linear_gain", 0.30);
    this->declare_parameter<double>("dock_angular_gain", 0.8);
    this->declare_parameter<double>("obstacle_emergency_stop_distance", 0.20);
    this->declare_parameter<double>("obstacle_side_stop_distance", 0.24);
    this->declare_parameter<double>("rear_stop_distance", 0.18);
    this->declare_parameter<double>("obstacle_target_stop_distance", 0.40);
    this->declare_parameter<double>("obstacle_clear_distance", 0.62);
    this->declare_parameter<double>("obstacle_backup_target_distance", 0.38);
    this->declare_parameter<double>("obstacle_stop_and_scan_seconds", 2.0);
    this->declare_parameter<double>("obstacle_target_sector_width", 0.52);
    this->declare_parameter<double>("obstacle_front_sector_width", 0.70);
    this->declare_parameter<double>("obstacle_avoid_linear_speed", 0.055);
    this->declare_parameter<double>("obstacle_avoid_angular_speed", 0.22);
    this->declare_parameter<double>("obstacle_rotate_in_place_heading_threshold", 0.35);
    this->declare_parameter<double>("obstacle_rotate_in_place_resume_threshold", 0.18);
    this->declare_parameter<double>("obstacle_rotate_in_place_max_heading", 1.05);
    this->declare_parameter<double>("obstacle_backup_speed", 0.045);
    this->declare_parameter<double>("local_planner_lookahead_distance", 0.28);
    this->declare_parameter<double>("local_planner_goal_tolerance", 0.12);
    this->declare_parameter<double>("local_planner_max_plan_distance", 3.5);
    this->declare_parameter<double>("local_planner_robot_radius", 0.18);
    this->declare_parameter<double>("local_planner_inflation_radius", 0.28);
    this->declare_parameter<double>("local_planner_dynamic_obstacle_radius", 0.22);
    this->declare_parameter<double>("local_planner_path_corridor_width", 0.80);
    this->declare_parameter<bool>("enable_mppi_rescue", true);
    this->declare_parameter<std::string>("mppi_follow_path_action", "follow_path");
    this->declare_parameter<double>("mppi_rescue_timeout_seconds", 18.0);
    this->declare_parameter<double>("mppi_rescue_stop_and_plan_seconds", 3.0);
    this->declare_parameter<double>("mppi_rescue_lateral_offset", 0.24);
    this->declare_parameter<double>("mppi_rescue_forward_offset", 0.85);
    this->declare_parameter<double>("mppi_rescue_min_handoff_seconds", 5.0);
    this->declare_parameter<double>("mppi_rescue_handoff_front_clearance", 0.48);
    this->declare_parameter<double>("mppi_rescue_handoff_side_clearance", 0.30);
    this->declare_parameter<double>("mppi_rescue_slow_handoff_seconds", 8.0);
    this->declare_parameter<double>("mppi_rescue_slow_handoff_front_clearance", 0.38);
    this->declare_parameter<double>("mppi_rescue_side_escape_seconds", 3.0);
    this->declare_parameter<bool>("enable_corridor_pass", true);
    this->declare_parameter<double>("corridor_robot_width", 0.306);
    this->declare_parameter<double>("corridor_side_clearance", 0.07);
    this->declare_parameter<double>("corridor_min_passage_width", 0.45);
    this->declare_parameter<double>("corridor_hard_stop_width", 0.40);
    this->declare_parameter<double>("corridor_max_lateral_offset", 0.24);

    topology_file_ = this->get_parameter("topology_file").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    enable_lidar_safety_ = this->get_parameter("enable_lidar_safety").as_bool();
    repeat_count_ = this->get_parameter("repeat_count").as_int();
    wait_seconds_ = this->get_parameter("wait_seconds").as_double();
    precision_wait_seconds_ = this->get_parameter("precision_wait_seconds").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    precision_goal_tolerance_ = this->get_parameter("precision_goal_tolerance").as_double();
    waypoint_tolerance_ = this->get_parameter("waypoint_tolerance").as_double();
    yaw_tolerance_ = this->get_parameter("yaw_tolerance").as_double();
    stop_rc_car_on_a_leader_slot_ccw_yaw_ =
      this->get_parameter("stop_rc_car_on_a_leader_slot_ccw_yaw").as_bool();
    rc_car_a_slot_stop_yaw_delta_ = std::max(
      0.0, this->get_parameter("rc_car_a_slot_stop_yaw_delta").as_double());
    max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
    max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
    linear_gain_ = this->get_parameter("linear_gain").as_double();
    angular_gain_ = this->get_parameter("angular_gain").as_double();
    drive_heading_limit_ = this->get_parameter("drive_heading_limit").as_double();
    curve_min_linear_speed_ = this->get_parameter("curve_min_linear_speed").as_double();
    slot_departure_linear_speed_ = this->get_parameter("slot_departure_linear_speed").as_double();
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
    enable_mppi_rescue_ = this->get_parameter("enable_mppi_rescue").as_bool();
    mppi_rescue_timeout_seconds_ = std::max(
      18.0,
      this->get_parameter("mppi_rescue_timeout_seconds").as_double());
    mppi_rescue_stop_and_plan_seconds_ =
      this->get_parameter("mppi_rescue_stop_and_plan_seconds").as_double();
    mppi_rescue_lateral_offset_ =
      this->get_parameter("mppi_rescue_lateral_offset").as_double();
    mppi_rescue_forward_offset_ =
      this->get_parameter("mppi_rescue_forward_offset").as_double();
    mppi_rescue_min_handoff_seconds_ =
      this->get_parameter("mppi_rescue_min_handoff_seconds").as_double();
    mppi_rescue_handoff_front_clearance_ =
      this->get_parameter("mppi_rescue_handoff_front_clearance").as_double();
    mppi_rescue_handoff_side_clearance_ =
      this->get_parameter("mppi_rescue_handoff_side_clearance").as_double();
    mppi_rescue_slow_handoff_seconds_ =
      this->get_parameter("mppi_rescue_slow_handoff_seconds").as_double();
    mppi_rescue_slow_handoff_front_clearance_ =
      this->get_parameter("mppi_rescue_slow_handoff_front_clearance").as_double();
    mppi_rescue_side_escape_seconds_ =
      this->get_parameter("mppi_rescue_side_escape_seconds").as_double();
    enable_corridor_pass_ = this->get_parameter("enable_corridor_pass").as_bool();
    corridor_robot_width_ = this->get_parameter("corridor_robot_width").as_double();
    corridor_side_clearance_ = this->get_parameter("corridor_side_clearance").as_double();
    corridor_min_passage_width_ = std::max(
      this->get_parameter("corridor_min_passage_width").as_double(),
      corridor_robot_width_ + 2.0 * corridor_side_clearance_);
    corridor_hard_stop_width_ =
      this->get_parameter("corridor_hard_stop_width").as_double();
    corridor_max_lateral_offset_ =
      this->get_parameter("corridor_max_lateral_offset").as_double();

    amr_topology::LocalPlannerOptions planner_options;
    planner_options.emergency_stop_distance =
      this->get_parameter("obstacle_emergency_stop_distance").as_double();
    planner_options.side_stop_distance =
      this->get_parameter("obstacle_side_stop_distance").as_double();
    planner_options.rear_stop_distance =
      this->get_parameter("rear_stop_distance").as_double();
    planner_options.obstacle_trigger_distance =
      this->get_parameter("obstacle_target_stop_distance").as_double();
    planner_options.goal_block_distance =
      this->get_parameter("obstacle_clear_distance").as_double();
    planner_options.stop_and_plan_seconds =
      this->get_parameter("obstacle_stop_and_scan_seconds").as_double();
    planner_options.front_sector_width =
      this->get_parameter("obstacle_front_sector_width").as_double();
    planner_options.max_linear_speed =
      this->get_parameter("obstacle_avoid_linear_speed").as_double();
    planner_options.max_angular_speed =
      this->get_parameter("obstacle_avoid_angular_speed").as_double();
    planner_options.rotate_in_place_heading_threshold =
      this->get_parameter("obstacle_rotate_in_place_heading_threshold").as_double();
    planner_options.rotate_in_place_resume_threshold =
      this->get_parameter("obstacle_rotate_in_place_resume_threshold").as_double();
    planner_options.rotate_in_place_max_heading =
      this->get_parameter("obstacle_rotate_in_place_max_heading").as_double();
    planner_options.lookahead_distance =
      this->get_parameter("local_planner_lookahead_distance").as_double();
    planner_options.goal_tolerance =
      this->get_parameter("local_planner_goal_tolerance").as_double();
    planner_options.max_plan_distance =
      this->get_parameter("local_planner_max_plan_distance").as_double();
    planner_options.robot_radius =
      this->get_parameter("local_planner_robot_radius").as_double();
    planner_options.inflation_radius =
      this->get_parameter("local_planner_inflation_radius").as_double();
    planner_options.dynamic_obstacle_radius =
      this->get_parameter("local_planner_dynamic_obstacle_radius").as_double();
    planner_options.path_corridor_width =
      this->get_parameter("local_planner_path_corridor_width").as_double();
    local_planner_ = amr_topology::LocalAStarPurePursuit(planner_options);

    load_topology();

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      this->get_parameter("cmd_vel_topic").as_string(), 10);
    leader_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      this->get_parameter("leader_pose_topic").as_string(), 10);
    rc_car_mode_pub_ = this->create_publisher<std_msgs::msg::String>(
      this->get_parameter("rc_car_mode_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local());
    rc_car_slot_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      this->get_parameter("rc_car_slot_status_topic").as_string(),
      10,
      std::bind(&MissionLoop::handle_rc_car_slot_status, this, std::placeholders::_1));
    rc_car_mode_timer_ = this->create_wall_timer(
      500ms, std::bind(&MissionLoop::republish_rc_car_mode, this));
    mppi_follow_path_client_ = rclcpp_action::create_client<FollowPath>(
      this,
      this->get_parameter("mppi_follow_path_action").as_string());

    mission_command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "mission_command",
      10,
      std::bind(&MissionLoop::handle_mission_command, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      this->get_parameter("scan_topic").as_string(),
      rclcpp::SensorDataQoS(),
      std::bind(&MissionLoop::handle_scan, this, std::placeholders::_1));

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      this->get_parameter("map_topic").as_string(),
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&MissionLoop::handle_map, this, std::placeholders::_1));
  }

  void run()
  {
    RCLCPP_INFO(this->get_logger(), "Starting A/B return mission loop");

    for (int cycle = 1; cycle <= repeat_count_ && rclcpp::ok(); ++cycle) {
      RCLCPP_INFO(this->get_logger(), "Cycle %d/%d: moving to A", cycle, repeat_count_);
      publish_rc_car_mode("follow");
      go_path({"loading", "intersection_1", "a_entry", "a_leader_slot"});
      if (!rclcpp::ok()) {
        return;
      }
      wait_at_slot("A", "a_entry");
      if (!rclcpp::ok()) {
        return;
      }
      publish_rc_car_mode("stop");
      hold_at_leader_slot("A leader slot");
      if (!rclcpp::ok()) {
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Cycle %d/%d: moving to B", cycle, repeat_count_);
      publish_rc_car_mode("follow");
      go_path({"loading", "intersection_2", "b_entry", "b_leader_slot"});
      if (!rclcpp::ok()) {
        return;
      }
      publish_rc_car_mode("stop");
      wait_at_slot("B", "b_entry");
      if (!rclcpp::ok()) {
        return;
      }
      publish_rc_car_mode("stop");
      hold_at_leader_slot("B leader slot");
    }

    publish_rc_car_mode("stop");
    wait_for_charger_request();
  }

private:
  struct CorridorPassDecision
  {
    bool passable{false};
    bool hard_blocked{false};
    double width{0.0};
    double left_min{std::numeric_limits<double>::infinity()};
    double right_min{std::numeric_limits<double>::infinity()};
    double signed_lateral_offset{0.0};
  };

  void go_path(const std::vector<std::string> & path)
  {
    if (path.empty()) {
      return;
    }

    size_t target_index = 0;
    local_planner_.reset();
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

      if (mppi_rescue_active_) {
        const double rescue_elapsed = (this->now() - mppi_rescue_started_at_).seconds();
        if (distance <= tolerance) {
          cancel_mppi_rescue();
          local_planner_.reset();
          if (is_final) {
            stop();
            if (target_name != "charger_entry") {
              rotate_to_node_yaw(target_name);
            }
            RCLCPP_INFO(this->get_logger(), "Reached %s with MPPI rescue", target_name.c_str());
            return;
          }
          RCLCPP_INFO(this->get_logger(), "Passed waypoint %s with MPPI rescue", target_name.c_str());
          reset_mppi_rescue_attempts();
          ++target_index;
          rate.sleep();
          continue;
        }

        if (
          rescue_elapsed >= mppi_rescue_min_handoff_seconds_ &&
          mppi_rescue_handoff_is_clear(pose.value(), target, false))
        {
          RCLCPP_WARN(
            this->get_logger(),
            "MPPI rescue handoff after %.1f seconds; returning control to local A*",
            rescue_elapsed);
          cancel_mppi_rescue();
          local_planner_.reset();
        } else if (
          rescue_elapsed >= mppi_rescue_slow_handoff_seconds_ &&
          mppi_rescue_handoff_is_clear(pose.value(), target, true))
        {
          RCLCPP_WARN(
            this->get_logger(),
            "MPPI rescue slow handoff after %.1f seconds; returning control to local A*",
            rescue_elapsed);
          cancel_mppi_rescue();
          local_planner_.reset();
        } else if (rescue_elapsed > mppi_rescue_timeout_seconds_) {
          RCLCPP_WARN(
            this->get_logger(),
            "MPPI rescue timed out after %.1f seconds; returning control to local A*",
            rescue_elapsed);
          cancel_mppi_rescue();
          local_planner_.reset();
        } else {
          rate.sleep();
          continue;
        }
      }

      if (
        enable_lidar_safety_ &&
        !is_final &&
        is_skippable_node(target_name, target) &&
        local_planner_.target_is_blocked(
          amr_topology::Pose2D{pose->x, pose->y, pose->yaw},
          amr_topology::Target2D{target.x, target.y},
          this->now()))
      {
        const auto & next_target = nodes_.at(path[target_index + 1]);
        if (local_planner_.target_direction_is_clear(
            amr_topology::Pose2D{pose->x, pose->y, pose->yaw},
            amr_topology::Target2D{next_target.x, next_target.y},
            this->now()))
        {
          RCLCPP_WARN(
            this->get_logger(),
            "Skipping blocked pass-through node %s and moving to %s",
            target_name.c_str(),
            path[target_index + 1].c_str());
          local_planner_.reset();
          reset_mppi_rescue_attempts();
          ++target_index;
          continue;
        }
      }

      if (
        enable_lidar_safety_ &&
        is_blocking_target_node(target_name) &&
        local_planner_.target_is_blocked(
          amr_topology::Pose2D{pose->x, pose->y, pose->yaw},
          amr_topology::Target2D{target.x, target.y},
          this->now()))
      {
        wait_for_blocked_target_clear(target_name, target);
        local_planner_.reset();
        rate.sleep();
        continue;
      }

      if (is_final && distance <= tolerance) {
        local_planner_.reset();
        stop();
        if (target_name != "charger_entry") {
          rotate_to_node_yaw(target_name);
        }
        RCLCPP_INFO(this->get_logger(), "Reached %s", target_name.c_str());
        return;
      }

      if (!is_final && distance <= tolerance) {
        RCLCPP_INFO(this->get_logger(), "Passed waypoint %s", target_name.c_str());
        local_planner_.reset();
        reset_mppi_rescue_attempts();
        ++target_index;
        continue;
      }

      const bool leaving_leader_slot =
        is_leader_slot(source_name) && is_entry_node(target_name);
      const auto command = leaving_leader_slot ?
        make_slot_departure_command() :
        make_drive_command(pose.value(), target, is_final);

      amr_topology::LocalPlannerDecision local_plan;
      if (enable_lidar_safety_) {
        local_plan = local_planner_.update(
          amr_topology::Pose2D{pose->x, pose->y, pose->yaw},
          amr_topology::Target2D{target.x, target.y},
          this->now());
      }
      if (enable_lidar_safety_ && local_plan.has_command) {
        const bool stop_and_plan = local_plan.state == "stop_and_plan";
        const bool side_escape = local_plan.state == "side_escape";
        if (stop_and_plan && !mppi_stop_and_plan_active_) {
          mppi_stop_and_plan_active_ = true;
          mppi_stop_and_plan_started_at_ = this->now();
        } else if (!stop_and_plan) {
          mppi_stop_and_plan_active_ = false;
        }
        if (side_escape && !mppi_side_escape_active_) {
          mppi_side_escape_active_ = true;
          mppi_side_escape_started_at_ = this->now();
        } else if (!side_escape) {
          mppi_side_escape_active_ = false;
        }

        const bool stop_and_plan_timed_out =
          mppi_stop_and_plan_active_ &&
          (this->now() - mppi_stop_and_plan_started_at_).seconds() >=
          mppi_rescue_stop_and_plan_seconds_;
        const bool side_escape_timed_out =
          mppi_side_escape_active_ &&
          (this->now() - mppi_side_escape_started_at_).seconds() >=
          mppi_rescue_side_escape_seconds_;
        const auto corridor = evaluate_corridor_pass();
        if (corridor.has_value() && corridor->hard_blocked) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Corridor blocked: width=%.2f required=%.2f; holding position",
            corridor->width,
            corridor_min_passage_width_);
          local_planner_.reset();
          cmd_pub_->publish(geometry_msgs::msg::Twist{});
          rate.sleep();
          continue;
        }

        if (
          enable_mppi_rescue_ &&
          (local_plan.blocked || stop_and_plan_timed_out || side_escape_timed_out))
        {
          if (start_mppi_rescue(pose.value(), target, target_name, corridor)) {
            mppi_stop_and_plan_active_ = false;
            mppi_side_escape_active_ = false;
            rate.sleep();
            continue;
          }
        }
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Local A* Pure Pursuit active: %s",
          local_plan.state.c_str());
        cmd_pub_->publish(local_plan.command);
      } else {
        cmd_pub_->publish(command);
      }
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

    if (std::abs(heading_error) > drive_heading_limit_) {
      return command;
    }

    const double speed_scale = std::max(0.25, std::cos(heading_error));
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
      node_name == "b_leader_slot_precision";
  }

  bool is_entry_node(const std::string & node_name) const
  {
    return node_name == "a_entry" || node_name == "b_entry";
  }

  bool is_skippable_node(const std::string & node_name, const MissionNode & node) const
  {
    if (
      node_name == "intersection_1" || node_name == "intersection_2" ||
      node_name == "a_entry" || node_name == "b_entry")
    {
      return true;
    }
    return node.type == "intersection" || node.type == "area_entry" ||
      node.type == "standby" || node.type == "waypoint";
  }

  bool is_blocking_target_node(const std::string & node_name) const
  {
    return node_name == "a_leader_slot" || node_name == "b_leader_slot" ||
      node_name == "b_leader_slot_precision";
  }

  geometry_msgs::msg::PoseStamped make_pose_stamped(double x, double y, double yaw) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = this->now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation = quaternion_from_yaw(yaw);
    return pose;
  }

  nav_msgs::msg::Path make_mppi_rescue_path(
    const RobotPose2D & pose,
    const MissionNode & target,
    const std::optional<CorridorPassDecision> & corridor) const
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = this->now();

    const double distance = std::hypot(target.x - pose.x, target.y - pose.y);
    const double target_heading = std::atan2(target.y - pose.y, target.x - pose.x);
    double signed_lateral_offset{0.0};
    double side{0.0};
    bool corridor_path{false};
    if (corridor.has_value() && corridor->passable) {
      signed_lateral_offset = corridor->signed_lateral_offset;
      side = signed_lateral_offset >= 0.0 ? 1.0 : -1.0;
      corridor_path = true;
    } else {
      const double left_min = min_scan_range_in_sector(0.85, 0.65);
      const double right_min = min_scan_range_in_sector(-0.85, 0.65);
      side = choose_mppi_rescue_side(pose, target);
      const double side_delta = std::isfinite(left_min) && std::isfinite(right_min) ?
        std::abs(left_min - right_min) :
        0.18;
      const double adaptive_lateral_offset = clamp(0.12 + side_delta * 0.50, 0.16, 0.24);
      double lateral_offset = std::min(
        clamp(mppi_rescue_lateral_offset_, 0.16, 0.34),
        adaptive_lateral_offset);
      if (mppi_rescue_attempt_count_ >= 2) {
        lateral_offset = std::max(lateral_offset, 0.24);
      }
      if (
        std::abs(side - mppi_preferred_rescue_side_) < 0.1 &&
        mppi_preferred_rescue_lateral_offset_ > 0.0)
      {
        lateral_offset = std::max(lateral_offset, mppi_preferred_rescue_lateral_offset_);
      }
      signed_lateral_offset = side * lateral_offset;
    }
    const double first_forward = clamp(
      distance * 0.32,
      0.55,
      std::max(0.65, mppi_rescue_forward_offset_));
    const double second_forward = clamp(distance * 0.62, 1.05, 1.55);
    const double fade_forward = clamp(distance * 0.82, 1.25, 1.95);

    const double fx = std::cos(target_heading);
    const double fy = std::sin(target_heading);
    const double lx = -std::sin(target_heading);
    const double ly = std::cos(target_heading);

    struct PathPoint
    {
      double x;
      double y;
      double yaw;
    };

    std::vector<PathPoint> control_points;
    control_points.push_back({pose.x, pose.y, target_heading});
    control_points.push_back({
      pose.x + fx * first_forward + lx * signed_lateral_offset,
      pose.y + fy * first_forward + ly * signed_lateral_offset,
      target_heading});
    control_points.push_back({
      pose.x + fx * second_forward + lx * signed_lateral_offset,
      pose.y + fy * second_forward + ly * signed_lateral_offset,
      target_heading});

    if (distance > 1.2) {
      control_points.push_back({
        pose.x + fx * fade_forward + lx * (signed_lateral_offset * 0.45),
        pose.y + fy * fade_forward + ly * (signed_lateral_offset * 0.45),
        target_heading});
    }

    control_points.push_back({target.x, target.y, target.yaw});

    path.poses.push_back(make_pose_stamped(
      control_points.front().x, control_points.front().y, control_points.front().yaw));
    for (size_t i = 1; i < control_points.size(); ++i) {
      const auto & from = control_points[i - 1];
      const auto & to = control_points[i];
      const double segment_distance = std::hypot(to.x - from.x, to.y - from.y);
      const int segments = std::max(1, static_cast<int>(std::ceil(segment_distance / 0.18)));
      const double segment_heading = segment_distance > 0.02 ?
        std::atan2(to.y - from.y, to.x - from.x) :
        to.yaw;
      for (int j = 1; j <= segments; ++j) {
        const double ratio = static_cast<double>(j) / static_cast<double>(segments);
        path.poses.push_back(make_pose_stamped(
          from.x + (to.x - from.x) * ratio,
          from.y + (to.y - from.y) * ratio,
          segment_heading));
      }
    }

    RCLCPP_WARN(
      this->get_logger(),
      "MPPI rescue path: mode=%s side=%s lateral=%.2f distance=%.2f poses=%zu attempt=%d",
      corridor_path ? "corridor" : "rescue",
      side > 0.0 ? "left" : "right",
      std::abs(signed_lateral_offset),
      distance,
      path.poses.size(),
      mppi_rescue_attempt_count_);
    last_mppi_rescue_side_ = side;
    last_mppi_rescue_lateral_offset_ = std::abs(signed_lateral_offset);
    return path;
  }

  double choose_mppi_rescue_side(
    const RobotPose2D & pose,
    const MissionNode & target) const
  {
    const double left_min = min_scan_range_in_sector(0.85, 0.65);
    const double right_min = min_scan_range_in_sector(-0.85, 0.65);
    if (std::isfinite(left_min) || std::isfinite(right_min)) {
      const double nominal_side = left_min >= right_min ? 1.0 : -1.0;
      const double side_delta =
        std::isfinite(left_min) && std::isfinite(right_min) ?
        std::abs(left_min - right_min) :
        std::numeric_limits<double>::infinity();
      if (mppi_preferred_rescue_side_ != 0.0 && mppi_rescue_attempt_count_ >= 1) {
        return mppi_preferred_rescue_side_;
      }
      if (mppi_rescue_attempt_count_ >= 2 && side_delta < 0.12) {
        return -nominal_side;
      }
      return nominal_side;
    }

    const double target_heading = std::atan2(target.y - pose.y, target.x - pose.x);
    return normalize_angle(target_heading - pose.yaw) >= 0.0 ? 1.0 : -1.0;
  }

  double min_scan_range_in_sector(double center_angle, double sector_width) const
  {
    if (!last_scan_.has_value() || last_scan_->ranges.empty()) {
      return std::numeric_limits<double>::infinity();
    }

    double min_range = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
      const float range = last_scan_->ranges[i];
      if (!std::isfinite(range) || range < last_scan_->range_min || range > last_scan_->range_max) {
        continue;
      }
      const double angle = last_scan_->angle_min + static_cast<double>(i) * last_scan_->angle_increment;
      if (std::abs(normalize_angle(angle - center_angle)) <= sector_width * 0.5) {
        min_range = std::min(min_range, static_cast<double>(range));
      }
    }
    return min_range;
  }

  std::optional<CorridorPassDecision> evaluate_corridor_pass() const
  {
    if (!enable_corridor_pass_ || !last_scan_.has_value()) {
      return std::nullopt;
    }

    CorridorPassDecision decision;
    decision.left_min = min_scan_range_in_sector(0.85, 0.65);
    decision.right_min = min_scan_range_in_sector(-0.85, 0.65);
    if (!std::isfinite(decision.left_min) || !std::isfinite(decision.right_min)) {
      return std::nullopt;
    }

    decision.width = decision.left_min + decision.right_min;
    const double side_min = std::min(decision.left_min, decision.right_min);
    decision.hard_blocked =
      decision.width < corridor_hard_stop_width_ ||
      side_min < corridor_side_clearance_ + 0.12;
    decision.passable =
      !decision.hard_blocked &&
      decision.width >= corridor_min_passage_width_;
    if (decision.passable) {
      decision.signed_lateral_offset = clamp(
        (decision.left_min - decision.right_min) * 0.5,
        -corridor_max_lateral_offset_,
        corridor_max_lateral_offset_);
    }
    return decision;
  }

  bool mppi_rescue_handoff_is_clear(
    const RobotPose2D & pose,
    const MissionNode & target,
    bool allow_slow_handoff) const
  {
    const double front_min = min_scan_range_in_sector(0.0, 0.52);
    const double front_left_min = min_scan_range_in_sector(0.55, 0.45);
    const double front_right_min = min_scan_range_in_sector(-0.55, 0.45);
    const double target_heading = std::atan2(target.y - pose.y, target.x - pose.x);
    const double heading_error = std::abs(normalize_angle(target_heading - pose.yaw));
    const double required_front_clearance = allow_slow_handoff ?
      mppi_rescue_slow_handoff_front_clearance_ :
      mppi_rescue_handoff_front_clearance_;
    const double required_side_clearance = allow_slow_handoff ?
      std::max(0.26, mppi_rescue_handoff_side_clearance_ - 0.04) :
      mppi_rescue_handoff_side_clearance_;
    const double required_heading_error = allow_slow_handoff ? 1.15 : 0.85;

    const bool front_clear =
      std::isfinite(front_min) &&
      front_min >= required_front_clearance;
    const bool side_clear =
      (!std::isfinite(front_left_min) || front_left_min >= required_side_clearance) &&
      (!std::isfinite(front_right_min) || front_right_min >= required_side_clearance);
    return front_clear && side_clear && heading_error <= required_heading_error;
  }

  bool start_mppi_rescue(
    const RobotPose2D & pose,
    const MissionNode & target,
    const std::string & target_name,
    const std::optional<CorridorPassDecision> & corridor)
  {
    if (!mppi_follow_path_client_->wait_for_action_server(0s)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "MPPI rescue requested, but FollowPath action server is not available");
      return false;
    }

    if (target_name != mppi_rescue_attempt_target_) {
      mppi_rescue_attempt_target_ = target_name;
      mppi_rescue_attempt_count_ = 0;
    }
    ++mppi_rescue_attempt_count_;

    FollowPath::Goal goal;
    goal.path = make_mppi_rescue_path(pose, target, corridor);
    goal.controller_id = "FollowPath";
    goal.goal_checker_id = "general_goal_checker";
    const int64_t goal_id = ++mppi_rescue_goal_id_;
    mppi_active_goal_id_ = goal_id;

    rclcpp_action::Client<FollowPath>::SendGoalOptions options;
    options.goal_response_callback =
      [this, target_name, goal_id](FollowPathGoalHandle::SharedPtr goal_handle) {
        if (goal_id != mppi_active_goal_id_) {
          return;
        }
        if (!goal_handle) {
          mppi_rescue_active_ = false;
          mppi_active_goal_id_ = 0;
          RCLCPP_WARN(this->get_logger(), "MPPI rescue goal to %s was rejected", target_name.c_str());
          return;
        }
        mppi_goal_handle_ = goal_handle;
        RCLCPP_WARN(this->get_logger(), "MPPI rescue active toward %s", target_name.c_str());
      };
    options.result_callback =
      [this, target_name, goal_id](const FollowPathGoalHandle::WrappedResult & result) {
        if (goal_id != mppi_active_goal_id_) {
          return;
        }
        mppi_rescue_active_ = false;
        mppi_active_goal_id_ = 0;
        mppi_goal_handle_.reset();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "MPPI rescue finished toward %s", target_name.c_str());
        } else {
          RCLCPP_WARN(this->get_logger(), "MPPI rescue ended before reaching %s", target_name.c_str());
        }
      };

    mppi_rescue_active_ = true;
    mppi_rescue_started_at_ = this->now();
    mppi_follow_path_client_->async_send_goal(goal, options);
    return true;
  }

  void cancel_mppi_rescue()
  {
    if (mppi_goal_handle_) {
      mppi_follow_path_client_->async_cancel_goal(mppi_goal_handle_);
    }
    mppi_goal_handle_.reset();
    mppi_rescue_active_ = false;
    mppi_active_goal_id_ = 0;
  }

  void reset_mppi_rescue_attempts()
  {
    if (mppi_rescue_attempt_count_ > 0 && last_mppi_rescue_side_ != 0.0) {
      mppi_preferred_rescue_side_ = last_mppi_rescue_side_;
      mppi_preferred_rescue_lateral_offset_ = std::max(
        last_mppi_rescue_lateral_offset_,
        0.24);
    }
    mppi_rescue_attempt_target_.clear();
    mppi_rescue_attempt_count_ = 0;
    last_mppi_rescue_side_ = 0.0;
    last_mppi_rescue_lateral_offset_ = 0.0;
  }

  void return_to_loading(const std::vector<std::string> & path)
  {
    go_path(path);
    publish_rc_car_mode("stop");
    wait_stopped("loading", precision_wait_seconds_);
  }

  void wait_for_charger_request()
  {
    stop();
    publish_rc_car_mode("stop");
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
    publish_rc_car_mode("stop");
    go_path({"loading", "charger_entry"});
    l_shaped_charger_parking("charger_front");
    stop();
    publish_rc_car_mode("stop");
    RCLCPP_INFO(this->get_logger(), "Charger parking complete");
  }

  void handle_mission_command(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "start_charger_parking") {
      charger_parking_requested_ = true;
      RCLCPP_INFO(this->get_logger(), "Received charger parking command");
    }
  }

  void handle_rc_car_slot_status(const std_msgs::msg::String::SharedPtr msg)
  {
    last_rc_car_slot_status_ = msg->data;
  }

  void handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const rclcpp::Time stamp =
      msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
      this->now() :
      rclcpp::Time(msg->header.stamp);
    last_scan_ = *msg;
    local_planner_.update_scan(*msg, stamp);
  }

  void handle_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    local_planner_.update_map(*msg);
  }

  void wait_for_blocked_target_clear(
    const std::string & target_name,
    const MissionNode & target)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Task target %s is blocked; stopping until the obstacle is cleared",
      target_name.c_str());
    stop();

    rclcpp::Rate rate(10.0);
    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      stop();

      const auto pose = lookup_robot_pose();
      if (pose.has_value()) {
        const bool still_blocked = local_planner_.target_is_blocked(
          amr_topology::Pose2D{pose->x, pose->y, pose->yaw},
          amr_topology::Target2D{target.x, target.y},
          this->now());
        if (!still_blocked) {
          RCLCPP_INFO(
            this->get_logger(),
            "Task target %s is clear; resuming mission",
            target_name.c_str());
          return;
        }
      }

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
      command.angular.z = clamp(angular_gain_ * heading_error, -0.18, 0.18);

      cmd_pub_->publish(command);
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

      cmd_pub_->publish(command);
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

      cmd_pub_->publish(command);
      rate.sleep();
    }
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
      publish_leader_pose(pose, rclcpp::Time(transform.header.stamp));
      return pose;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for TF %s -> %s: %s",
        map_frame_.c_str(), base_frame_.c_str(), ex.what());
      return std::nullopt;
    }
  }

  void wait_at_slot(
    const std::string & slot_name,
    const std::string & exit_node_name)
  {
    wait_stopped(slot_name, wait_seconds_);
    rotate_to_face(exit_node_name);
  }

  void wait_at_slot_for_rc_car(
    const std::string & slot_name,
    const std::string & exit_node_name,
    const std::string & slot_key)
  {
    if (!rclcpp::ok()) {
      return;
    }

    rotate_to_face(exit_node_name);
    stop();

    const std::string arrived_status = slot_key + "_arrived";
    RCLCPP_INFO(
      this->get_logger(),
      "Waiting at %s until RC car reports %s",
      slot_name.c_str(),
      arrived_status.c_str());

    while (rclcpp::ok() && last_rc_car_slot_status_ != arrived_status) {
      rclcpp::spin_some(this->get_node_base_interface());
      stop();
      rclcpp::sleep_for(100ms);
    }

    stop();
    RCLCPP_INFO(
      this->get_logger(),
      "RC car reached %s staging point; holding at leader slot",
      slot_name.c_str());
  }

  void hold_at_leader_slot(const std::string & label)
  {
    stop();
    RCLCPP_INFO(this->get_logger(), "Holding at %s until shutdown", label.c_str());

    rclcpp::Rate rate(10.0);
    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      stop();
      rate.sleep();
    }
  }

  void wait_stopped(const std::string & label, double seconds)
  {
    if (!rclcpp::ok()) {
      return;
    }
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
    std::optional<double> start_yaw;
    bool rc_car_stop_sent_for_yaw_delta = false;
    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      const auto pose = lookup_robot_pose();
      if (!pose.has_value()) {
        rate.sleep();
        continue;
      }

      if (!start_yaw.has_value()) {
        start_yaw = pose->yaw;
      }

      if (
        stop_rc_car_on_a_leader_slot_ccw_yaw_ &&
        label == "a_leader_slot" &&
        !rc_car_stop_sent_for_yaw_delta)
      {
        const double ccw_yaw_delta = normalize_angle(pose->yaw - start_yaw.value());
        if (ccw_yaw_delta >= rc_car_a_slot_stop_yaw_delta_) {
          publish_rc_car_mode("stop");
          rc_car_stop_sent_for_yaw_delta = true;
          RCLCPP_INFO(
            this->get_logger(),
            "Stopping RC car after %.2f rad CCW rotation at %s",
            ccw_yaw_delta,
            label.c_str());
        }
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
      node.type = data["type"] ? data["type"].as<std::string>() : "waypoint";
      nodes_[name] = node;
    }
  }

  void stop()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }

  void publish_leader_pose(const RobotPose2D & pose, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = stamp;
    msg.pose.position.x = pose.x;
    msg.pose.position.y = pose.y;
    msg.pose.orientation = quaternion_from_yaw(pose.yaw);
    leader_pose_pub_->publish(msg);
  }

  void publish_rc_car_mode(const std::string & mode)
  {
    if (mode == last_rc_car_mode_) {
      return;
    }

    if (mode == "slot_wait_a" || mode == "slot_wait_b") {
      last_rc_car_slot_status_.clear();
    }

    std_msgs::msg::String msg;
    msg.data = mode;
    rc_car_mode_pub_->publish(msg);
    last_rc_car_mode_ = mode;
  }

  void republish_rc_car_mode()
  {
    if (last_rc_car_mode_.empty()) {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = last_rc_car_mode_;
    rc_car_mode_pub_->publish(msg);
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
  bool stop_rc_car_on_a_leader_slot_ccw_yaw_{false};
  double rc_car_a_slot_stop_yaw_delta_{M_PI / 3.0};
  double max_linear_speed_{0.14};
  double max_angular_speed_{0.45};
  double linear_gain_{0.70};
  double angular_gain_{1.1};
  double drive_heading_limit_{1.75};
  double curve_min_linear_speed_{0.03};
  double slot_departure_linear_speed_{0.065};
  double dock_tolerance_{0.04};
  double dock_lateral_tolerance_{0.08};
  double dock_longitudinal_tolerance_{0.04};
  double dock_max_reverse_speed_{0.055};
  double dock_max_reverse_angular_speed_{0.22};
  double dock_reverse_start_distance_{0.75};
  double dock_parking_lateral_offset_{0.15};
  double dock_linear_gain_{0.30};
  double dock_angular_gain_{0.8};
  double mppi_rescue_timeout_seconds_{18.0};
  double mppi_rescue_stop_and_plan_seconds_{3.0};
  double mppi_rescue_lateral_offset_{0.24};
  double mppi_rescue_forward_offset_{0.85};
  double mppi_rescue_min_handoff_seconds_{5.0};
  double mppi_rescue_handoff_front_clearance_{0.48};
  double mppi_rescue_handoff_side_clearance_{0.30};
  double mppi_rescue_slow_handoff_seconds_{8.0};
  double mppi_rescue_slow_handoff_front_clearance_{0.38};
  double mppi_rescue_side_escape_seconds_{3.0};
  double corridor_robot_width_{0.306};
  double corridor_side_clearance_{0.07};
  double corridor_min_passage_width_{0.45};
  double corridor_hard_stop_width_{0.40};
  double corridor_max_lateral_offset_{0.24};
  bool enable_lidar_safety_{true};
  bool enable_mppi_rescue_{true};
  bool enable_corridor_pass_{true};
  bool mppi_rescue_active_{false};
  bool mppi_stop_and_plan_active_{false};
  bool mppi_side_escape_active_{false};
  bool charger_parking_requested_{false};
  std::string last_rc_car_mode_;
  std::string last_rc_car_slot_status_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr leader_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rc_car_mode_pub_;
  rclcpp::TimerBase::SharedPtr rc_car_mode_timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rc_car_slot_status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp_action::Client<FollowPath>::SharedPtr mppi_follow_path_client_;
  FollowPathGoalHandle::SharedPtr mppi_goal_handle_;
  rclcpp::Time mppi_rescue_started_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time mppi_stop_and_plan_started_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time mppi_side_escape_started_at_{0, 0, RCL_ROS_TIME};
  int64_t mppi_rescue_goal_id_{0};
  int64_t mppi_active_goal_id_{0};
  std::string mppi_rescue_attempt_target_;
  int mppi_rescue_attempt_count_{0};
  mutable double last_mppi_rescue_side_{0.0};
  mutable double last_mppi_rescue_lateral_offset_{0.0};
  double mppi_preferred_rescue_side_{0.0};
  double mppi_preferred_rescue_lateral_offset_{0.0};
  std::optional<sensor_msgs::msg::LaserScan> last_scan_;
  amr_topology::LocalAStarPurePursuit local_planner_;
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
