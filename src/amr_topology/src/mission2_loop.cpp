#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
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
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateToPoseGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  MissionLoop()
  : Node("mission2_loop"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    const auto default_topology =
      ament_index_cpp::get_package_share_directory("amr_topology") + "/config/topology.yaml";

    this->declare_parameter<std::string>("topology_file", default_topology);
    this->declare_parameter<std::string>("start_mode", "mission");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("base_frame", "base_footprint");
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<std::string>("leader_pose_topic", "/turtlebot/pose");
    this->declare_parameter<std::string>("rc_car_mode_topic", "/rc_car/follower_mode");
    this->declare_parameter<std::string>("rc_car_slot_status_topic", "/rc_car/slot_wait_status");
    this->declare_parameter<std::string>("rc_car_odom_topic", "/rc_car/odom");
    this->declare_parameter<std::string>("aruco_enable_topic", "/aruco_marker/enable");
    this->declare_parameter<std::string>("aruco_target_topic", "/aruco_marker/target");
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
    this->declare_parameter<double>("rc_b_stop_x", 0.7269076704978943);
    this->declare_parameter<double>("rc_b_stop_y", -1.4004881381988525);
    this->declare_parameter<double>("rc_b_stop_tolerance", 0.12);
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
    this->declare_parameter<std::string>("navigate_to_pose_action", "navigate_to_pose");
    this->declare_parameter<double>("charger_nav2_server_wait_seconds", 5.0);
    this->declare_parameter<double>("charger_nav2_timeout_seconds", 45.0);
    this->declare_parameter<double>("charger_nav2_stuck_seconds", 4.0);
    this->declare_parameter<double>("charger_nav2_stuck_distance", 0.04);
    this->declare_parameter<double>("charger_nav2_stuck_front_clearance", 0.36);
    this->declare_parameter<std::string>(
      "charger_global_costmap_node",
      "/global_costmap/global_costmap");
    this->declare_parameter<std::string>(
      "charger_global_inflation_parameter",
      "inflation_layer.inflation_radius");
    this->declare_parameter<double>("charger_nav2_inflation_radius", 0.25);
    this->declare_parameter<double>("charger_nav2_parameter_timeout_seconds", 2.0);
    this->declare_parameter<double>("charger_entry_accept_tolerance", 0.35);
    this->declare_parameter<double>("charger_entry_handoff_tolerance", 0.85);
    this->declare_parameter<double>("charger_entry_hold_handoff_tolerance", 1.20);
    this->declare_parameter<double>("charger_entry_max_hold_seconds", 3.0);
    this->declare_parameter<double>("charger_entry_hold_movement_epsilon", 0.03);
    this->declare_parameter<int>("charger_corridor_escape_max_attempts", 3);
    this->declare_parameter<double>("charger_corridor_escape_backup_distance", 0.08);
    this->declare_parameter<double>("charger_corridor_escape_forward_distance", 0.45);
    this->declare_parameter<double>("charger_corridor_escape_linear_speed", 0.05);
    this->declare_parameter<double>("charger_corridor_escape_angular_speed", 0.24);
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
    this->declare_parameter<bool>("enable_a_slot_aruco_follow", false);
    this->declare_parameter<bool>("enable_b_slot_aruco_follow", true);
    this->declare_parameter<double>("aruco_search_angular_speed", 0.18);
    this->declare_parameter<double>("aruco_approach_linear_speed", 0.045);
    this->declare_parameter<double>("aruco_approach_angular_gain", 0.40);
    this->declare_parameter<double>("aruco_stop_distance", 0.28);
    this->declare_parameter<double>("b_slot_aruco_stop_distance", 0.36);
    this->declare_parameter<double>("b_slot_aruco_final_yaw", M_PI_2);
    this->declare_parameter<double>("aruco_detection_timeout_seconds", 0.6);
    topology_file_ = this->get_parameter("topology_file").as_string();
    start_mode_ = this->get_parameter("start_mode").as_string();
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
    rc_b_stop_x_ = this->get_parameter("rc_b_stop_x").as_double();
    rc_b_stop_y_ = this->get_parameter("rc_b_stop_y").as_double();
    rc_b_stop_tolerance_ = std::max(
      0.01, this->get_parameter("rc_b_stop_tolerance").as_double());
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
    charger_entry_accept_tolerance_ = std::max(
      0.05, this->get_parameter("charger_entry_accept_tolerance").as_double());
    charger_entry_handoff_tolerance_ = std::max(
      charger_entry_accept_tolerance_,
      this->get_parameter("charger_entry_handoff_tolerance").as_double());
    charger_entry_hold_handoff_tolerance_ = std::max(
      charger_entry_handoff_tolerance_,
      this->get_parameter("charger_entry_hold_handoff_tolerance").as_double());
    charger_entry_max_hold_seconds_ = std::max(
      0.1, this->get_parameter("charger_entry_max_hold_seconds").as_double());
    charger_entry_hold_movement_epsilon_ = std::max(
      0.005, this->get_parameter("charger_entry_hold_movement_epsilon").as_double());
    charger_corridor_escape_max_attempts_ = std::max(
      0, static_cast<int>(this->get_parameter("charger_corridor_escape_max_attempts").as_int()));
    charger_corridor_escape_backup_distance_ = std::max(
      0.0, this->get_parameter("charger_corridor_escape_backup_distance").as_double());
    charger_corridor_escape_forward_distance_ = std::max(
      0.0, this->get_parameter("charger_corridor_escape_forward_distance").as_double());
    charger_corridor_escape_linear_speed_ = std::max(
      0.01, this->get_parameter("charger_corridor_escape_linear_speed").as_double());
    charger_corridor_escape_angular_speed_ = std::max(
      0.0, this->get_parameter("charger_corridor_escape_angular_speed").as_double());
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
    enable_a_slot_aruco_follow_ =
      this->get_parameter("enable_a_slot_aruco_follow").as_bool();
    enable_b_slot_aruco_follow_ =
      this->get_parameter("enable_b_slot_aruco_follow").as_bool();
    aruco_search_angular_speed_ =
      this->get_parameter("aruco_search_angular_speed").as_double();
    aruco_approach_linear_speed_ =
      this->get_parameter("aruco_approach_linear_speed").as_double();
    aruco_approach_angular_gain_ =
      this->get_parameter("aruco_approach_angular_gain").as_double();
    aruco_stop_distance_ = this->get_parameter("aruco_stop_distance").as_double();
    b_slot_aruco_stop_distance_ =
      this->get_parameter("b_slot_aruco_stop_distance").as_double();
    b_slot_aruco_final_yaw_ =
      this->get_parameter("b_slot_aruco_final_yaw").as_double();
    aruco_detection_timeout_seconds_ =
      this->get_parameter("aruco_detection_timeout_seconds").as_double();
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
    aruco_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      this->get_parameter("aruco_enable_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local());
    aruco_target_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      this->get_parameter("aruco_target_topic").as_string(),
      10,
      std::bind(&MissionLoop::handle_aruco_target, this, std::placeholders::_1));
    rc_car_slot_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      this->get_parameter("rc_car_slot_status_topic").as_string(),
      10,
      std::bind(&MissionLoop::handle_rc_car_slot_status, this, std::placeholders::_1));
    rc_car_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      this->get_parameter("rc_car_odom_topic").as_string(),
      10,
      std::bind(&MissionLoop::handle_rc_car_odom, this, std::placeholders::_1));
    rc_car_mode_timer_ = this->create_wall_timer(
      500ms, std::bind(&MissionLoop::republish_rc_car_mode, this));
    mppi_follow_path_client_ = rclcpp_action::create_client<FollowPath>(
      this,
      this->get_parameter("mppi_follow_path_action").as_string());
    navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
      this,
      this->get_parameter("navigate_to_pose_action").as_string());

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
    if (start_mode_ == "charger_only") {
      RCLCPP_INFO(
        this->get_logger(),
        "Starting charger-only mode. Waiting for /mission_command start_charger_parking.");
      wait_for_charger_request();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Starting B-only mission loop");

    for (int cycle = 1; cycle <= repeat_count_ && rclcpp::ok(); ++cycle) {
      RCLCPP_INFO(this->get_logger(), "Cycle %d/%d: moving to B", cycle, repeat_count_);
      rc_b_stop_monitoring_ = true;
      rc_b_stop_sent_ = false;
      publish_rc_car_mode("follow");
      go_path({"loading", "intersection_2", "b_entry", "b_leader_slot"});
      rc_b_stop_monitoring_ = false;
      if (handle_charger_parking_request()) {
        return;
      }
      if (!rclcpp::ok()) {
        return;
      }
      publish_rc_car_mode("stop");
      wait_stopped("B leader slot", wait_seconds_);
      rotate_to_face("b_entry");
      if (enable_b_slot_aruco_follow_) {
        aruco_follow_after_leader_slot("b_leader_slot", "B", b_slot_aruco_stop_distance_);
      }
      if (handle_charger_parking_request()) {
        return;
      }
      if (!rclcpp::ok()) {
        return;
      }
      hold_at_leader_slot("B leader slot");
      if (handle_charger_parking_request()) {
        return;
      }
    }

    publish_rc_car_mode("stop");
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
      if (charger_parking_interrupt_requested()) {
        stop_current_action_for_charger_interrupt();
        return;
      }
      publish_scheduled_rc_car_mode();
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
          if (charger_parking_interrupt_requested()) {
            stop_current_action_for_charger_interrupt();
            return;
          }
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
    return node_name == "b_leader_slot";
  }

  bool is_entry_node(const std::string & node_name) const
  {
    return node_name == "b_entry";
  }

  bool is_skippable_node(const std::string & node_name, const MissionNode & node) const
  {
    if (
      node_name == "intersection_1" ||
      node_name == "intersection_2" ||
      node_name == "a_entry" ||
      node_name == "b_entry")
    {
      return true;
    }
    return node.type == "intersection" || node.type == "area_entry" ||
      node.type == "standby" || node.type == "waypoint";
  }

  bool is_blocking_target_node(const std::string & node_name) const
  {
    return node_name == "b_leader_slot";
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

  bool charger_fast_handoff_reached(const MissionNode & charger_entry)
  {
    const auto pose = lookup_robot_pose();
    if (!pose.has_value()) {
      return false;
    }
    const double distance = distance_to_target(pose.value(), charger_entry);
    if (distance > charger_entry_handoff_tolerance_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Waiting for charger_entry handoff: distance=%.3f tolerance=%.3f",
        distance, charger_entry_handoff_tolerance_);
      return false;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Accepting charger_entry handoff by TF distance: distance=%.3f tolerance=%.3f",
      distance,
      charger_entry_handoff_tolerance_);
    return true;
  }

  bool charger_entry_hold_handoff_reached(
    const MissionNode & charger_entry,
    const RobotPose2D & current_pose,
    const RobotPose2D & hold_reference_pose,
    const rclcpp::Time & hold_started_at)
  {
    const double distance = distance_to_target(current_pose, charger_entry);
    if (distance > charger_entry_hold_handoff_tolerance_) {
      return false;
    }

    const double movement = std::hypot(
      current_pose.x - hold_reference_pose.x,
      current_pose.y - hold_reference_pose.y);
    if (movement > charger_entry_hold_movement_epsilon_) {
      return false;
    }

    const double held_seconds = (this->now() - hold_started_at).seconds();
    if (held_seconds < charger_entry_max_hold_seconds_) {
      return false;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Charger entry hold reached %.2fs near entry: distance=%.3f tolerance=%.3f movement=%.3f",
      held_seconds,
      distance,
      charger_entry_hold_handoff_tolerance_,
      movement);
    return true;
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

    handle_charger_parking_request();
  }

  void handle_mission_command(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "start_charger_parking") {
      if (charger_parking_active_) {
        RCLCPP_WARN(this->get_logger(), "Ignoring charger parking command: already active");
        return;
      }
      charger_parking_requested_ = true;
      RCLCPP_INFO(this->get_logger(), "Received charger parking command");
    }
  }

  bool charger_parking_interrupt_requested() const
  {
    return charger_parking_requested_ && !charger_parking_active_;
  }

  void stop_current_action_for_charger_interrupt()
  {
    clear_scheduled_rc_car_mode();
    cancel_mppi_rescue();
    local_planner_.reset();
    set_aruco_detection_enabled(false);
    stop();
    publish_rc_car_mode("stop", true);
  }

  bool handle_charger_parking_request()
  {
    if (!charger_parking_interrupt_requested()) {
      return false;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Interrupting current mission for charger parking");
    charger_parking_requested_ = false;
    charger_parking_active_ = true;
    charger_corridor_escape_attempts_ = 0;
    rc_b_stop_monitoring_ = false;
    rc_b_stop_sent_ = false;
    stop_current_action_for_charger_interrupt();
    const bool charger_inflation_override_applied = apply_charger_nav2_inflation_override();

    RCLCPP_INFO(
      this->get_logger(),
      "Navigating to charger_entry with Nav2 NavigateToPose");
    const bool reached_charger_entry = navigate_to_charger_entry();
    if (charger_inflation_override_applied) {
      restore_charger_nav2_inflation();
    }
    if (rclcpp::ok() && reached_charger_entry) {
      l_shaped_charger_parking("charger_front");
    } else if (rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Charger entry navigation failed; skipping L-shaped parking");
    }
    stop();
    publish_rc_car_mode("stop", true);
    charger_parking_active_ = false;
    RCLCPP_INFO(this->get_logger(), "Charger parking complete");
    return true;
  }

  bool apply_charger_nav2_inflation_override()
  {
    const auto node_name = this->get_parameter("charger_global_costmap_node").as_string();
    const auto parameter_name =
      this->get_parameter("charger_global_inflation_parameter").as_string();
    const double override_radius =
      this->get_parameter("charger_nav2_inflation_radius").as_double();
    const double timeout_seconds = std::max(
      0.1,
      this->get_parameter("charger_nav2_parameter_timeout_seconds").as_double());

    auto client = std::make_shared<rclcpp::SyncParametersClient>(this, node_name);
    if (!client->wait_for_service(std::chrono::duration<double>(timeout_seconds))) {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not apply charger Nav2 inflation override: parameter service %s unavailable",
        node_name.c_str());
      return false;
    }

    try {
      charger_nav2_inflation_restore_value_ = client->get_parameter<double>(parameter_name);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not read charger Nav2 inflation parameter %s/%s: %s",
        node_name.c_str(),
        parameter_name.c_str(),
        e.what());
      charger_nav2_inflation_restore_value_.reset();
      return false;
    }

    const auto results = client->set_parameters(
      {rclcpp::Parameter(parameter_name, override_radius)});
    if (results.empty() || !results.front().successful) {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not set charger Nav2 inflation parameter %s/%s to %.3f: %s",
        node_name.c_str(),
        parameter_name.c_str(),
        override_radius,
        results.empty() ? "no result" : results.front().reason.c_str());
      charger_nav2_inflation_restore_value_.reset();
      return false;
    }

    charger_nav2_inflation_override_active_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Applied charger-only Nav2 inflation override: %s/%s %.3f -> %.3f",
      node_name.c_str(),
      parameter_name.c_str(),
      *charger_nav2_inflation_restore_value_,
      override_radius);
    return true;
  }

  void restore_charger_nav2_inflation()
  {
    if (!charger_nav2_inflation_override_active_ ||
      !charger_nav2_inflation_restore_value_.has_value())
    {
      return;
    }

    const auto node_name = this->get_parameter("charger_global_costmap_node").as_string();
    const auto parameter_name =
      this->get_parameter("charger_global_inflation_parameter").as_string();
    const double timeout_seconds = std::max(
      0.1,
      this->get_parameter("charger_nav2_parameter_timeout_seconds").as_double());
    const double restore_radius = *charger_nav2_inflation_restore_value_;

    auto client = std::make_shared<rclcpp::SyncParametersClient>(this, node_name);
    if (!client->wait_for_service(std::chrono::duration<double>(timeout_seconds))) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to restore charger Nav2 inflation: parameter service %s unavailable",
        node_name.c_str());
      return;
    }

    const auto results = client->set_parameters(
      {rclcpp::Parameter(parameter_name, restore_radius)});
    if (results.empty() || !results.front().successful) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to restore charger Nav2 inflation parameter %s/%s to %.3f: %s",
        node_name.c_str(),
        parameter_name.c_str(),
        restore_radius,
        results.empty() ? "no result" : results.front().reason.c_str());
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Restored charger-only Nav2 inflation override: %s/%s -> %.3f",
      node_name.c_str(),
      parameter_name.c_str(),
      restore_radius);
    charger_nav2_inflation_override_active_ = false;
    charger_nav2_inflation_restore_value_.reset();
  }

  bool navigate_to_charger_entry()
  {
    const auto node_it = nodes_.find("charger_entry");
    if (node_it == nodes_.end()) {
      RCLCPP_ERROR(this->get_logger(), "No charger_entry node in topology");
      return false;
    }

    const double server_wait_seconds = std::max(
      0.0,
      this->get_parameter("charger_nav2_server_wait_seconds").as_double());
    const double timeout_seconds = std::max(
      1.0,
      this->get_parameter("charger_nav2_timeout_seconds").as_double());
    const double stuck_seconds = std::max(
      1.0,
      this->get_parameter("charger_nav2_stuck_seconds").as_double());
    const double stuck_distance = std::max(
      0.01,
      this->get_parameter("charger_nav2_stuck_distance").as_double());
    const double stuck_front_clearance = std::max(
      0.05,
      this->get_parameter("charger_nav2_stuck_front_clearance").as_double());

    const auto wait_started_at = this->now();
    rclcpp::Rate wait_rate(10.0);
    while (
      rclcpp::ok() &&
      !navigate_to_pose_client_->wait_for_action_server(0s) &&
      (this->now() - wait_started_at).seconds() <= server_wait_seconds)
    {
      rclcpp::spin_some(this->get_node_base_interface());
      publish_rc_car_mode("stop", true);
      wait_rate.sleep();
    }

    if (!navigate_to_pose_client_->wait_for_action_server(0s)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "NavigateToPose action server is not available");
      return false;
    }

    const auto & charger_entry = node_it->second;
    if (is_near_charger_entry(charger_entry, "before NavigateToPose")) {
      stop();
      return true;
    }

    NavigateToPose::Goal goal;
    goal.pose = make_pose_stamped(charger_entry.x, charger_entry.y, charger_entry.yaw);

    RCLCPP_WARN(
      this->get_logger(),
      "Sending NavigateToPose goal to charger_entry: x=%.3f y=%.3f yaw=%.2f",
      charger_entry.x,
      charger_entry.y,
      charger_entry.yaw);

    auto goal_future = navigate_to_pose_client_->async_send_goal(goal);
    while (
      rclcpp::ok() &&
      goal_future.wait_for(0ms) != std::future_status::ready)
    {
      rclcpp::spin_some(this->get_node_base_interface());
      publish_rc_car_mode("stop", true);
      rclcpp::sleep_for(50ms);
    }

    if (!rclcpp::ok()) {
      return false;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "NavigateToPose goal was rejected");
      return false;
    }

    auto result_future = navigate_to_pose_client_->async_get_result(goal_handle);
    const auto nav_started_at = this->now();
    auto last_progress_pose = lookup_robot_pose();
    auto last_progress_at = this->now();
    std::optional<RobotPose2D> charger_entry_hold_reference_pose;
    std::optional<rclcpp::Time> charger_entry_hold_started_at;
    while (
      rclcpp::ok() &&
      result_future.wait_for(0ms) != std::future_status::ready)
    {
      rclcpp::spin_some(this->get_node_base_interface());
      publish_rc_car_mode("stop", true);
      const auto current_pose = lookup_robot_pose();
      if (current_pose.has_value()) {
        if (charger_fast_handoff_reached(charger_entry)) {
          RCLCPP_INFO(
            this->get_logger(),
            "Fast handoff at charger_entry; canceling NavigateToPose for L-shaped parking");
          navigate_to_pose_client_->async_cancel_goal(goal_handle);
          stop();
          return true;
        }
        const double distance_to_entry = distance_to_target(current_pose.value(), charger_entry);
        if (distance_to_entry <= charger_entry_hold_handoff_tolerance_) {
          if (!charger_entry_hold_reference_pose.has_value()) {
            charger_entry_hold_reference_pose = current_pose;
            charger_entry_hold_started_at = this->now();
          } else {
            const double movement_from_hold_reference = std::hypot(
              current_pose->x - charger_entry_hold_reference_pose->x,
              current_pose->y - charger_entry_hold_reference_pose->y);
            if (movement_from_hold_reference > charger_entry_hold_movement_epsilon_) {
              charger_entry_hold_reference_pose = current_pose;
              charger_entry_hold_started_at = this->now();
            } else if (
              charger_entry_hold_started_at.has_value() &&
              charger_entry_hold_handoff_reached(
                charger_entry,
                current_pose.value(),
                charger_entry_hold_reference_pose.value(),
                charger_entry_hold_started_at.value()))
            {
              RCLCPP_WARN(
                this->get_logger(),
                "Canceling NavigateToPose after charger_entry hold for L-shaped parking");
              navigate_to_pose_client_->async_cancel_goal(goal_handle);
              stop();
              return true;
            }
          }
        } else {
          charger_entry_hold_reference_pose.reset();
          charger_entry_hold_started_at.reset();
        }
        if (
          !last_progress_pose.has_value() ||
          std::hypot(
            current_pose->x - last_progress_pose->x,
            current_pose->y - last_progress_pose->y) >= stuck_distance)
        {
          last_progress_pose = current_pose;
          last_progress_at = this->now();
        } else if ((this->now() - last_progress_at).seconds() >= stuck_seconds) {
          const double front_min = min_scan_range_in_sector(0.0, 0.70);
          if (std::isfinite(front_min) && front_min < stuck_front_clearance) {
            RCLCPP_ERROR(
              this->get_logger(),
              "NavigateToPose appears stuck near obstacle: moved < %.3fm for %.1fs, front=%.3f. Starting charger corridor escape immediately.",
              stuck_distance, stuck_seconds, front_min);
            navigate_to_pose_client_->async_cancel_goal(goal_handle);
            if (try_charger_corridor_escape(charger_entry, "after NavigateToPose stuck watchdog")) {
              return navigate_to_charger_entry();
            }
            return false;
          }
        }
      }
      if ((this->now() - nav_started_at).seconds() > timeout_seconds) {
        RCLCPP_ERROR(
          this->get_logger(),
          "NavigateToPose timed out after %.1f seconds",
          timeout_seconds);
        navigate_to_pose_client_->async_cancel_goal(goal_handle);
        if (is_near_charger_entry(charger_entry, "after NavigateToPose timeout")) {
          stop();
          return true;
        }
        if (try_charger_corridor_escape(charger_entry, "after NavigateToPose timeout")) {
          return navigate_to_charger_entry();
        }
        return false;
      }
      rclcpp::sleep_for(50ms);
    }

    if (!rclcpp::ok()) {
      return false;
    }

    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(
        this->get_logger(),
        "NavigateToPose failed with result code %d",
        static_cast<int>(result.code));
      if (is_near_charger_entry(charger_entry, "after NavigateToPose failure")) {
        stop();
        return true;
      }
      if (try_charger_corridor_escape(charger_entry, "after NavigateToPose failure")) {
        return navigate_to_charger_entry();
      }
      return false;
    }

    stop();
    RCLCPP_INFO(this->get_logger(), "Reached charger_entry with Nav2");
    return true;
  }

  void handle_rc_car_slot_status(const std_msgs::msg::String::SharedPtr msg)
  {
    last_rc_car_slot_status_ = msg->data;
  }

  void handle_rc_car_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (rc_b_stop_monitoring_ && !rc_b_stop_sent_) {
      const double dx = msg->pose.pose.position.x - rc_b_stop_x_;
      const double dy = msg->pose.pose.position.y - rc_b_stop_y_;
      const double distance = std::hypot(dx, dy);
      if (distance <= rc_b_stop_tolerance_) {
        rc_b_stop_sent_ = true;
        publish_rc_car_mode("stop");
        RCLCPP_INFO(
          this->get_logger(),
          "RC car reached rc_b_stop: distance=%.3f tolerance=%.3f; stopping RC car",
          distance,
          rc_b_stop_tolerance_);
      }
    }

  }

  void handle_aruco_target(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    last_aruco_target_ = *msg;
    last_aruco_target_time_ = this->now();
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
      if (charger_parking_interrupt_requested()) {
        stop_current_action_for_charger_interrupt();
        return;
      }
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
    publish_rc_car_mode("stop", true);

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
      RCLCPP_ERROR(
        this->get_logger(),
        "Cannot start L-shaped charger parking: robot pose unavailable");
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
      if (charger_parking_interrupt_requested()) {
        stop_current_action_for_charger_interrupt();
        return;
      }
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
      if (charger_parking_interrupt_requested()) {
        stop_current_action_for_charger_interrupt();
        return;
      }
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
      if (charger_parking_interrupt_requested()) {
        stop_current_action_for_charger_interrupt();
        return;
      }
      const auto pose = lookup_robot_pose();
      if (!pose.has_value()) {
        rate.sleep();
        continue;
      }

      const double dx = dock_target.x - pose->x;
      const double dy = dock_target.y - pose->y;
      const double distance = std::hypot(dx, dy);
      const double cos_yaw = std::cos(dock_target.yaw);
      const double sin_yaw = std::sin(dock_target.yaw);
      const double longitudinal_error = dx * cos_yaw + dy * sin_yaw;
      const double reverse_remaining = -longitudinal_error;
      const double lateral_error = dx * -sin_yaw + dy * cos_yaw;
      const double yaw_error = normalize_angle(dock_target.yaw - pose->yaw);

      if (
        distance <= dock_tolerance_ ||
        (reverse_remaining <= dock_longitudinal_tolerance_ &&
        std::abs(lateral_error) <= dock_lateral_tolerance_))
      {
        stop();
        RCLCPP_INFO(
          this->get_logger(),
          "Docking target reached. distance=%.3f longitudinal=%.3f lateral=%.3f yaw_error=%.3f",
          distance,
          reverse_remaining,
          lateral_error,
          yaw_error);
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
      const double reverse_speed = std::min(
        dock_max_reverse_speed_,
        std::max(0.0, dock_linear_gain_ * reverse_remaining));
      if (reverse_speed <= 1e-4) {
        stop();
        return;
      }
      command.linear.x = -reverse_speed;
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

  bool is_near_charger_entry(const MissionNode & charger_entry, const std::string & context)
  {
    const auto pose = lookup_robot_pose();
    if (!pose.has_value()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Cannot check charger_entry proximity %s: robot pose unavailable",
        context.c_str());
      return false;
    }

    const double distance = distance_to_target(pose.value(), charger_entry);
    if (distance > charger_entry_accept_tolerance_) {
      RCLCPP_WARN(
        this->get_logger(),
        "charger_entry proximity check %s failed: distance=%.3f tolerance=%.3f",
        context.c_str(), distance, charger_entry_accept_tolerance_);
      return false;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Accepting charger_entry by TF proximity %s: distance=%.3f tolerance=%.3f",
      context.c_str(), distance, charger_entry_accept_tolerance_);
    return true;
  }

  bool try_charger_corridor_escape(const MissionNode & charger_entry, const std::string & context)
  {
    if (charger_corridor_escape_attempts_ >= charger_corridor_escape_max_attempts_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Charger corridor escape skipped %s: max attempts reached (%d)",
        context.c_str(), charger_corridor_escape_max_attempts_);
      return false;
    }
    if (!last_scan_.has_value()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Charger corridor escape skipped %s: scan unavailable",
        context.c_str());
      return false;
    }

    ++charger_corridor_escape_attempts_;
    const double front_min = min_scan_range_in_sector(0.0, 0.70);
    const double front_left_min = min_scan_range_in_sector(0.45, 0.55);
    const double front_right_min = min_scan_range_in_sector(-0.45, 0.55);
    const double left_min = min_scan_range_in_sector(0.85, 0.65);
    const double right_min = min_scan_range_in_sector(-0.85, 0.65);
    RCLCPP_WARN(
      this->get_logger(),
      "Starting charger corridor escape %s: attempt %d/%d front=%.3f front_left=%.3f front_right=%.3f left=%.3f right=%.3f",
      context.c_str(), charger_corridor_escape_attempts_, charger_corridor_escape_max_attempts_,
      front_min, front_left_min, front_right_min, left_min, right_min);

    stop();
    publish_rc_car_mode("stop", true);
    if (std::isfinite(front_min) && front_min < mppi_rescue_handoff_front_clearance_) {
      drive_charger_escape_distance(
        -charger_corridor_escape_linear_speed_,
        0.0,
        charger_corridor_escape_backup_distance_,
        "charger corridor short backup");
      wait_stopped("charger corridor scan settle", 0.20);
    }

    double escaped_distance = 0.0;
    double last_side = 0.0;
    constexpr double kEscapeStepDistance = 0.15;
    constexpr double kFrontStopDistance = 0.28;
    constexpr double kFrontBackupDistance = 0.34;
    for (int step = 0; rclcpp::ok() && step < 5 && escaped_distance < charger_corridor_escape_forward_distance_; ++step) {
      const double post_front_min = min_scan_range_in_sector(0.0, 0.70);
      const double post_front_left_min = min_scan_range_in_sector(0.45, 0.55);
      const double post_front_right_min = min_scan_range_in_sector(-0.45, 0.55);
      const double post_left_min = min_scan_range_in_sector(0.85, 0.65);
      const double post_right_min = min_scan_range_in_sector(-0.85, 0.65);

      if (std::isfinite(post_front_min) && post_front_min < kFrontBackupDistance) {
        RCLCPP_WARN(
          this->get_logger(),
          "Charger escape front too close before step %d: front=%.3f; backing up shortly",
          step + 1, post_front_min);
        drive_charger_escape_distance(
          -charger_corridor_escape_linear_speed_,
          0.0,
          charger_corridor_escape_backup_distance_,
          "charger escape short backup before step");
        wait_stopped("charger escape scan settle", 0.15);
      }

      const auto clearance_score = [](double front_side, double side) {
        double score = 0.0;
        bool valid = false;
        if (std::isfinite(front_side)) {
          score += front_side;
          valid = true;
          if (front_side < 0.30) {
            score -= 0.60;
          }
        }
        if (std::isfinite(side)) {
          score += 0.35 * side;
          valid = true;
        }
        return valid ? score : -1.0;
      };
      double left_score = clearance_score(post_front_left_min, post_left_min);
      double right_score = clearance_score(post_front_right_min, post_right_min);
      if (left_score < 0.0 && right_score < 0.0) {
        RCLCPP_ERROR(this->get_logger(), "Charger escape failed: no usable front-side scan");
        return false;
      }
      if (left_score >= 0.0 && right_score >= 0.0 &&
        std::abs(left_score - right_score) < 0.08 && last_side != 0.0)
      {
        if (last_side > 0.0) {
          left_score -= 0.20;
        } else {
          right_score -= 0.20;
        }
      }

      const double side = left_score >= right_score ? 1.0 : -1.0;
      last_side = side;
      const double step_distance = std::min(
        kEscapeStepDistance,
        charger_corridor_escape_forward_distance_ - escaped_distance);
      RCLCPP_WARN(
        this->get_logger(),
        "Charger escape step %d chooses %s: left_score=%.3f right_score=%.3f front=%.3f front_left=%.3f front_right=%.3f left=%.3f right=%.3f step=%.3f",
        step + 1,
        side > 0.0 ? "left" : "right",
        left_score, right_score, post_front_min, post_front_left_min, post_front_right_min,
        post_left_min, post_right_min, step_distance);

      const bool completed_step = drive_charger_escape_distance(
        charger_corridor_escape_linear_speed_,
        side * charger_corridor_escape_angular_speed_,
        step_distance,
        "charger corridor step escape",
        kFrontStopDistance);
      if (completed_step) {
        escaped_distance += step_distance;
      } else {
        wait_stopped("charger escape blocked scan settle", 0.15);
      }
    }

    if (is_near_charger_entry(charger_entry, "after charger corridor escape")) {
      return true;
    }
    return rclcpp::ok() && escaped_distance > 0.05;
  }

  bool drive_charger_escape_distance(
    double linear_speed,
    double angular_speed,
    double distance,
    const std::string & label,
    double front_stop_distance = 0.0)
  {
    if (!rclcpp::ok() || distance <= 0.0 || std::abs(linear_speed) <= 1e-4) {
      return false;
    }

    const auto start_pose = lookup_robot_pose();
    const auto started_at = this->now();
    const double timeout_seconds = std::max(1.0, distance / std::abs(linear_speed) + 1.0);
    RCLCPP_WARN(
      this->get_logger(),
      "Driving %s: linear=%.3f angular=%.3f distance=%.3f timeout=%.1f",
      label.c_str(), linear_speed, angular_speed, distance, timeout_seconds);

    rclcpp::Rate rate(20.0);
    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      if (start_pose.has_value()) {
        const auto pose = lookup_robot_pose();
        if (pose.has_value()) {
          const double traveled = std::hypot(pose->x - start_pose->x, pose->y - start_pose->y);
          if (traveled >= distance) {
            break;
          }
        }
      }
      if ((this->now() - started_at).seconds() >= timeout_seconds) {
        break;
      }
      if (linear_speed > 0.0 && front_stop_distance > 0.0) {
        const double front_min = min_scan_range_in_sector(0.0, 0.70);
        if (std::isfinite(front_min) && front_min < front_stop_distance) {
          RCLCPP_WARN(
            this->get_logger(),
            "Stopping %s early: front=%.3f stop=%.3f",
            label.c_str(), front_min, front_stop_distance);
          stop();
          return false;
        }
      }

      geometry_msgs::msg::Twist command;
      command.linear.x = linear_speed;
      command.angular.z = angular_speed;
      cmd_pub_->publish(command);
      publish_rc_car_mode("stop", true);
      rate.sleep();
    }
    stop();
    return true;
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
      if (charger_parking_interrupt_requested()) {
        stop_current_action_for_charger_interrupt();
        return;
      }
      stop();
      rclcpp::sleep_for(100ms);
    }

    stop();
    RCLCPP_INFO(
      this->get_logger(),
      "RC car reached %s staging point; holding at leader slot",
      slot_name.c_str());
  }

  void wait_for_rc_car_status(
    const std::string & expected_status,
    const std::string & label)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Waiting for %s to report %s",
      label.c_str(),
      expected_status.c_str());

    while (rclcpp::ok() && last_rc_car_slot_status_ != expected_status) {
      rclcpp::spin_some(this->get_node_base_interface());
      if (charger_parking_interrupt_requested()) {
        stop_current_action_for_charger_interrupt();
        return;
      }
      stop();
      rclcpp::sleep_for(100ms);
    }

    stop();
    RCLCPP_INFO(
      this->get_logger(),
      "%s reported %s",
      label.c_str(),
      expected_status.c_str());
  }

  void hold_at_leader_slot(const std::string & label)
  {
    stop();
    RCLCPP_INFO(this->get_logger(), "Holding at %s until shutdown", label.c_str());

    rclcpp::Rate rate(10.0);
    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      if (charger_parking_interrupt_requested()) {
        stop_current_action_for_charger_interrupt();
        return;
      }
      stop();
      rate.sleep();
    }
  }

  void aruco_follow_after_leader_slot(
    const std::string & leader_slot_name,
    const std::string & slot_label,
    double stop_distance)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Starting %s leader slot ArUco follow mission",
      slot_label.c_str());
    set_aruco_detection_enabled(true);
    last_aruco_target_.reset();

    rclcpp::Rate rate(20.0);
    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      if (charger_parking_interrupt_requested()) {
        stop_current_action_for_charger_interrupt();
        return;
      }

      if (!aruco_target_is_fresh()) {
        geometry_msgs::msg::Twist command;
        command.angular.z = std::abs(aruco_search_angular_speed_);
        cmd_pub_->publish(command);
        rate.sleep();
        continue;
      }

      stop();
      RCLCPP_INFO(this->get_logger(), "RC car ArUco marker detected. Approaching marker");
      break;
    }

    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      if (charger_parking_interrupt_requested()) {
        stop_current_action_for_charger_interrupt();
        return;
      }

      if (!aruco_target_is_fresh()) {
        geometry_msgs::msg::Twist command;
        command.angular.z = std::abs(aruco_search_angular_speed_) * 0.5;
        cmd_pub_->publish(command);
        rate.sleep();
        continue;
      }

      const auto target = last_aruco_target_.value();
      const double distance = target.point.y;
      if (distance <= stop_distance) {
        stop();
        RCLCPP_INFO(
          this->get_logger(),
          "Reached %s ArUco stop distance %.2fm <= %.2fm. Stopping and disabling ArUco detection",
          slot_label.c_str(),
          distance,
          stop_distance);
        set_aruco_detection_enabled(false);
        rotate_to_node_yaw(leader_slot_name);
        if (charger_parking_interrupt_requested()) {
          return;
        }
        publish_rc_car_mode("turn_ccw_90");
        wait_for_rc_car_status("turn_ccw_90_done", "RC car ArUco turn");
        if (charger_parking_interrupt_requested()) {
          return;
        }
        publish_rc_car_mode("stop");
        rotate_ccw_relative(M_PI, "after RC car 90 degree turn");
        if (charger_parking_interrupt_requested()) {
          return;
        }
        drive_straight_distance(-0.075);
        if (charger_parking_interrupt_requested()) {
          return;
        }
        stop();
        publish_rc_car_mode("stop");
        RCLCPP_INFO(
          this->get_logger(),
          "First ArUco mission complete after 180 degree turn and 7.5cm backward move; stopping");
        return;
      }

      geometry_msgs::msg::Twist command;
      command.linear.x = aruco_approach_linear_speed_;
      command.angular.z = clamp(
        -aruco_approach_angular_gain_ * target.point.x,
        -std::abs(max_angular_speed_),
        std::abs(max_angular_speed_));
      cmd_pub_->publish(command);
      rate.sleep();
    }

    stop();
    set_aruco_detection_enabled(false);
  }

  bool aruco_target_is_fresh() const
  {
    return last_aruco_target_.has_value() &&
      (this->now() - last_aruco_target_time_).seconds() <= aruco_detection_timeout_seconds_;
  }

  void aruco_follow_after_driving_toward_node(
    const std::string & target_node_name,
    const std::string & label)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Starting %s ArUco follow mission toward %s",
      label.c_str(),
      target_node_name.c_str());
    set_aruco_detection_enabled(true);
    last_aruco_target_.reset();
    rotate_to_face(target_node_name);
    if (charger_parking_interrupt_requested()) {
      return;
    }

    rclcpp::Rate rate(20.0);
    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      if (charger_parking_interrupt_requested()) {
        stop_current_action_for_charger_interrupt();
        return;
      }
      if (aruco_target_is_fresh()) {
        stop();
        RCLCPP_INFO(
          this->get_logger(),
          "%s ArUco marker detected while driving toward %s. Approaching marker",
          label.c_str(),
          target_node_name.c_str());
        break;
      }

      const auto pose = lookup_robot_pose();
      if (!pose.has_value()) {
        rate.sleep();
        continue;
      }

      const auto & target_node = nodes_.at(target_node_name);
      const double distance = distance_to_target(pose.value(), target_node);
      const double target_heading = std::atan2(target_node.y - pose->y, target_node.x - pose->x);
      const double heading_error = normalize_angle(target_heading - pose->yaw);

      geometry_msgs::msg::Twist command;
      command.linear.x = clamp(linear_gain_ * distance, 0.025, 0.08);
      command.angular.z = clamp(angular_gain_ * heading_error, -0.18, 0.18);
      cmd_pub_->publish(command);
      rate.sleep();
    }

    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      if (charger_parking_interrupt_requested()) {
        stop_current_action_for_charger_interrupt();
        return;
      }

      if (!aruco_target_is_fresh()) {
        geometry_msgs::msg::Twist command;
        command.angular.z = std::abs(aruco_search_angular_speed_) * 0.5;
        cmd_pub_->publish(command);
        rate.sleep();
        continue;
      }

      const auto target = last_aruco_target_.value();
      const double distance = target.point.y;
      if (distance <= aruco_stop_distance_) {
        stop();
        RCLCPP_INFO(
          this->get_logger(),
          "Reached %s ArUco stop distance %.2fm. Stopping and disabling ArUco detection",
          label.c_str(),
          distance);
        set_aruco_detection_enabled(false);
        rotate_to_node_yaw(target_node_name);
        if (charger_parking_interrupt_requested()) {
          return;
        }
        publish_rc_car_mode("turn_ccw_90");
        wait_for_rc_car_status("turn_ccw_90_done", "RC car post ArUco turn");
        if (charger_parking_interrupt_requested()) {
          return;
        }
        publish_rc_car_mode("stop");
        rotate_ccw_relative(M_PI, "after post ArUco RC car turn");
        stop();
        return;
      }

      geometry_msgs::msg::Twist command;
      command.linear.x = aruco_approach_linear_speed_;
      command.angular.z = clamp(
        -aruco_approach_angular_gain_ * target.point.x,
        -std::abs(max_angular_speed_),
        std::abs(max_angular_speed_));
      cmd_pub_->publish(command);
      rate.sleep();
    }

    stop();
    set_aruco_detection_enabled(false);
  }

  void set_aruco_detection_enabled(bool enabled)
  {
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    aruco_enable_pub_->publish(msg);
  }

  void drive_straight_distance(double distance)
  {
    if (!rclcpp::ok() || std::abs(distance) <= 0.0) {
      return;
    }

    const auto start_pose = lookup_robot_pose();
    if (!start_pose.has_value()) {
      return;
    }

    const double target_distance = std::abs(distance);
    const double direction = distance >= 0.0 ? 1.0 : -1.0;
    RCLCPP_INFO(this->get_logger(), "Driving straight %.3fm", distance);
    rclcpp::Rate rate(20.0);
    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      if (charger_parking_interrupt_requested()) {
        stop_current_action_for_charger_interrupt();
        return;
      }
      const auto pose = lookup_robot_pose();
      if (!pose.has_value()) {
        rate.sleep();
        continue;
      }

      const double traveled = std::hypot(pose->x - start_pose->x, pose->y - start_pose->y);
      if (traveled >= target_distance) {
        stop();
        return;
      }

      geometry_msgs::msg::Twist command;
      command.linear.x = direction * 0.02;
      cmd_pub_->publish(command);
      rate.sleep();
    }
    stop();
  }

  void rotate_ccw_relative(double delta_yaw, const std::string & label)
  {
    const auto start_pose = lookup_robot_pose();
    if (!start_pose.has_value()) {
      return;
    }

    double last_yaw = start_pose->yaw;
    double rotated = 0.0;
    const double target_delta = std::abs(delta_yaw);
    constexpr double kYawDeltaDeadband = 0.002;
    RCLCPP_INFO(
      this->get_logger(),
      "Rotating CCW %.2f rad for %s",
      target_delta,
      label.c_str());

    rclcpp::Rate rate(20.0);
    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      if (charger_parking_interrupt_requested()) {
        stop_current_action_for_charger_interrupt();
        return;
      }
      const auto pose = lookup_robot_pose();
      if (!pose.has_value()) {
        rate.sleep();
        continue;
      }

      const double delta = normalize_angle(pose->yaw - last_yaw);
      last_yaw = pose->yaw;
      if (delta > kYawDeltaDeadband) {
        rotated += delta;
      }

      const double remaining = target_delta - rotated;
      if (remaining <= yaw_tolerance_) {
        stop();
        return;
      }

      geometry_msgs::msg::Twist command;
      command.angular.z = clamp(
        angular_gain_ * remaining,
        0.12,
        max_angular_speed_);
      cmd_pub_->publish(command);
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
      if (charger_parking_interrupt_requested()) {
        stop_current_action_for_charger_interrupt();
        return;
      }
      stop();
      rclcpp::sleep_for(100ms);
    }
  }

  void rotate_to_face(const std::string & target_node_name)
  {
    const auto & target = nodes_.at(target_node_name);
    RCLCPP_INFO(this->get_logger(), "Rotating in place toward %s", target_node_name.c_str());
    rotate_to_face(target, target_node_name);
  }

  void rotate_to_face(const MissionNode & target, const std::string & label)
  {
    RCLCPP_INFO(this->get_logger(), "Rotating in place toward %s", label.c_str());
    rclcpp::Rate rate(20.0);
    while (rclcpp::ok()) {
      rclcpp::spin_some(this->get_node_base_interface());
      if (charger_parking_interrupt_requested()) {
        stop_current_action_for_charger_interrupt();
        return;
      }
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
      if (charger_parking_interrupt_requested()) {
        stop_current_action_for_charger_interrupt();
        return;
      }
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

  void publish_rc_car_mode(const std::string & mode, bool force = false)
  {
    if (!force && mode == last_rc_car_mode_) {
      return;
    }

    if (
      mode == "slot_wait_a" ||
      mode == "turn_ccw_90" ||
      mode == "turn_cw_90" ||
      mode == "align_b_stop_yaw" ||
      mode == "drive_to_post_aruco_target")
    {
      last_rc_car_slot_status_.clear();
    }

    std_msgs::msg::String msg;
    msg.data = mode;
    rc_car_mode_pub_->publish(msg);
    last_rc_car_mode_ = mode;
  }

  void schedule_rc_car_mode(const std::string & mode, double delay_seconds)
  {
    if (delay_seconds <= 0.0) {
      publish_rc_car_mode(mode);
      return;
    }

    scheduled_rc_car_mode_ = mode;
    scheduled_rc_car_mode_at_ =
      this->now() + rclcpp::Duration::from_seconds(delay_seconds);
    RCLCPP_INFO(
      this->get_logger(),
      "Scheduled RC car mode %s after %.1f seconds",
      mode.c_str(),
      delay_seconds);
  }

  void publish_scheduled_rc_car_mode()
  {
    if (!scheduled_rc_car_mode_.has_value()) {
      return;
    }
    if (this->now() < scheduled_rc_car_mode_at_) {
      return;
    }

    const auto mode = scheduled_rc_car_mode_.value();
    scheduled_rc_car_mode_.reset();
    publish_rc_car_mode(mode);
    RCLCPP_INFO(this->get_logger(), "Published scheduled RC car mode %s", mode.c_str());
  }

  void clear_scheduled_rc_car_mode()
  {
    scheduled_rc_car_mode_.reset();
  }

  void republish_rc_car_mode()
  {
    if (charger_parking_active_) {
      std_msgs::msg::String msg;
      msg.data = "stop";
      rc_car_mode_pub_->publish(msg);
      last_rc_car_mode_ = "stop";
      return;
    }

    if (last_rc_car_mode_.empty()) {
      return;
    }
    if (last_rc_car_mode_ == "turn_ccw_90") {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = last_rc_car_mode_;
    rc_car_mode_pub_->publish(msg);
  }

  void release_rc_car_mode_control()
  {
    last_rc_car_mode_.clear();
  }

  std::string topology_file_;
  std::string start_mode_{"mission"};
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
  double rc_b_stop_x_{0.7269076704978943};
  double rc_b_stop_y_{-1.4004881381988525};
  double rc_b_stop_tolerance_{0.12};
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
  double charger_entry_accept_tolerance_{0.35};
  double charger_entry_handoff_tolerance_{0.85};
  double charger_entry_hold_handoff_tolerance_{1.20};
  double charger_entry_max_hold_seconds_{3.0};
  double charger_entry_hold_movement_epsilon_{0.03};
  int charger_corridor_escape_max_attempts_{3};
  int charger_corridor_escape_attempts_{0};
  double charger_corridor_escape_backup_distance_{0.08};
  double charger_corridor_escape_forward_distance_{0.45};
  double charger_corridor_escape_linear_speed_{0.05};
  double charger_corridor_escape_angular_speed_{0.24};
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
  bool enable_a_slot_aruco_follow_{true};
  bool enable_b_slot_aruco_follow_{true};
  double aruco_search_angular_speed_{0.18};
  double aruco_approach_linear_speed_{0.045};
  double aruco_approach_angular_gain_{0.40};
  double aruco_stop_distance_{0.28};
  double b_slot_aruco_stop_distance_{0.36};
  double b_slot_aruco_final_yaw_{M_PI_2};
  double aruco_detection_timeout_seconds_{0.6};
  bool enable_lidar_safety_{true};
  bool enable_mppi_rescue_{true};
  bool enable_corridor_pass_{true};
  bool mppi_rescue_active_{false};
  bool mppi_stop_and_plan_active_{false};
  bool mppi_side_escape_active_{false};
  bool charger_parking_requested_{false};
  bool charger_parking_active_{false};
  bool charger_nav2_inflation_override_active_{false};
  std::optional<double> charger_nav2_inflation_restore_value_;
  bool rc_b_stop_monitoring_{false};
  bool rc_b_stop_sent_{false};
  std::optional<std::string> scheduled_rc_car_mode_;
  rclcpp::Time scheduled_rc_car_mode_at_{0, 0, RCL_ROS_TIME};
  std::string last_rc_car_mode_;
  std::string last_rc_car_slot_status_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr leader_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rc_car_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr aruco_enable_pub_;
  rclcpp::TimerBase::SharedPtr rc_car_mode_timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rc_car_slot_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rc_car_odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr aruco_target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp_action::Client<FollowPath>::SharedPtr mppi_follow_path_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
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
  std::optional<geometry_msgs::msg::PointStamped> last_aruco_target_;
  rclcpp::Time last_aruco_target_time_{0, 0, RCL_ROS_TIME};
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
