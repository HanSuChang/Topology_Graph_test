#ifndef AMR_TOPOLOGY__LOCAL_ASTAR_PURE_PURSUIT_HPP_
#define AMR_TOPOLOGY__LOCAL_ASTAR_PURE_PURSUIT_HPP_

#include <optional>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace amr_topology
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Target2D
{
  double x{0.0};
  double y{0.0};
};

struct LocalPlannerOptions
{
  double emergency_stop_distance{0.22};
  double side_stop_distance{0.26};
  double rear_stop_distance{0.18};
  double obstacle_trigger_distance{0.42};
  double goal_block_distance{0.35};
  double stop_and_plan_seconds{2.0};
  double front_sector_width{0.70};
  double side_sector_min_angle{0.45};
  double side_sector_max_angle{1.20};
  double scan_timeout_seconds{0.7};
  double occupied_threshold{50.0};
  double robot_radius{0.18};
  double inflation_radius{0.28};
  double dynamic_obstacle_radius{0.22};
  double path_corridor_width{0.80};
  double max_plan_distance{3.5};
  double lookahead_distance{0.28};
  double goal_tolerance{0.12};
  double max_linear_speed{0.08};
  double min_linear_speed{0.025};
  double max_angular_speed{0.45};
  double side_escape_angular_speed{0.18};
  double angular_gain{1.2};
  double rotate_in_place_heading_threshold{0.35};
  double rotate_in_place_resume_threshold{0.18};
  double rotate_in_place_max_heading{1.05};
};

struct LocalPlannerDecision
{
  bool has_command{false};
  bool reached{false};
  bool blocked{false};
  geometry_msgs::msg::Twist command;
  std::string state{"normal"};
};

class LocalAStarPurePursuit
{
public:
  explicit LocalAStarPurePursuit(const LocalPlannerOptions & options = {});

  void update_scan(const sensor_msgs::msg::LaserScan & scan, const rclcpp::Time & stamp);
  void update_map(const nav_msgs::msg::OccupancyGrid & map);
  LocalPlannerDecision update(
    const Pose2D & pose,
    const Target2D & target,
    const rclcpp::Time & now);
  bool target_is_blocked(
    const Pose2D & pose,
    const Target2D & target,
    const rclcpp::Time & now) const;
  bool target_direction_is_clear(
    const Pose2D & pose,
    const Target2D & target,
    const rclcpp::Time & now) const;
  void reset();

private:
  enum class State
  {
    Normal,
    StopAndPlan,
    FollowDetour,
    Blocked
  };

  struct GridIndex
  {
    int x{0};
    int y{0};
  };

  struct ScanSummary
  {
    double front_min{std::numeric_limits<double>::infinity()};
    double left_min{std::numeric_limits<double>::infinity()};
    double right_min{std::numeric_limits<double>::infinity()};
    double rear_min{std::numeric_limits<double>::infinity()};
  };

  bool scan_is_fresh(const rclcpp::Time & now) const;
  bool map_is_ready() const;
  ScanSummary summarize_scan() const;
  double min_range_in_sector(double center, double width) const;
  bool world_to_grid(double wx, double wy, GridIndex & grid) const;
  Target2D grid_to_world(const GridIndex & grid) const;
  bool cell_is_occupied(const std::vector<uint8_t> & grid, int x, int y) const;
  void mark_circle(std::vector<uint8_t> & grid, double wx, double wy, double radius) const;
  std::vector<uint8_t> build_planning_grid(const Pose2D & pose) const;
  bool make_plan(const Pose2D & pose, const Target2D & target);
  std::optional<GridIndex> nearest_free_goal(
    const std::vector<uint8_t> & grid,
    const GridIndex & requested_goal) const;
  std::vector<Target2D> astar(
    const std::vector<uint8_t> & grid,
    const GridIndex & start,
    const GridIndex & goal) const;
  geometry_msgs::msg::Twist make_pure_pursuit_command(const Pose2D & pose);
  geometry_msgs::msg::Twist make_side_escape_command(const ScanSummary & scan_summary);
  geometry_msgs::msg::Twist stop_command() const;
  void enter_state(State state, const rclcpp::Time & now);
  std::string state_name() const;

  LocalPlannerOptions options_;
  std::optional<sensor_msgs::msg::LaserScan> scan_;
  std::optional<nav_msgs::msg::OccupancyGrid> map_;
  rclcpp::Time last_scan_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time state_started_at_{0, 0, RCL_ROS_TIME};
  State state_{State::Normal};
  std::vector<Target2D> path_;
  bool detour_alignment_done_{false};
  bool detour_alignment_active_{false};
  bool side_escape_active_{false};
  double side_escape_direction_{0.0};
};

}  // namespace amr_topology

#endif  // AMR_TOPOLOGY__LOCAL_ASTAR_PURE_PURSUIT_HPP_
