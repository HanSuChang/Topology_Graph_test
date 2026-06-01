#pragma once

#include <functional>
#include <optional>
#include <vector>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace amr_topology
{

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

struct ScanPoint2D
{
  double x{0.0};
  double y{0.0};
};

struct DwaPlannerConfig
{
  double min_linear_speed{0.04};
  double max_linear_speed{0.095};
  double max_angular_speed{0.55};
  int linear_samples{4};
  int angular_samples{11};
  double robot_radius{0.18};
  double safety_margin{0.25};
  double static_map_clearance{0.21};
  double obstacle_range{1.6};
  double goal_weight{0.9};
  double clearance_weight{2.0};
  double speed_weight{1.8};
  double sim_time{3.0};
  double sim_step{0.10};
  double front_clear_distance{0.90};
  double front_stop_distance{0.70};
  double stuck_turn_speed{0.38};
  double arc_commit_duration{1.4};
};

double normalize_angle(double angle);
bool angle_in_sector(double angle, double center, double width);
double clamp(double value, double lower, double upper);
double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q);

class DwaPlanner
{
public:
  using StaticCollisionFn = std::function<bool(double x, double y, double clearance)>;

  void set_config(const DwaPlannerConfig & config);

  int choose_turn_sign(double left_avg_range, double right_avg_range) const;

  std::optional<geometry_msgs::msg::Twist> plan_command(
    const RobotPose2D & pose,
    const MissionNode & target,
    int preferred_turn_sign,
    const std::vector<ScanPoint2D> & scan_points,
    double front_min_range,
    const StaticCollisionFn & static_collision) const;

  geometry_msgs::msg::Twist make_immediate_escape_command(
    int turn_sign,
    const RobotPose2D & pose,
    const MissionNode & target,
    double front_min_range,
    bool forward_collision_blocked,
    bool front_emergency_blocked) const;

private:
  DwaPlannerConfig config_;
};

}  // namespace amr_topology
