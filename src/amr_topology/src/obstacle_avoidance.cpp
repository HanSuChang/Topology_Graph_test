#include "amr_topology/obstacle_avoidance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace amr_topology
{

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

bool angle_in_sector(double angle, double center, double width)
{
  return std::abs(normalize_angle(angle - center)) <= width * 0.5;
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

void DwaPlanner::set_config(const DwaPlannerConfig & config)
{
  config_ = config;
}

int DwaPlanner::choose_turn_sign(double left_avg_range, double right_avg_range) const
{
  if (std::isfinite(left_avg_range) && std::isfinite(right_avg_range)) {
    return left_avg_range >= right_avg_range ? 1 : -1;
  }
  if (std::isfinite(left_avg_range)) {
    return 1;
  }
  if (std::isfinite(right_avg_range)) {
    return -1;
  }
  return 1;
}

std::optional<geometry_msgs::msg::Twist> DwaPlanner::plan_command(
  const RobotPose2D & pose,
  const MissionNode & target,
  int preferred_turn_sign,
  const std::vector<ScanPoint2D> & scan_points,
  double front_min_range,
  const StaticCollisionFn & static_collision) const
{
  const int linear_samples = std::max(1, config_.linear_samples);
  const int angular_samples = std::max(1, config_.angular_samples);
  const double dt = std::max(0.02, config_.sim_step);
  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);
  const bool needs_escape_turn =
    std::isfinite(front_min_range) && front_min_range < config_.front_clear_distance;
  const double arc_linear = std::min(
    config_.max_linear_speed,
    std::max(config_.min_linear_speed, 0.075));
  const double tight_linear = std::min(config_.max_linear_speed, 0.055);
  const double turn_linear = std::min(config_.max_linear_speed, 0.04);

  struct ArcCandidate
  {
    double linear;
    double angular;
  };

  std::vector<ArcCandidate> candidates;
  const int first_sign = preferred_turn_sign == 0 ? 1 : preferred_turn_sign;
  const int second_sign = -first_sign;
  for (const int sign : {first_sign, second_sign}) {
    candidates.push_back({arc_linear, static_cast<double>(sign) * 0.32});
    candidates.push_back({tight_linear, static_cast<double>(sign) * 0.45});
    candidates.push_back({turn_linear, static_cast<double>(sign) * 0.55});
  }

  std::optional<geometry_msgs::msg::Twist> best_command;
  double best_score = -std::numeric_limits<double>::infinity();

  (void)linear_samples;
  (void)angular_samples;

  for (const auto & candidate : candidates) {
      const double linear = candidate.linear;
      const double angular = candidate.angular;

      double sim_x = 0.0;
      double sim_y = 0.0;
      double sim_yaw = 0.0;
      double min_clearance = std::numeric_limits<double>::infinity();
      bool collision = false;

      for (double time = 0.0; time <= config_.sim_time; time += dt) {
        sim_x += linear * std::cos(sim_yaw) * dt;
        sim_y += linear * std::sin(sim_yaw) * dt;
        sim_yaw = normalize_angle(sim_yaw + angular * dt);

        const double sim_map_x = pose.x + cos_yaw * sim_x - sin_yaw * sim_y;
        const double sim_map_y = pose.y + sin_yaw * sim_x + cos_yaw * sim_y;
        if (
          static_collision &&
          static_collision(sim_map_x, sim_map_y, config_.static_map_clearance))
        {
          collision = true;
          break;
        }

        for (const auto & point : scan_points) {
          const double clearance = std::hypot(point.x - sim_x, point.y - sim_y) -
            config_.robot_radius;
          min_clearance = std::min(min_clearance, clearance);
          if (clearance <= config_.safety_margin) {
            collision = true;
            break;
          }
        }

        if (collision) {
          break;
        }
      }

      if (collision) {
        continue;
      }

      const double end_x = pose.x + cos_yaw * sim_x - sin_yaw * sim_y;
      const double end_y = pose.y + sin_yaw * sim_x + cos_yaw * sim_y;
      const double end_yaw = normalize_angle(pose.yaw + sim_yaw);
      const double goal_distance = std::hypot(target.x - end_x, target.y - end_y);
      const double goal_heading = std::atan2(target.y - end_y, target.x - end_x);
      const double heading_error = std::abs(normalize_angle(goal_heading - end_yaw));
      const double clearance_score = std::min(min_clearance, config_.obstacle_range);
      const double escape_turn_score = needs_escape_turn && config_.max_angular_speed > 0.0 ?
        std::min(1.0, std::abs(angular) / config_.max_angular_speed) : 0.0;
      const double preferred_turn_bonus =
        preferred_turn_sign != 0 && angular * preferred_turn_sign > 0.0 ? 0.35 : 0.0;
      const double score =
        -config_.goal_weight * goal_distance -
        0.35 * heading_error +
        config_.clearance_weight * clearance_score +
        config_.speed_weight * linear +
        0.75 * escape_turn_score +
        preferred_turn_bonus;

      if (score > best_score) {
        geometry_msgs::msg::Twist command;
        command.linear.x = linear;
        command.angular.z = angular;
        best_command = command;
        best_score = score;
      }
  }

  return best_command;
}

geometry_msgs::msg::Twist DwaPlanner::make_immediate_escape_command(
  int turn_sign,
  const RobotPose2D & pose,
  const MissionNode & target,
  double front_min_range,
  bool forward_collision_blocked,
  bool front_emergency_blocked) const
{
  geometry_msgs::msg::Twist command;
  const double target_heading = std::atan2(target.y - pose.y, target.x - pose.x);
  const double heading_error = normalize_angle(target_heading - pose.yaw);
  const double target_turn = clamp(
    0.45 * heading_error,
    -config_.stuck_turn_speed,
    config_.stuck_turn_speed);
  command.angular.z = target_turn;

  if (turn_sign != 0 && command.angular.z * turn_sign < 0.0) {
    command.angular.z = static_cast<double>(turn_sign) * 0.5 * config_.stuck_turn_speed;
  }
  if (std::abs(command.angular.z) < 0.08) {
    command.angular.z = static_cast<double>(turn_sign == 0 ? 1 : turn_sign) * 0.08;
  }

  const bool has_room_for_arc =
    std::isfinite(front_min_range) &&
    front_min_range > config_.front_stop_distance + 0.08 &&
    !forward_collision_blocked &&
    !front_emergency_blocked;

  if (has_room_for_arc) {
    command.linear.x = std::min(0.06, config_.max_linear_speed);
  }
  return command;
}

}  // namespace amr_topology
