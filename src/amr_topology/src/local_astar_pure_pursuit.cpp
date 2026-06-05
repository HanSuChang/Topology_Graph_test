#include "amr_topology/local_astar_pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>

namespace amr_topology
{
namespace
{

constexpr int kFree = 0;
constexpr int kOccupied = 1;

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

bool valid_range(const sensor_msgs::msg::LaserScan & scan, double range)
{
  return std::isfinite(range) && range >= scan.range_min && range <= scan.range_max;
}

int key(int x, int y, int width)
{
  return y * width + x;
}

}  // namespace

LocalAStarPurePursuit::LocalAStarPurePursuit(const LocalPlannerOptions & options)
: options_(options)
{
}

void LocalAStarPurePursuit::update_scan(
  const sensor_msgs::msg::LaserScan & scan,
  const rclcpp::Time & stamp)
{
  scan_ = scan;
  last_scan_stamp_ = stamp;
}

void LocalAStarPurePursuit::update_map(const nav_msgs::msg::OccupancyGrid & map)
{
  map_ = map;
}

LocalPlannerDecision LocalAStarPurePursuit::update(
  const Pose2D & pose,
  const Target2D & target,
  const rclcpp::Time & now)
{
  LocalPlannerDecision decision;
  decision.state = state_name();

  if (!scan_is_fresh(now)) {
    reset();
    return decision;
  }

  const auto scan_summary = summarize_scan();
  const bool front_emergency =
    scan_summary.front_min <= options_.emergency_stop_distance;
  const bool side_emergency =
    scan_summary.left_min <= options_.side_stop_distance ||
    scan_summary.right_min <= options_.side_stop_distance;
  const bool obstacle_on_link =
    target_direction_is_clear(pose, target, now) == false;

  if (front_emergency) {
    enter_state(State::StopAndPlan, now);
    decision.has_command = true;
    decision.command = stop_command();
    decision.state = state_name();
    return decision;
  } else if (side_emergency && state_ != State::FollowDetour) {
    decision.has_command = true;
    decision.command = make_side_escape_command(scan_summary);
    decision.state = "side_escape";
    return decision;
  }

  if (state_ == State::Normal && obstacle_on_link) {
    enter_state(State::StopAndPlan, now);
  }

  const double state_elapsed = (now - state_started_at_).seconds();
  switch (state_) {
    case State::Normal:
      return decision;

    case State::StopAndPlan:
      decision.has_command = true;
      decision.command = stop_command();
      decision.state = state_name();
      if (state_elapsed >= options_.stop_and_plan_seconds) {
        if (make_plan(pose, target)) {
          enter_state(State::FollowDetour, now);
        } else {
          enter_state(State::Blocked, now);
        }
      }
      return decision;

    case State::FollowDetour: {
      decision.has_command = true;
      decision.state = state_name();
      if (std::hypot(target.x - pose.x, target.y - pose.y) <= options_.goal_tolerance) {
        reset();
        decision.has_command = false;
        decision.reached = true;
        decision.state = state_name();
        return decision;
      }
      if (front_emergency || path_.empty()) {
        enter_state(State::StopAndPlan, now);
        decision.command = stop_command();
        decision.state = state_name();
        return decision;
      }
      decision.command = make_pure_pursuit_command(pose);
      return decision;
    }

    case State::Blocked:
      decision.has_command = true;
      decision.blocked = true;
      decision.command = stop_command();
      decision.state = state_name();
      return decision;
  }

  return decision;
}

bool LocalAStarPurePursuit::target_is_blocked(
  const Pose2D & pose,
  const Target2D & target,
  const rclcpp::Time & now) const
{
  if (!scan_is_fresh(now)) {
    return false;
  }

  const double distance = std::hypot(target.x - pose.x, target.y - pose.y);
  if (distance > options_.goal_block_distance) {
    return false;
  }

  if (map_is_ready()) {
    const auto grid = build_planning_grid(pose);
    GridIndex target_grid;
    if (world_to_grid(target.x, target.y, target_grid) &&
      cell_is_occupied(grid, target_grid.x, target_grid.y))
    {
      return true;
    }
  }

  return !target_direction_is_clear(pose, target, now);
}

bool LocalAStarPurePursuit::target_direction_is_clear(
  const Pose2D & pose,
  const Target2D & target,
  const rclcpp::Time & now) const
{
  if (!scan_is_fresh(now) || !scan_.has_value()) {
    return true;
  }

  const double target_heading = std::atan2(target.y - pose.y, target.x - pose.x);
  const double heading_error = normalize_angle(target_heading - pose.yaw);
  const double distance = std::hypot(target.x - pose.x, target.y - pose.y);
  const double obstacle_distance = min_range_in_sector(heading_error, options_.front_sector_width);
  const double trigger = std::min(options_.obstacle_trigger_distance, std::max(0.15, distance));
  return obstacle_distance > trigger;
}

void LocalAStarPurePursuit::reset()
{
  state_ = State::Normal;
  path_.clear();
  detour_alignment_done_ = false;
  detour_alignment_active_ = false;
}

bool LocalAStarPurePursuit::scan_is_fresh(const rclcpp::Time & now) const
{
  return scan_.has_value() && (now - last_scan_stamp_).seconds() <= options_.scan_timeout_seconds;
}

bool LocalAStarPurePursuit::map_is_ready() const
{
  return map_.has_value() && map_->info.width > 0 && map_->info.height > 0 &&
    map_->info.resolution > 0.0f && !map_->data.empty();
}

LocalAStarPurePursuit::ScanSummary LocalAStarPurePursuit::summarize_scan() const
{
  ScanSummary summary;
  summary.front_min = min_range_in_sector(0.0, options_.front_sector_width);
  const double side_center =
    (options_.side_sector_min_angle + options_.side_sector_max_angle) * 0.5;
  const double side_width =
    options_.side_sector_max_angle - options_.side_sector_min_angle;
  summary.left_min = min_range_in_sector(side_center, side_width);
  summary.right_min = min_range_in_sector(-side_center, side_width);
  summary.rear_min = min_range_in_sector(M_PI, 0.70);
  return summary;
}

double LocalAStarPurePursuit::min_range_in_sector(double center, double width) const
{
  if (!scan_.has_value() || scan_->angle_increment == 0.0f) {
    return std::numeric_limits<double>::infinity();
  }

  const auto & scan = scan_.value();
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double angle =
      normalize_angle(scan.angle_min + static_cast<double>(i) * scan.angle_increment);
    if (std::abs(normalize_angle(angle - center)) > width * 0.5) {
      continue;
    }
    const double range = scan.ranges[i];
    if (valid_range(scan, range)) {
      best = std::min(best, range);
    }
  }
  return best;
}

bool LocalAStarPurePursuit::world_to_grid(double wx, double wy, GridIndex & grid) const
{
  if (!map_is_ready()) {
    return false;
  }

  const auto & info = map_->info;
  const double origin_x = info.origin.position.x;
  const double origin_y = info.origin.position.y;
  grid.x = static_cast<int>(std::floor((wx - origin_x) / info.resolution));
  grid.y = static_cast<int>(std::floor((wy - origin_y) / info.resolution));
  return grid.x >= 0 && grid.y >= 0 &&
    grid.x < static_cast<int>(info.width) &&
    grid.y < static_cast<int>(info.height);
}

Target2D LocalAStarPurePursuit::grid_to_world(const GridIndex & grid) const
{
  const auto & info = map_->info;
  Target2D point;
  point.x = info.origin.position.x + (static_cast<double>(grid.x) + 0.5) * info.resolution;
  point.y = info.origin.position.y + (static_cast<double>(grid.y) + 0.5) * info.resolution;
  return point;
}

bool LocalAStarPurePursuit::cell_is_occupied(
  const std::vector<uint8_t> & grid,
  int x,
  int y) const
{
  const auto & info = map_->info;
  if (x < 0 || y < 0 || x >= static_cast<int>(info.width) || y >= static_cast<int>(info.height)) {
    return true;
  }
  return grid[static_cast<size_t>(key(x, y, info.width))] == kOccupied;
}

void LocalAStarPurePursuit::mark_circle(
  std::vector<uint8_t> & grid,
  double wx,
  double wy,
  double radius) const
{
  GridIndex center;
  if (!world_to_grid(wx, wy, center)) {
    return;
  }

  const auto & info = map_->info;
  const int cells = static_cast<int>(std::ceil(radius / info.resolution));
  for (int dy = -cells; dy <= cells; ++dy) {
    for (int dx = -cells; dx <= cells; ++dx) {
      const int x = center.x + dx;
      const int y = center.y + dy;
      if (x < 0 || y < 0 || x >= static_cast<int>(info.width) ||
        y >= static_cast<int>(info.height))
      {
        continue;
      }
      if (std::hypot(dx * info.resolution, dy * info.resolution) <= radius) {
        grid[static_cast<size_t>(key(x, y, info.width))] = kOccupied;
      }
    }
  }
}

std::vector<uint8_t> LocalAStarPurePursuit::build_planning_grid(const Pose2D & pose) const
{
  const auto & info = map_->info;
  std::vector<uint8_t> grid(static_cast<size_t>(info.width) * info.height, kFree);

  for (uint32_t y = 0; y < info.height; ++y) {
    for (uint32_t x = 0; x < info.width; ++x) {
      const auto value = map_->data[static_cast<size_t>(key(x, y, info.width))];
      if (value < 0 || value >= options_.occupied_threshold) {
        const auto center = grid_to_world(GridIndex{static_cast<int>(x), static_cast<int>(y)});
        mark_circle(grid, center.x, center.y, options_.inflation_radius);
      }
    }
  }

  if (scan_.has_value()) {
    const auto & scan = scan_.value();
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const double range = scan.ranges[i];
      if (!valid_range(scan, range)) {
        continue;
      }
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double wx = pose.x + range * std::cos(pose.yaw + angle);
      const double wy = pose.y + range * std::sin(pose.yaw + angle);
      mark_circle(grid, wx, wy, options_.dynamic_obstacle_radius);
    }
  }

  return grid;
}

bool LocalAStarPurePursuit::make_plan(const Pose2D & pose, const Target2D & target)
{
  if (!map_is_ready() || std::hypot(target.x - pose.x, target.y - pose.y) > options_.max_plan_distance) {
    path_.clear();
    return false;
  }

  const auto grid = build_planning_grid(pose);
  GridIndex start;
  GridIndex goal;
  if (!world_to_grid(pose.x, pose.y, start) || !world_to_grid(target.x, target.y, goal)) {
    path_.clear();
    return false;
  }

  const auto free_goal = nearest_free_goal(grid, goal);
  if (!free_goal.has_value() || cell_is_occupied(grid, start.x, start.y)) {
    path_.clear();
    return false;
  }

  path_ = astar(grid, start, free_goal.value());
  return path_.size() >= 2;
}

std::optional<LocalAStarPurePursuit::GridIndex> LocalAStarPurePursuit::nearest_free_goal(
  const std::vector<uint8_t> & grid,
  const GridIndex & requested_goal) const
{
  if (!cell_is_occupied(grid, requested_goal.x, requested_goal.y)) {
    return requested_goal;
  }

  const int max_radius = static_cast<int>(std::ceil(options_.goal_block_distance / map_->info.resolution));
  for (int radius = 1; radius <= max_radius; ++radius) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != radius) {
          continue;
        }
        const GridIndex candidate{requested_goal.x + dx, requested_goal.y + dy};
        if (!cell_is_occupied(grid, candidate.x, candidate.y)) {
          return candidate;
        }
      }
    }
  }
  return std::nullopt;
}

std::vector<Target2D> LocalAStarPurePursuit::astar(
  const std::vector<uint8_t> & grid,
  const GridIndex & start,
  const GridIndex & goal) const
{
  const auto & info = map_->info;
  struct QueueNode
  {
    int x;
    int y;
    double f;
    bool operator>(const QueueNode & other) const {return f > other.f;}
  };

  const auto heuristic = [&goal](int x, int y) {
      return std::hypot(goal.x - x, goal.y - y);
    };

  std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> open;
  std::unordered_map<int, double> cost;
  std::unordered_map<int, int> parent;
  const int start_key = key(start.x, start.y, info.width);
  const int goal_key = key(goal.x, goal.y, info.width);
  open.push(QueueNode{start.x, start.y, heuristic(start.x, start.y)});
  cost[start_key] = 0.0;
  parent[start_key] = start_key;

  const int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  while (!open.empty()) {
    const auto current = open.top();
    open.pop();
    const int current_key = key(current.x, current.y, info.width);
    if (current_key == goal_key) {
      break;
    }

    for (int i = 0; i < 8; ++i) {
      const int nx = current.x + dxs[i];
      const int ny = current.y + dys[i];
      if (cell_is_occupied(grid, nx, ny)) {
        continue;
      }
      const double step = i < 4 ? 1.0 : 1.41421356237;
      const int next_key = key(nx, ny, info.width);
      const double new_cost = cost[current_key] + step;
      if (!cost.count(next_key) || new_cost < cost[next_key]) {
        cost[next_key] = new_cost;
        parent[next_key] = current_key;
        open.push(QueueNode{nx, ny, new_cost + heuristic(nx, ny)});
      }
    }
  }

  if (!parent.count(goal_key)) {
    return {};
  }

  std::vector<GridIndex> grid_path;
  int current = goal_key;
  while (current != start_key) {
    grid_path.push_back(GridIndex{
        current % static_cast<int>(info.width),
        current / static_cast<int>(info.width)});
    current = parent[current];
  }
  grid_path.push_back(start);
  std::reverse(grid_path.begin(), grid_path.end());

  std::vector<Target2D> path;
  path.reserve(grid_path.size());
  for (const auto & point : grid_path) {
    path.push_back(grid_to_world(point));
  }
  return path;
}

geometry_msgs::msg::Twist LocalAStarPurePursuit::make_pure_pursuit_command(
  const Pose2D & pose)
{
  geometry_msgs::msg::Twist command;
  if (path_.empty()) {
    return command;
  }

  Target2D lookahead = path_.back();
  size_t nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path_.size(); ++i) {
    const double distance = std::hypot(path_[i].x - pose.x, path_[i].y - pose.y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = i;
    }
  }

  double accumulated_distance = 0.0;
  for (size_t i = nearest_index + 1; i < path_.size(); ++i) {
    accumulated_distance += std::hypot(
      path_[i].x - path_[i - 1].x,
      path_[i].y - path_[i - 1].y);
    if (accumulated_distance >= options_.lookahead_distance) {
      lookahead = path_[i];
      break;
    }
  }

  const double heading = std::atan2(lookahead.y - pose.y, lookahead.x - pose.x);
  const double heading_error = normalize_angle(heading - pose.yaw);
  command.angular.z = clamp(
    options_.angular_gain * heading_error,
    -options_.max_angular_speed,
    options_.max_angular_speed);

  if (!detour_alignment_done_) {
    const double abs_heading_error = std::abs(heading_error);
    if (!detour_alignment_active_ &&
      abs_heading_error > options_.rotate_in_place_heading_threshold &&
      abs_heading_error <= options_.rotate_in_place_max_heading)
    {
      detour_alignment_active_ = true;
    }
    if (detour_alignment_active_ &&
      abs_heading_error > options_.rotate_in_place_resume_threshold)
    {
      command.linear.x = 0.0;
      return command;
    }
    detour_alignment_active_ = false;
    detour_alignment_done_ = true;
  }

  const double speed_scale = std::max(0.25, std::cos(heading_error));
  command.linear.x = clamp(
    options_.max_linear_speed * speed_scale,
    options_.min_linear_speed,
    options_.max_linear_speed);
  return command;
}

geometry_msgs::msg::Twist LocalAStarPurePursuit::make_side_escape_command(
  const ScanSummary & scan_summary) const
{
  geometry_msgs::msg::Twist command;
  if (scan_summary.right_min <= scan_summary.left_min) {
    command.angular.z = options_.side_escape_angular_speed;
  } else {
    command.angular.z = -options_.side_escape_angular_speed;
  }
  return command;
}

geometry_msgs::msg::Twist LocalAStarPurePursuit::stop_command() const
{
  return geometry_msgs::msg::Twist{};
}

void LocalAStarPurePursuit::enter_state(State state, const rclcpp::Time & now)
{
  if (state_ != state) {
    state_ = state;
    state_started_at_ = now;
    if (state_ == State::FollowDetour) {
      detour_alignment_done_ = false;
      detour_alignment_active_ = false;
    }
  }
}

std::string LocalAStarPurePursuit::state_name() const
{
  switch (state_) {
    case State::Normal:
      return "normal";
    case State::StopAndPlan:
      return "stop_and_plan";
    case State::FollowDetour:
      return "follow_detour";
    case State::Blocked:
      return "blocked";
  }
  return "unknown";
}

}  // namespace amr_topology
