#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

struct TopologyNode
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct RobotPose
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

class TopologyFollower : public rclcpp::Node
{
public:
  TopologyFollower()
  : Node("topology_follower"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    const auto default_topology =
      ament_index_cpp::get_package_share_directory("amr_topology") + "/config/topology.yaml";

    this->declare_parameter<std::string>("topology_file", default_topology);
    this->declare_parameter<std::vector<std::string>>(
      "route", {"loading", "intersection_1", "a_entry", "a_leader_slot"});
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<double>("goal_tolerance", 0.08);
    this->declare_parameter<double>("waypoint_tolerance", 0.18);
    this->declare_parameter<double>("start_skip_tolerance", 0.35);
    this->declare_parameter<double>("yaw_tolerance", 0.12);
    this->declare_parameter<double>("max_linear_speed", 0.12);
    this->declare_parameter<double>("max_angular_speed", 0.45);
    this->declare_parameter<double>("min_turning_linear_speed", 0.03);
    this->declare_parameter<double>("drive_heading_limit", 1.57);
    this->declare_parameter<bool>("align_final_yaw", false);
    this->declare_parameter<double>("linear_gain", 0.45);
    this->declare_parameter<double>("angular_gain", 0.8);

    topology_file_ = this->get_parameter("topology_file").as_string();
    route_ = this->get_parameter("route").as_string_array();
    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    waypoint_tolerance_ = this->get_parameter("waypoint_tolerance").as_double();
    start_skip_tolerance_ = this->get_parameter("start_skip_tolerance").as_double();
    yaw_tolerance_ = this->get_parameter("yaw_tolerance").as_double();
    max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
    max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
    min_turning_linear_speed_ = this->get_parameter("min_turning_linear_speed").as_double();
    drive_heading_limit_ = this->get_parameter("drive_heading_limit").as_double();
    align_final_yaw_ = this->get_parameter("align_final_yaw").as_bool();
    linear_gain_ = this->get_parameter("linear_gain").as_double();
    angular_gain_ = this->get_parameter("angular_gain").as_double();

    load_topology();

    const auto cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
  }

  void run()
  {
    RCLCPP_INFO(this->get_logger(), "Following topology route");

    if (route_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Route is empty");
      return;
    }

    size_t target_index = 0;
    rclcpp::Rate rate(20.0);

    while (rclcpp::ok() && target_index < route_.size()) {
      rclcpp::spin_some(this->get_node_base_interface());

      const auto pose = lookup_robot_pose();
      if (!pose.has_value()) {
        rate.sleep();
        continue;
      }

      const bool is_first_node = target_index == 0;
      const bool is_final_node = target_index == route_.size() - 1;
      const auto & target_name = route_[target_index];
      const auto & target = nodes_.at(target_name);
      const double distance = distance_to_target(pose.value(), target);

      if (is_first_node && distance <= start_skip_tolerance_ && route_.size() > 1) {
        RCLCPP_INFO(
          this->get_logger(),
          "Skipping start node %s, distance %.2fm",
          target_name.c_str(),
          distance);
        ++target_index;
        continue;
      }

      if (!is_final_node && distance <= waypoint_tolerance_) {
        RCLCPP_INFO(
          this->get_logger(),
          "Passing waypoint %s, distance %.2fm",
          target_name.c_str(),
          distance);
        ++target_index;
        continue;
      }

      if (is_final_node && distance <= goal_tolerance_) {
        if (!align_final_yaw_ || final_yaw_aligned(pose.value(), target)) {
          stop();
          RCLCPP_INFO(this->get_logger(), "Reached final node: %s", target_name.c_str());
          break;
        }
      }

      cmd_pub_->publish(make_command(pose.value(), target));

      rate.sleep();
    }

    stop();
    RCLCPP_INFO(this->get_logger(), "Topology route completed");
  }

private:
  std::optional<RobotPose> lookup_robot_pose()
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      RobotPose pose;
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

  double distance_to_target(const RobotPose & pose, const TopologyNode & target) const
  {
    return std::hypot(target.x - pose.x, target.y - pose.y);
  }

  bool final_yaw_aligned(const RobotPose & pose, const TopologyNode & target) const
  {
    return std::abs(normalize_angle(target.yaw - pose.yaw)) <= yaw_tolerance_;
  }

  geometry_msgs::msg::Twist make_command(
    const RobotPose & pose,
    const TopologyNode & target)
  {
    geometry_msgs::msg::Twist command;
    const double dx = target.x - pose.x;
    const double dy = target.y - pose.y;
    const double distance = std::hypot(dx, dy);

    if (distance <= goal_tolerance_) {
      const double yaw_error = normalize_angle(target.yaw - pose.yaw);
      if (align_final_yaw_ && std::abs(yaw_error) > yaw_tolerance_) {
        command.angular.z = clamp(
          angular_gain_ * yaw_error,
          -max_angular_speed_,
          max_angular_speed_);
      }
      return command;
    }

    const double target_heading = std::atan2(dy, dx);
    const double heading_error = normalize_angle(target_heading - pose.yaw);
    command.angular.z = clamp(
      angular_gain_ * heading_error,
      -max_angular_speed_,
      max_angular_speed_);

    if (std::abs(heading_error) < drive_heading_limit_) {
      const double speed_scale = std::max(0.25, std::cos(heading_error));
      const double requested_speed = linear_gain_ * distance * speed_scale;
      command.linear.x = clamp(
        requested_speed,
        min_turning_linear_speed_,
        max_linear_speed_);
    }

    return command;
  }

  void load_topology()
  {
    const YAML::Node topology = YAML::LoadFile(topology_file_);
    frame_id_ = topology["frame_id"] ? topology["frame_id"].as<std::string>() : "map";

    const YAML::Node yaml_nodes = topology["nodes"];
    if (!yaml_nodes) {
      throw std::runtime_error("topology.yaml has no nodes section");
    }

    for (const auto & item : yaml_nodes) {
      const auto name = item.first.as<std::string>();
      const auto data = item.second;
      TopologyNode node;
      node.x = data["x"].as<double>();
      node.y = data["y"].as<double>();
      node.yaw = data["yaw"] ? data["yaw"].as<double>() : 0.0;
      nodes_[name] = node;
    }

    for (const auto & name : route_) {
      if (nodes_.find(name) == nodes_.end()) {
        throw std::runtime_error("route contains unknown node: " + name);
      }
    }

    if (frame_id_ != map_frame_) {
      RCLCPP_WARN(
        this->get_logger(),
        "topology frame_id=%s, but map_frame=%s",
        frame_id_.c_str(),
        map_frame_.c_str());
    }
  }

  void stop()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }

  std::string topology_file_;
  std::string frame_id_;
  std::string map_frame_;
  std::string base_frame_;
  std::vector<std::string> route_;
  std::unordered_map<std::string, TopologyNode> nodes_;

  double goal_tolerance_{0.08};
  double waypoint_tolerance_{0.18};
  double start_skip_tolerance_{0.35};
  double yaw_tolerance_{0.12};
  double max_linear_speed_{0.12};
  double max_angular_speed_{0.45};
  double min_turning_linear_speed_{0.03};
  double drive_heading_limit_{1.57};
  bool align_final_yaw_{false};
  double linear_gain_{0.45};
  double angular_gain_{0.8};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TopologyFollower>();
  node->run();

  rclcpp::shutdown();
  return 0;
}
