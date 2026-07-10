#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace
{

struct MarkerPoint
{
  std::string name;
  std::string label;
  double x{0.0};
  double y{0.0};
  double z{0.03};
  double r{0.2};
  double g{0.6};
  double b{1.0};
  double radius{0.08};
};

std::array<double, 3> color_for_name(const std::string & name)
{
  if (name == "intersection_1" || name.rfind("a_", 0) == 0) {
    return {1.0, 0.1, 0.1};
  }
  if (name.rfind("b_", 0) == 0 || name.rfind("rc_b_", 0) == 0) {
    return {0.1, 0.35, 1.0};
  }
  if (name == "loading") {
    return {0.85, 0.85, 0.85};
  }
  return {0.55, 0.55, 0.55};
}

}  // namespace

class TopologyMarkerNode : public rclcpp::Node
{
public:
  TopologyMarkerNode()
  : Node("topology_marker_node")
  {
    const auto default_topology =
      ament_index_cpp::get_package_share_directory("amr_topology") + "/config/topology.yaml";

    this->declare_parameter<std::string>("topology_file", default_topology);
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<double>("marker_radius", 0.08);
    this->declare_parameter<double>("marker_alpha", 0.85);
    this->declare_parameter<bool>("show_labels", true);
    this->declare_parameter<bool>("show_rc_a_stop", true);
    this->declare_parameter<double>("rc_a_stop_x", 0.7064711451530457);
    this->declare_parameter<double>("rc_a_stop_y", 0.29016929864883423);
    this->declare_parameter<bool>("show_rc_b_stop", true);
    this->declare_parameter<double>("rc_b_stop_x", 0.7269076704978943);
    this->declare_parameter<double>("rc_b_stop_y", -1.4004881381988525);

    frame_id_ = this->get_parameter("frame_id").as_string();
    marker_radius_ = this->get_parameter("marker_radius").as_double();
    marker_alpha_ = std::clamp(this->get_parameter("marker_alpha").as_double(), 0.0, 1.0);
    show_labels_ = this->get_parameter("show_labels").as_bool();

    load_markers();

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/topology_markers", rclcpp::QoS(1).transient_local().reliable());
    timer_ = this->create_wall_timer(1s, std::bind(&TopologyMarkerNode::publish_markers, this));
  }

private:
  void load_markers()
  {
    markers_.clear();
    const auto topology_file = this->get_parameter("topology_file").as_string();
    const YAML::Node root = YAML::LoadFile(topology_file);
    const YAML::Node nodes = root["nodes"];

    for (const auto & entry : nodes) {
      const std::string name = entry.first.as<std::string>();
      if (name == "standby_1") {
        continue;
      }
      const YAML::Node node = entry.second;
      const auto color = color_for_name(name);

      MarkerPoint point;
      point.name = name;
      point.label = name;
      point.x = node["x"].as<double>();
      point.y = node["y"].as<double>();
      point.r = color[0];
      point.g = color[1];
      point.b = color[2];
      point.radius = marker_radius_;
      markers_.push_back(point);
    }

    if (this->get_parameter("show_rc_a_stop").as_bool()) {
      MarkerPoint point;
      point.name = "rc_a_stop";
      point.label = "rc_car_a_stop";
      point.x = this->get_parameter("rc_a_stop_x").as_double();
      point.y = this->get_parameter("rc_a_stop_y").as_double();
      point.r = 1.0;
      point.g = 0.1;
      point.b = 0.1;
      point.radius = marker_radius_ * 1.15;
      markers_.push_back(point);
    }

    if (this->get_parameter("show_rc_b_stop").as_bool()) {
      MarkerPoint point;
      point.name = "rc_b_stop";
      point.label = "rc_car_b_stop";
      point.x = this->get_parameter("rc_b_stop_x").as_double();
      point.y = this->get_parameter("rc_b_stop_y").as_double();
      point.r = 1.0;
      point.g = 0.1;
      point.b = 0.1;
      point.radius = marker_radius_ * 1.15;
      markers_.push_back(point);
    }

  }

  visualization_msgs::msg::Marker make_circle_marker(const MarkerPoint & point, int id) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = this->now();
    marker.ns = "topology_points";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = point.radius;
    marker.scale.y = point.radius;
    marker.scale.z = 0.035;
    marker.color.r = point.r;
    marker.color.g = point.g;
    marker.color.b = point.b;
    marker.color.a = marker_alpha_;
    return marker;
  }

  visualization_msgs::msg::Marker make_text_marker(const MarkerPoint & point, int id) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = this->now();
    marker.ns = "topology_labels";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = 0.16;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.08;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.text = point.label.empty() ? point.name : point.label;
    return marker;
  }

  void publish_markers()
  {
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id_;
    clear_marker.header.stamp = this->now();
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    int id = 0;
    for (const auto & point : markers_) {
      marker_array.markers.push_back(make_circle_marker(point, id++));
    }
    if (show_labels_) {
      for (const auto & point : markers_) {
        marker_array.markers.push_back(make_text_marker(point, id++));
      }
    }
    marker_pub_->publish(marker_array);
  }

  std::string frame_id_{"map"};
  double marker_radius_{0.08};
  double marker_alpha_{0.85};
  bool show_labels_{true};
  std::vector<MarkerPoint> markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopologyMarkerNode>());
  rclcpp::shutdown();
  return 0;
}
