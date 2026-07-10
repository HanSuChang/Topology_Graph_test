#!/usr/bin/env bash
# Run the Python ROS2 Bridge. When ROS2 (rclpy) is sourced the bridge
# subscribes to the TurtleBot3 fleet and forwards messages to the Go
# backend over WebSocket. Without rclpy it falls back to a synth feed.
#
# Defaults:
#   ROS_DOMAIN_ID=27  (override by exporting before invoking this script)
set -euo pipefail
cd "$(dirname "$0")/.."

# Project-default ROS_DOMAIN_ID. Allow operator override.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-27}"

# Source ROS2 if it hasn't been sourced yet so the bridge can import
# rclpy/geometry_msgs/etc. Adjust DISTRO if you're on Iron/Jazzy.
ROS_DISTRO="${ROS_DISTRO:-humble}"
if [ -z "${AMENT_PREFIX_PATH:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

cd bridge
exec python3 -m ros2_bridge --config ../backend/configs/config.yaml
