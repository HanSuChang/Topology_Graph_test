#!/usr/bin/env bash
# GUI에서 2D Pose Estimate를 쓰기 위한 로컬 ROS2 localization/Nav2 stack.
# 터틀봇 bringup과 RC카 접속은 사용자가 먼저 수동으로 처리하고, 이 스크립트는
# GCS/로컬 PC에서 필요한 map server, MPPI, charger Nav2, AMCL만 올린다.
set -eo pipefail

TOPOLOGY_WS="${TOPOLOGY_WS:-$HOME/Downloads/Topology_Graph_test-main}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-27}"
TURTLEBOT_SSH="${TURTLEBOT_SSH:-han2@han2.local}"
TURTLEBOT_TOPOLOGY_WS="${TURTLEBOT_TOPOLOGY_WS:-\$HOME/Downloads/Topology_Graph_test-main}"

source /opt/ros/humble/setup.bash
if [ -f "$TOPOLOGY_WS/install/setup.bash" ]; then
  source "$TOPOLOGY_WS/install/setup.bash"
fi
set -u
export ROS_DOMAIN_ID

pids=()
cleaned_up=0
cleanup() {
  if [ "$cleaned_up" -eq 1 ]; then
    return
  fi
  cleaned_up=1

  echo "[localization] stopping local launch processes"
  for pid in "${pids[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done

  echo "[localization] stopping turtlebot launch processes"
  ssh -o ConnectTimeout=2 "$TURTLEBOT_SSH" "
    pkill -TERM -f 'ros2 launch amr_topology picam_rotate_compressed.launch.py' || true
    pkill -TERM -f 'image_rotate_compress_node' || true
    pkill -TERM -f 'ros2 launch amr_topology esp32_serial_bridge.launch.py' || true
    pkill -TERM -f 'esp32_servo_bridge_node.py' || true
  " 2>/dev/null || true

  wait 2>/dev/null || true
}
on_signal() {
  cleanup
  exit 130
}
trap on_signal INT TERM
trap cleanup EXIT

start() {
  local name="$1"
  shift
  echo "[localization] starting $name: $*"
  "$@" &
  pids+=("$!")
  sleep 1
}

start "map_server" ros2 launch amr_topology map_server.launch.py
start "mppi_controller" ros2 launch amr_topology mppi_controller.launch.py
start "charger_navigation" ros2 launch amr_topology charger_navigation.launch.py
start "picam_rotate_compressed@turtlebot" ssh "$TURTLEBOT_SSH"   "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID; source /opt/ros/humble/setup.bash; if [ -f $TURTLEBOT_TOPOLOGY_WS/install/setup.bash ]; then source $TURTLEBOT_TOPOLOGY_WS/install/setup.bash; fi; ros2 launch amr_topology picam_rotate_compressed.launch.py jpeg_quality:=70"
start "esp32_serial_bridge@turtlebot" ssh "$TURTLEBOT_SSH"   "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID; source /opt/ros/humble/setup.bash; if [ -f $TURTLEBOT_TOPOLOGY_WS/install/setup.bash ]; then source $TURTLEBOT_TOPOLOGY_WS/install/setup.bash; fi; ros2 launch amr_topology esp32_serial_bridge.launch.py"
start "arm_vision_pick" ros2 launch amr_topology arm_vision_pick.launch.py target_color:=blue auto_start_enabled:=true
start "amcl" ros2 run nav2_amcl amcl --ros-args \
  -p use_sim_time:=false \
  -p base_frame_id:=base_footprint \
  -p odom_frame_id:=odom \
  -p global_frame_id:=map \
  -p scan_topic:=/scan

echo "[localization] waiting for /amcl lifecycle service"
for _ in $(seq 1 30); do
  if ros2 lifecycle get /amcl >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

ros2 lifecycle set /amcl configure || true
ros2 lifecycle set /amcl activate || true

echo "[localization] stack running. Press Ctrl+C to stop."
wait
