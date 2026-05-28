# Topology Graph Test

ROS 2 Humble 기반 TurtleBot3 `waffle_pi`용 topology graph 주행 테스트 패키지입니다.

이 패키지는 Nav2의 경로계획/제어 스택을 사용하지 않고, `topology.yaml`에 정의된 노드 좌표를 따라 자체 주행 노드가 `/cmd_vel`을 직접 발행합니다.

## Package

- `amr_topology`
  - map server launch
  - topology graph YAML
  - waypoint follower
  - A/B 반복 미션 루프
  - 충전 위치 후진 도킹 프로토타입

## Workspace

```bash
cd /home/lee/Downloads/Topology_Graph_test-main
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
```

처음 빌드하거나 코드를 수정한 뒤에는:

```bash
colcon build --packages-select amr_topology
source install/setup.bash
```

## 실행 순서

### 1. TurtleBot3 Bringup

새 터미널에서:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

ros2 launch turtlebot3_bringup robot.launch.py
```

### 2. Map Server

새 터미널에서:

```bash
cd /home/lee/Downloads/Topology_Graph_test-main
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

ros2 launch amr_topology map_server.launch.py
```

맵 경로를 직접 지정하려면:

```bash
ros2 launch amr_topology map_server.launch.py \
  map:=/home/lee/Downloads/Topology_Graph_test-main/src/amr_topology/maps/map.yaml
```

### 3. AMCL

이 주행 노드는 현재 위치를 TF에서 읽습니다. 따라서 `map -> base_footprint` TF가 필요하고, 이를 위해 AMCL을 실행합니다.

새 터미널에서:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

ros2 run nav2_amcl amcl --ros-args \
  -p use_sim_time:=false \
  -p base_frame_id:=base_footprint \
  -p odom_frame_id:=odom \
  -p global_frame_id:=map \
  -p scan_topic:=/scan
```

AMCL lifecycle 활성화:

```bash
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate
```

### 4. RViz2

새 터미널에서:

```bash
source /opt/ros/humble/setup.bash
rviz2
```

RViz2 설정:

- `Global Options > Fixed Frame`: `map`
- `Add > Map`
  - Topic: `/map`
- `Add > LaserScan`
  - Topic: `/scan`
- `Add > TF`
- `Add > RobotModel`
- `Add > PoseWithCovarianceStamped`
  - Topic: `/amcl_pose`

RViz2 상단의 `2D Pose Estimate`로 실제 로봇 위치와 방향을 맵 위에 지정합니다.

TF 확인:

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

숫자가 계속 출력되면 localization이 정상입니다.

## 주행 실행

### 전체 미션 루프

A 구역과 B 구역을 반복 주행한 뒤 충전 위치로 이동하고 후진 도킹을 수행합니다.

```bash
cd /home/lee/Downloads/Topology_Graph_test-main
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

ros2 run amr_topology mission_loop --ros-args \
  -p base_frame:=base_footprint \
  -p enable_lidar_safety:=true \
  -p scan_topic:=/scan \
  -p rear_stop_distance:=0.18
```

반복 횟수 변경:

```bash
ros2 run amr_topology mission_loop --ros-args \
  -p base_frame:=base_footprint \
  -p repeat_count:=1 \
  -p enable_lidar_safety:=true \
  -p scan_topic:=/scan
```

### 특정 경로만 주행

상차 위치에서 A 구역:

```bash
ros2 run amr_topology topology_follower --ros-args \
  -p base_frame:=base_footprint \
  -p route:="[loading, intersection_1, a_entry, a_leader_slot]" \
  -p enable_lidar_safety:=true
```

A 구역에서 B 구역:

```bash
ros2 run amr_topology topology_follower --ros-args \
  -p base_frame:=base_footprint \
  -p route:="[a_leader_slot, a_entry, b_entry, b_leader_slot]" \
  -p enable_lidar_safety:=true
```

B 구역에서 충전 진입점:

```bash
ros2 run amr_topology topology_follower --ros-args \
  -p base_frame:=base_footprint \
  -p route:="[b_leader_slot, charger_entry]" \
  -p enable_lidar_safety:=true
```

## 수동 주행 테스트

키보드 조작:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

정지 명령:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

저속 전진:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

## 확인 명령

토픽 확인:

```bash
ros2 topic list
```

맵 확인:

```bash
ros2 topic echo /map
```

라이다 확인:

```bash
ros2 topic echo /scan
```

속도 명령 확인:

```bash
ros2 topic echo /cmd_vel
```

TF 확인:

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

## 주행 의도

- `loading`에서 출발해 `intersection_1`으로 직진합니다.
- `intersection_1` 부근에서는 제자리 회전이 아니라 커브를 돌듯이 `a_entry` 방향으로 진행합니다.
- `a_leader_slot`에 도착하면 3초 정지 후 `a_entry` 방향으로 빠져나갑니다.
- `a_entry`에서 `b_entry`로 갈 때도 제자리 회전이 아니라 회전 주행으로 이동합니다.
- `b_leader_slot`에 도착하면 3초 정지 후 `b_entry` 방향으로 빠져나갑니다.
- 제자리 회전이 필요한 위치는 `a_leader_slot`, `b_leader_slot`입니다.
