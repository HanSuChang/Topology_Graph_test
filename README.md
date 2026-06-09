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

## 장애물 회피 구조

현재 주행 노드는 topology graph 기반 미션 주행 위에 라이다 안전 회피, A* + Pure Pursuit, MPPI, corridor pass, 작업 지점 장애물 대기 로직을 계층적으로 적용합니다.

핵심 파일:

- `src/amr_topology/src/mission_loop.cpp`
  - 전체 미션 순서 실행
  - 목표 노드 이동
  - 경유 노드 스킵
  - MPPI rescue 호출
  - corridor pass 통과 가능 판단
  - 작업 지점 장애물 대기 및 `/sound` 서비스 호출
- `src/amr_topology/src/local_astar_pure_pursuit.cpp`
  - 라이다 기반 장애물 감지
  - `stop_and_plan`
  - A* detour 생성
  - Pure Pursuit detour 추종
  - `side_escape`
- `src/amr_topology/config/topology.yaml`
  - `loading`, `intersection`, `entry`, `slot` 좌표와 노드 type 정의
- `src/amr_topology/config/mppi_controller.yaml`
  - Nav2 `controller_server`의 MPPI controller 설정
- `src/amr_topology/launch/mppi_controller.launch.py`
  - MPPI controller server 실행 launch

### 전체 판단 흐름

```text
토폴로지 목표 노드로 주행
-> 장애물 없음: 일반 목표 추종 주행
-> 넓은 공간 장애물: A* + Pure Pursuit 회피
-> 측면 근접: side_escape
-> 좁은 벽-장애물 사이: corridor_pass 판단 후 MPPI 주행
-> 통과 불가능: 정지
-> 경유 노드 막힘: 다음 노드로 스킵
-> 작업 노드 막힘: 정지 + /sound value=2 반복 호출 + 장애물 제거 대기
```

## 일반 주행

장애물이 없을 때는 현재 위치와 목표 노드 좌표를 비교해 `/cmd_vel`을 직접 발행합니다.

```text
현재 위치
-> 목표 노드 방향 계산
-> heading error 계산
-> angular.z 보정
-> linear.x 전진
```

이 로직은 `mission_loop.cpp`의 `make_drive_command()`에서 수행합니다.

## A* + Pure Pursuit 회피

넓은 공간 또는 여유 있는 장애물 회피는 `LocalAStarPurePursuit`가 담당합니다.

동작 흐름:

```text
목표 방향에 장애물 감지
-> stop_and_plan
-> local map + scan 기반 planning grid 생성
-> A*로 detour path 생성
-> Pure Pursuit로 detour path 추종
```

주요 로그:

```text
Local A* Pure Pursuit active: stop_and_plan
Local A* Pure Pursuit active: follow_detour
```

주요 거리 기준:

```text
obstacle_emergency_stop_distance = 0.22
obstacle_side_stop_distance = 0.24
obstacle_target_stop_distance = 0.42
```

## side_escape

측면이 너무 가까우면 `side_escape`가 동작합니다.

조건:

```text
min(left_min, right_min) <= obstacle_side_stop_distance
```

동작:

```text
가까운 쪽 반대 방향으로 회전
전방이 열려 있고 emergency 거리가 아니면 천천히 전진
```

주요 로그:

```text
Local A* Pure Pursuit active: side_escape
```

## MPPI rescue

A* + Pure Pursuit가 막히거나 `side_escape`가 오래 지속되면 MPPI rescue가 켜집니다.

조건:

```text
local_plan.blocked == true
또는 stop_and_plan 지속 시간이 mppi_rescue_stop_and_plan_seconds 이상
또는 side_escape 지속 시간이 mppi_rescue_side_escape_seconds 이상
```

MPPI는 Nav2 전체 navigation stack을 쓰는 것이 아니라, `controller_server`의 `FollowPath` action만 사용합니다.

```text
mission_loop
-> /follow_path action
-> controller_server
-> MPPI controller
-> /cmd_vel
```

## corridor_pass

좁은 벽-장애물 사이에서는 MPPI를 바로 고민시키지 않고, 먼저 Waffle Pi가 실제로 지나갈 수 있는 통로 폭인지 계산합니다.

기준:

```text
left_min + right_min = corridor width
```

Waffle Pi 기본값:

```text
corridor_robot_width = 0.306
corridor_side_clearance = 0.07
corridor_min_passage_width = 0.45
corridor_hard_stop_width = 0.40
corridor_max_lateral_offset = 0.24
```

판정:

```text
width >= 0.45
-> 통과 가능
-> mode=corridor path 생성
-> 통로 중심 쪽으로 MPPI path 전달

width < 0.40
또는 한쪽 side clearance가 너무 작음
-> 통과 불가
-> 정지 유지
```

주요 로그:

```text
MPPI rescue path: mode=corridor ...
Corridor blocked: width=... required=...; holding position
```

이 구조는 사람이 보기에는 지나갈 수 있어 보이지만, costmap inflation 때문에 MPPI가 버벅이는 상황을 줄이기 위해 추가했습니다. 라이다 기준으로 Waffle Pi 몸체 폭과 안전마진을 수치로 판단한 뒤, 통과 가능하면 통로 중심 path를 MPPI에 제공합니다.

## 경유 노드 장애물 처리

아래 4개 노드는 작업 지점이 아니라 경유 지점으로 봅니다.

```text
intersection_1
intersection_2
a_entry
b_entry
```

이 노드가 막혀 있고 다음 목표 방향이 clear하면 현재 노드를 스킵합니다.

```text
loading -> intersection_1 -> a_entry -> a_leader_slot
```

예를 들어 `a_entry`가 막혔고 `a_leader_slot` 방향이 clear하면:

```text
a_entry 스킵
-> a_leader_slot으로 이동
```

## 작업 노드 장애물 처리

아래 4개 노드는 로봇팔 임무 예정 지점입니다.

```text
a_leader_slot
b_leader_slot
a_leader_slot_precision
b_leader_slot_precision
```

이 노드가 막혀 있으면 스킵하지 않습니다.

동작:

```text
목표 작업 노드 blocked
-> 정지
-> /sound 서비스 value=2 반복 호출
-> 장애물이 치워질 때까지 대기
-> 장애물이 치워지면 임무 재개
```

사운드 서비스:

```text
/sound [turtlebot3_msgs/srv/Sound]
value = 2
```

수동 테스트:

```bash
ros2 service call /sound turtlebot3_msgs/srv/Sound "{value: 2}"
```

## 권장 mission_loop 실행 명령

MPPI controller를 먼저 실행합니다.

```bash
ros2 launch amr_topology mppi_controller.launch.py
```

그 다음 mission loop를 실행합니다.

```bash
ros2 run amr_topology mission_loop --ros-args \
  -p base_frame:=base_footprint \
  -p enable_lidar_safety:=true \
  -p scan_topic:=/scan \
  -p obstacle_avoid_linear_speed:=0.075 \
  -p obstacle_avoid_angular_speed:=0.22 \
  -p obstacle_stop_and_scan_seconds:=0.8 \
  -p obstacle_emergency_stop_distance:=0.22 \
  -p obstacle_side_stop_distance:=0.24 \
  -p rear_stop_distance:=0.18 \
  -p obstacle_target_stop_distance:=0.42 \
  -p obstacle_rotate_in_place_heading_threshold:=0.35 \
  -p obstacle_rotate_in_place_resume_threshold:=0.18 \
  -p obstacle_rotate_in_place_max_heading:=1.05 \
  -p enable_mppi_rescue:=true \
  -p mppi_follow_path_action:=follow_path \
  -p mppi_rescue_timeout_seconds:=18.0 \
  -p mppi_rescue_stop_and_plan_seconds:=3.0 \
  -p enable_corridor_pass:=true \
  -p corridor_robot_width:=0.306 \
  -p corridor_side_clearance:=0.07 \
  -p corridor_min_passage_width:=0.45 \
  -p corridor_hard_stop_width:=0.40 \
  -p corridor_max_lateral_offset:=0.24 \
  -p blocked_target_sound_value:=2 \
  -p blocked_target_beep_period:=1.0
```














