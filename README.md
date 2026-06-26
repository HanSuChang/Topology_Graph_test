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

























상용 AMR 멀티 로봇 순환 물류 시스템 최종 알고리즘 스택

1. 전역 경로 계획 및 교통 관제 (Global Planner & Fleet Management)
개별 로봇이 독립적으로 경로를 계산하여 꼬이는 것을 방지하고, 좁은 주행 레인과 교차로를 통합 제어하기 위한 사령탑 라우팅 알고리즘입니다.


[알고리즘 1] Topology Graph (토폴로지 그래프 기반 도로망)   [어느정도 완료]

역할: 맵 위의 상차 구역(Loading Zone), 하차 구역(A, B, C 공간), 로봇별 고정 대기 슬롯(Stationing Slots), 충전 노드 및 안전 대기 구역(Global Standby Nodes) 등의 거점을 노드(Node)로 정의하고 이들을 물리적 주행 레인인 링크(Link)로 연결한 가상의 맵 네트워크입니다.   

효과: 로봇들이 지정된 일방통행 구역 및 격리 레인으로만 주행하게 강제하여 다중 로봇 간의 전역 동선 교차를 원천 차단합니다.


[알고리즘 2] Time-Expanded Dijkstra (시간축 확장 다익스트라)

역할: 토폴로지 도로망 상에서 최단 경로를 계산하되, 시간($t$) 축 진입 타임라인 윈도우를 결합하여 라우팅 스케줄링을 연산합니다.

효과: 복귀 중인 RC카와 하차지에서 뒤늦게 복귀하는 터틀봇이 외길 복도나 교차로에서 마주치는 병목을 예측하여, 우선순위가 낮은 로봇을 인접 노드에 일시 정차(Hold)시킨 뒤 통과시키는 데드락(Deadlock) 프리 교통 관제를 실현합니다.


2. 리더 터틀봇 주행 알고리즘 (Leader Local Planner & Controller)
로봇팔(Robotic Arm)을 장착하여 상하차 유무에 따라 무게 중심(CoM)과 회전 관성이 극심하게 가변되는 리더 로봇 전용 고정밀 제어 알고리즘입니다.

[알고리즘 3] Cubic Spline Trajectory Generator (3차 스플라인 궤적 생성기)

역할: 다익스트라가 배정한 각진 토폴로지 전역 노드 패스들을 수학적으로 보간하여 조향 곡률이 연속적인 부드러운 곡선 궤적으로 매끄럽게 다듬어줍니다.

효과: 선회 구간 주행 시 급격한 조향 가속도 변화(Jerk)를 제거하여, 전방 로봇팔 링크에 가해지는 관성 흔들림 및 차체의 꿀렁임을 원천 차단합니다.



[알고리즘 4] Kinematic MPC (모델 예측 제어) — 전/후진 양용

역할: 터틀봇의 가속도 한계 및 차량 운동학 모델을 기반으로 미래 제어 시계를 예측하여 최적의 $v, \omega$ 명령을 제어 주기(50ms)마다 산출합니다.

효과: 물건 상하차에 따른 동적 관성 변화를 비용 함수(Cost Function) 내 가변 매개변수로 흡수하여 정밀 경로 추종성을 확보하고, 충전 스테이션 진입 시 음수 속도 프로파일($v < 0$)을 통한 0.5cm 오차 이내의 고정밀 S자 후진 주차를 제어합니다.

[알고리즘 5] 동적 서브골(Subgoal) 회피 매커니즘

역할: 주행 중 라이더(LDS-02) 센서 종단에 돌발 정적/동적 장애물이 감지되면, 토폴로지 그래프의 인접 링크를 탐색해 안전 우회 반경 내에 임시 목적지(Subgoal)를 동적으로 생성합니다.



3. 팔로워 RC카 대열 주행 알고리즘 (Follower Local Controller)
로봇팔 없이 바구니(Cargo Basket)만 장착한 채 자재 이송 셔틀 역할을 수행하며, 라즈베리파이4(RPi4)의 연산 자원을 최소화하고 대열을 동기화하기 위한 알고리즘입니다.



[알고리즘 6] Virtual Structure (가상 구조 기반 군집 제어)

역할: 공간 상에 대열 형태의 가상 기하학적 구조틀 프레임이 존재한다고 가정하고, 리더의 주행 속도와 조향에 연동된 각 RC카별 할당 타깃 TF 좌표를 실시간 분배합니다.

효과: 선회 구간 통과 시 후속 로봇들이 안쪽으로 파고들어 물류창고 벽면이나 랙(Rack) 코너에 충돌하는 내륜차(칼치기) 현상을 기하학적으로 방지합니다.



[알고리즘 7] Path History Queue (과거 궤적 추종) + PID 거리 제어

역할: 팔로워 로봇들은 리더의 현재 위치를 직선으로 쫓지 않고, 리더가 안전하게 주행하며 누적한 과거 주행 궤적 좌표 배열(Queue)을 순서대로 밟아갑니다. 종방향 가감속은 앞 차량과의 실시간 거리를 기준으로 PID 피드백 제어를 수행합니다.

효과: 리더 터틀봇이 장애물을 우회하여 S자 서브골 패스를 그리면 후속 RC카들도 동일한 궤적을 100% 모사해 안전 구역으로만 통과하며, 격자 맵 연산이 불필요해 RPi4 CPU 점유율을 극도로 낮춥니다.



4. 미션 시퀀스 및 예외 처리 총괄 (Mission Coordinator)
지속 가능한 자동화 물류 라인의 오더 큐(Mission Queue) 스케줄링과 배터리 예외 처리를 완벽하게 조율하는 소프트웨어 사령탑입니다.


[알고리즘 8] Finite State Machine (중앙 관제 FSM 상태 머신)역할: 순환 오더 배정 및 배터리 잔량 토픽 감시($\le 40\%$)에 따른 비상 충전(Auto-Docking) 상태 전환을 통제하는 시스템 타임라인 총괄 모듈입니다.









######################################################################################
[State 1: 상차 단계 (Loading)] 
➔ 복귀/상차 구역 정차 상태. 바구니가 없는 터틀봇이 로봇팔을 구동하여 바구니가 장착된 RC 1, 2호기에 순차 상차 완료.
➔ 미션 큐에서 첫 번째 목적지 [A공간] 로드 후 State 2 전환. 
                                                              
                                                               ▼ 
                                                              
[State 2: 군집 주행 단계 (Formation Driving)] 
➔ 터틀봇 앞장서서 안내 주행 (Dijkstra + Spline + MPC). 
➔ RC 1, 2호기는 터틀봇의 과거 궤적 큐를 복사 추종하며 가상 구조 대열 유지. 

                                                              ▼ 
[State 3: A공간 진입 및 지정석 정차 (Stationing)]
 ➔ 하차지(A공간) 진입 순간 군집 모드 해제(Off).
  ➔ RC 1, 2호기는 글로벌 노드 좌표 데이터를 기반으로 각자 할당된 지정 정차석(Slot 1, 2)으로 이동 후 Lock 상태 대기. 

                                                              ▼ 
[State 4: 순차 하차 및 개별 복귀 (Sequential Process)] 
➔ 터틀봇이 멈춰 있는 RC 1호기 앞으로 이동 ➔ 로봇팔 하차 작업 완료 ➔ RC 1호기 '복귀 승인' 신호 수신 후 혼자 단독 복귀 출발. ➔ 그동안 RC 2호기는 자기 차례가 아니므로 고정석에서 미동도 없이 Lock 상태 대기  ➔ RC 1 복귀 후 터틀봇이 RC 2 하차 완료 및 복귀 출발. 

                                                              ▼ 
[State 5: 복귀 및 지속 순환 (Mission Queue Loop)] 
➔ 팔로워들과 리더 터틀봇이 모두 복귀 노드로 복귀 완료. 
➔ FSM 스케줄러가 [A공간 미션 완료]를 확인하고, 다음 미션 큐인 [B공간] (이후 C공간 -> 다시 A공간) 오더를 로드하여 재상차 및 이송 루프 반복. 

                                                             ▼ 
[Emergency State: 배터리 잔량 40% 이하 감지 시 예외 처리 인터럽트] 
➔ 주행 중 터틀봇의 배터리가 40% 이하로 떨어지면 FSM은 진행 중인 물류 순환 미션을 즉시 일시 정지(Pause).
➔ [팔로워 RC 1, 2호기]: 터틀봇의 S자 주차 회전 반경을 확보해주기 위해 군집 해제 후, 주변의 안전 글로벌 대기 노드(Global Node)로 각자 이동하여 Lock 상태 대기. 
➔ [리더 터틀봇]: 역방향 MPC 제어기를 가동하여 충전 도킹 스테이션으로 고정밀 S자 후진 주차 진입 후 충전 시작. 
➔ [배터리 완충 시(예: 90%)]: RC카 락 해제 및 집결 노드 호출 ➔ 중단되었던 미션 구간(B 또는 C) 오더를 복구하여 순환 루프 재개(Resume).



