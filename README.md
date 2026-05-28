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




