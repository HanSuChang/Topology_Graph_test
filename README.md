# 매니퓰레이터가 탑재된 터틀봇 기반 AMR 물류 이송 시스템

## Topology Graph 기반 자율주행 물류 로봇 + RC카 추종 + 로봇팔 + GUI 관제 시스템

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white)
![C++](https://img.shields.io/badge/C++-Mission%20Control-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)
![Python](https://img.shields.io/badge/Python-ROS2%20Bridge%20%2B%20Vision-3776AB?style=for-the-badge&logo=python&logoColor=white)
![Go](https://img.shields.io/badge/Go-Backend-00ADD8?style=for-the-badge&logo=go&logoColor=white)
![React](https://img.shields.io/badge/React-GUI-61DAFB?style=for-the-badge&logo=react&logoColor=111111)
![TypeScript](https://img.shields.io/badge/TypeScript-Frontend-3178C6?style=for-the-badge&logo=typescript&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-Vision-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)
![Nav2](https://img.shields.io/badge/Nav2-MPPI%20%2B%20NavigateToPose-0A66C2?style=for-the-badge)
![A*](https://img.shields.io/badge/A*-Local%20Path%20Planning-FF6F00?style=for-the-badge)
![Pure Pursuit](https://img.shields.io/badge/Pure%20Pursuit-Path%20Tracking-2E7D32?style=for-the-badge)
![WebSocket](https://img.shields.io/badge/WebSocket-Realtime%20Dashboard-111827?style=for-the-badge)
![SQLite](https://img.shields.io/badge/SQLite-Mission%20History-003B57?style=for-the-badge&logo=sqlite&logoColor=white)

이 저장소는 ROS 2 Humble 환경에서 TurtleBot3 기반 자율주행 로봇, RC카 팔로워, 비전 기반 로봇팔, 웹 GUI 관제 시스템을 하나의 프로젝트로 통합한 코드입니다. 단순히 로봇을 한 지점으로 이동시키는 예제가 아니라, 실제 물류 시나리오를 가정해 "상차 지점에서 출발 → A/B 작업 구역 이동 → 장애물 회피 → RC카 추종 → ArUco 정밀 접근 → 로봇팔 픽앤플레이스 → 충전소 이동" 흐름을 구성합니다.

핵심 목표는 다음과 같습니다.

- SLAM map 위에 정의한 Topology Graph를 따라 TurtleBot이 안정적으로 이동합니다.
- TurtleBot이 현재 위치가 아니라 "지나간 과거 궤적"을 RC카가 뒤따르는 추종 주행을 수행합니다.
- LiDAR 장애물 감지, Local A* 우회 경로, Pure Pursuit 경로 추종, MPPI rescue를 조합해 좁은 통로와 돌발 장애물을 처리합니다.
- A/B 작업 구역과 충전소 같은 의미 있는 노드를 구분하고, 각 노드별로 다른 주행/정지/정밀 접근 전략을 적용합니다.
- 카메라 기반 색상 객체 인식과 planar inverse kinematics를 이용해 로봇팔이 물체를 집고 놓는 동작을 수행합니다.
- React GUI, Go 백엔드, Python ROS2 bridge를 통해 로봇 상태, 경로, 미션, 지도, 로그를 웹 대시보드에서 관제합니다.

## 전체 시스템 구조

```text
Topology_Graph_test
├── src/amr_topology/                 # ROS2 주행, 추종, 로봇팔 패키지
│   ├── src/                          # C++ 주행/추종/비전 노드
│   ├── scripts/                      # Python 로봇팔/ESP32/카메라 보조 노드
│   ├── config/                       # topology, AMCL, MPPI 설정
│   ├── launch/                       # 주행, 로봇팔, 카메라, MPPI launch
│   ├── maps/                         # map.yaml / map.pgm
│   └── rviz/                         # RViz 설정
└── gui_system/                       # 웹 관제 시스템
    ├── frontend/                     # React + Vite + TypeScript 대시보드
    ├── backend/                      # Go + Gin API/WebSocket 서버
    ├── bridge/                       # Python ROS2 bridge
    ├── maps/                         # GUI용 map/nodes
    ├── launch/                       # GUI stack launch
    └── proto/                        # bridge 메시지 정의
```

실행 시 데이터 흐름은 다음과 같습니다.

```text
SLAM map + topology.yaml
        │
        ▼
mission_loop / mission2_loop
        │
        ├─ /cmd_vel --------------------------> TurtleBot3 주행
        ├─ /turtlebot/pose -------------------> RC카 추종 기준 pose history
        ├─ /rc_car/follower_mode -------------> RC카 follow/stop/align 모드
        ├─ /aruco_marker/enable --------------> ArUco 정밀 접근 on/off
        ├─ A_mission_start -------------------> 로봇팔 작업 시작
        └─ /mission_command <------------------ GUI/bridge 충전소 명령

rc_car_follower_node
        ├─ /rc_car/odom ----------------------> GUI follower 마커
        ├─ /rc_car/command -------------------> PWM 명령 로그
        └─ Arduino serial --------------------> RC카 모터 제어

arm_vision_gripper_node
        ├─ camera image ----------------------> 색상 객체 검출
        ├─ /arm_servo_angles -----------------> ESP32 servo bridge
        └─ /arm_vision_status ----------------> 작업 상태

gui_system
        ├─ frontend --------------------------> 웹 대시보드
        ├─ backend ---------------------------> REST API + WebSocket hub
        └─ bridge ----------------------------> ROS2 topic/action/service 연동
```

## 주요 기술 스택

| 영역 | 사용 기술 |
| --- | --- |
| 로봇 미들웨어 | ROS 2 Humble, rclcpp, rclpy |
| 위치 추정 | SLAM map, AMCL, TF `map -> base_footprint` |
| 주행 제어 | Topology Graph, 직접 `/cmd_vel` 제어, Pure Pursuit |
| 장애물 회피 | LiDAR `/scan`, Local A*, occupancy grid, corridor pass, MPPI rescue |
| Nav2 연동 | `FollowPath`, `NavigateToPose`, MPPI controller, local costmap |
| 추종 주행 | leader pose history 기반 delayed following, encoder dead reckoning |
| 비전 | OpenCV, HSV color segmentation, ArUco marker detection |
| 로봇팔 | 5-servo manipulator, planar IK, ESP32 serial bridge |
| GUI | React 18, TypeScript, Vite, Tailwind, Canvas map renderer |
| 백엔드 | Go 1.22, Gin, WebSocket hub, SQLite, idempotency, audit log |
| 브릿지 | Python ROS2 bridge, WebSocket command bridge, ROS topic subscription |

## 코드 구조

### `src/amr_topology`

ROS2 로봇 제어의 중심 패키지입니다.

```text
src/amr_topology/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── topology.yaml                 # 노드/엣지 기반 주행 그래프
│   ├── amcl.yaml                     # AMCL localization 설정
│   ├── mppi_controller.yaml          # Nav2 MPPI rescue 설정
│   └── mission_loop.yaml             # 미션 파라미터
├── include/amr_topology/
│   └── local_astar_pure_pursuit.hpp
├── src/
│   ├── mission_loop.cpp              # A 구역 중심 미션 루프
│   ├── mission2_loop.cpp             # B 구역/충전소 확장 미션 루프
│   ├── local_astar_pure_pursuit.cpp  # Local A* + Pure Pursuit 장애물 회피
│   ├── rc_car_follower_node.cpp      # RC카 과거 궤적 추종
│   ├── topology_follower.cpp         # 단일 route 테스트용 follower
│   ├── topology_marker_node.cpp      # topology 노드 RViz marker
│   ├── aruco_marker_detector_node.cpp
│   ├── image_rotate_node.cpp
│   └── image_rotate_compress_node.cpp
├── scripts/
│   ├── arm_vision_gripper_node.py    # 비전 기반 로봇팔 자동 픽앤플레이스
│   ├── esp32_servo_bridge_node.py    # ESP32 servo serial bridge
│   ├── servo_hold_90_node.py
│   └── turtle_mission
├── launch/
│   ├── turtlebot_mission.launch.py
│   ├── mission_loop_aruco.launch.py
│   ├── mppi_controller.launch.py
│   ├── arm_vision_pick.launch.py
│   ├── picam_rotate_esp32_bridge.launch.py
│   └── ...
└── maps/
    ├── map.yaml
    └── map.pgm
```

### `gui_system`

웹 관제 시스템입니다.

```text
gui_system/
├── frontend/                         # React dashboard
│   ├── src/dashboard/cards/          # 미션, 로봇 상태, 로봇팔, 로그 카드
│   ├── src/dashboard/navigation_map/ # Canvas 기반 지도/경로/마커 렌더링
│   ├── src/hooks/                    # WebSocket, mission, robot state hooks
│   └── src/lib/                      # API, websocket, event log
├── backend/                          # Go backend
│   ├── cmd/gui_main/main.go
│   ├── internal/api/                 # mission/status/logs/map/analytics API
│   ├── internal/gateway/             # WebSocket fan-out hub
│   ├── internal/database/            # SQLite repository
│   ├── internal/bridge/              # bridge client
│   └── internal/topology/            # nodes.yaml loader
├── bridge/
│   └── ros2_bridge/                  # ROS2 topic/action <-> backend bridge
├── maps/
│   ├── map.yaml
│   ├── map.pgm
│   └── nodes.yaml
└── scripts/
```

## Topology Graph 기반 주행

이 프로젝트의 주행은 좌표 하나를 임의로 찍어 Nav2에 맡기는 방식이 아니라, 물류 환경의 주요 지점을 graph node로 정의하고 그 node 사이를 이동하는 방식입니다.

`src/amr_topology/config/topology.yaml`에는 다음과 같은 노드가 정의되어 있습니다.

- `loading`: 출발/상차 위치
- `intersection_1`, `intersection_2`: 분기점 또는 통과 지점
- `a_entry`, `b_entry`: A/B 작업 구역 진입 노드
- `a_leader_slot`, `b_leader_slot`: TurtleBot이 정밀 정차하는 작업 슬롯
- `charger_entry`: 충전소 진입 전 정렬 노드
- `charger_front`: 최종 충전 위치
- `standby_1`: 대기 위치

엣지는 노드 사이 연결 관계와 속도 제한을 표현합니다. 예를 들어 `loading -> intersection_1 -> a_entry -> a_leader_slot`은 A 구역으로 가는 대표 경로이고, `loading -> intersection_2 -> b_entry -> b_leader_slot`은 B 구역으로 가는 경로입니다.

이 구조의 장점은 다음과 같습니다.

- 물류 현장의 "의미 있는 위치"를 코드와 지도에서 같은 이름으로 관리할 수 있습니다.
- A/B 작업 구역, 충전소, 교차점처럼 서로 다른 행동이 필요한 지점을 type으로 구분할 수 있습니다.
- GUI에서도 같은 노드 ID를 사용하므로 사용자가 클릭한 목적지를 주행 로직의 목표 노드와 직접 연결할 수 있습니다.
- 장애물이 생겼을 때 중간 경유 노드는 skip하고 다음 노드로 넘어가는 판단을 넣을 수 있습니다.

## TurtleBot 미션 주행 로직

`mission_loop.cpp`와 `mission2_loop.cpp`가 TurtleBot 주행 미션을 담당합니다.

- `mission_loop`: A 구역 중심 미션 루프
- `mission2_loop`: B 구역, 충전소 이동, Nav2 충전소 handoff가 포함된 확장 루프

두 노드는 공통적으로 다음 입력을 사용합니다.

| 입력 | 역할 |
| --- | --- |
| TF `map -> base_footprint` | 현재 로봇 pose 추정 |
| `/scan` | LiDAR 기반 장애물 감지 |
| `/map` | occupancy grid 기반 local A* 계획 |
| `/mission_command` | GUI/bridge에서 오는 충전소 이동 명령 |
| `/rc_car/odom` | RC카 위치 확인 |
| `/rc_car/slot_wait_status` | RC카가 특정 stop 지점에 도달했는지 확인 |
| `/aruco_marker/target` | ArUco 정밀 접근을 위한 마커 상대 위치 |

주요 출력은 다음과 같습니다.

| 출력 | 역할 |
| --- | --- |
| `/cmd_vel` | TurtleBot 직접 속도 명령 |
| `/turtlebot/pose` | RC카와 GUI가 쓰는 leader pose |
| `/rc_car/follower_mode` | RC카 follow/stop/align 모드 제어 |
| `/aruco_marker/enable` | ArUco detector 활성화 |
| `A_mission_start` | 로봇팔 자동 작업 시작 |

일반 주행은 다음 순서로 동작합니다.

```text
1. topology.yaml에서 목표 node 좌표를 읽음
2. TF에서 현재 TurtleBot pose를 가져옴
3. 현재 위치와 목표 node 사이의 거리, 목표 heading을 계산
4. heading error를 angular.z로 보정
5. 목표 방향이 크게 틀어지지 않았으면 linear.x로 전진
6. waypoint tolerance 안에 들어오면 다음 node로 전환
7. 최종 slot에 도달하면 정지 후 yaw 정렬
```

이 방식은 Nav2 global planner에 전부 맡기는 구조가 아니라, 물류 시나리오에 맞게 짧은 topology segment를 직접 제어합니다. 직접 제어를 쓰는 이유는 RC카 추종 모드, 작업 슬롯 정밀 정지, ArUco 접근, 충전소 handoff처럼 미션별 상태 전이가 많이 들어가기 때문입니다.

## 장애물 회피: Local A* + Pure Pursuit

`local_astar_pure_pursuit.cpp`는 로컬 장애물 회피를 담당합니다. 이 노드는 `/scan`과 `/map`을 이용해 로봇 주변의 planning grid를 만들고, 막힌 목표 방향을 우회하는 짧은 detour path를 생성합니다.

### A* 개념

A*는 시작점에서 목표점까지 격자 위 최단 경로를 찾는 탐색 알고리즘입니다. 단순 Dijkstra와 달리 목표까지의 휴리스틱 거리를 같이 사용해 불필요한 탐색을 줄입니다.

이 코드에서는 A*가 다음 방식으로 쓰입니다.

- `/map` occupancy grid에서 벽/장애물 cell을 읽습니다.
- `/scan`으로 들어온 실시간 장애물을 dynamic obstacle로 grid에 반영합니다.
- 로봇 반경, inflation radius를 적용해 실제 로봇이 지나가기 어려운 영역을 넓게 막습니다.
- 현재 pose 주변에서 최대 계획 거리 안의 local grid를 구성합니다.
- 목표 방향이 막히면 가까운 free goal을 찾고 A*로 우회 경로를 계산합니다.

### Pure Pursuit 개념

Pure Pursuit는 경로 위의 lookahead point를 하나 고르고, 로봇이 그 점을 따라가도록 조향하는 경로 추종 알고리즘입니다. 경로 전체를 복잡하게 최적화하지 않고 "조금 앞의 목표점"을 따라가므로 구현이 단순하고 실시간성이 좋습니다.

이 코드에서는 A*가 만든 detour path를 Pure Pursuit로 따라갑니다.

```text
LiDAR 장애물 감지
-> 목표 방향 blocked 판단
-> local grid 생성
-> A*로 detour path 생성
-> Pure Pursuit로 lookahead point 추종
-> 장애물이 사라지면 topology 직접 주행으로 복귀
```

상태는 크게 다음처럼 나뉩니다.

- `Normal`: 장애물 없이 topology 목표로 직접 주행
- `StopAndPlan`: 목표 방향이 막혀 정지 후 local plan 생성
- `FollowDetour`: A* detour path를 Pure Pursuit로 추종
- `Blocked`: local planner만으로 해결이 어려운 상태
- `side_escape`: 좌우 측면이 너무 가까울 때 반대 방향으로 빠져나가는 동작

## MPPI Rescue

MPPI(Model Predictive Path Integral)는 여러 제어 입력 샘플을 rollout하고, cost가 낮은 궤적을 선택하는 sampling 기반 local trajectory optimization 기법입니다. 장애물 cost, 목표 접근 cost, 경로 정렬 cost 등을 종합해서 속도 명령을 고릅니다.

이 프로젝트에서 MPPI는 기본 주행기가 아니라 "rescue controller"로 사용됩니다.

일반 상황:

```text
Topology Graph 직접 주행
```

장애물 발생:

```text
Local A* + Pure Pursuit 우회
```

로컬 우회 실패 또는 장시간 blocked:

```text
MPPI FollowPath action에 짧은 rescue path 전달
```

`mppi_controller.yaml`에서는 Nav2 `controller_server`의 `FollowPath` plugin으로 `nav2_mppi_controller::MPPIController`를 사용합니다. 주요 설정은 다음과 같습니다.

- `motion_model: DiffDrive`: TurtleBot3 차동구동 모델
- `time_steps: 48`, `model_dt: 0.05`: 약 2.4초 horizon
- `batch_size: 2200`: 많은 후보 궤적 샘플링
- `vx_max: 0.17`, `wz_max: 1.00`: 저속 실내 주행에 맞춘 제한
- `CostCritic`, `GoalCritic`, `PathAlignCritic`, `PathFollowCritic`, `PreferForwardCritic`: 충돌 회피, 목표 접근, 경로 정렬, 전진 선호 cost 적용

MPPI rescue가 계속 켜져 있으면 직접 제어와 충돌할 수 있기 때문에, `mission_loop`는 다음 조건에서 다시 local A* 주행으로 handoff합니다.

- 일정 시간 이상 MPPI가 주행했고 전방/측면 clearance가 확보됨
- 목표 node tolerance 안에 도달함
- MPPI rescue timeout 초과

## Corridor Pass

좁은 통로에서는 inflation radius가 크면 실제로 지나갈 수 있는 공간도 막힌 것으로 판단될 수 있습니다. 이를 보완하기 위해 `mission_loop`에는 corridor pass 판단이 들어 있습니다.

기본 아이디어는 LiDAR의 좌우 최소 거리로 통로 폭을 추정하는 것입니다.

```text
corridor_width = left_min + right_min
```

판단 파라미터는 다음과 같습니다.

- `corridor_robot_width`: TurtleBot 폭
- `corridor_side_clearance`: 좌우 안전 여유
- `corridor_min_passage_width`: 통과 가능 최소 폭
- `corridor_hard_stop_width`: 이보다 좁으면 즉시 정지
- `corridor_max_lateral_offset`: 통로 중심에서 벗어난 정도 허용값

통과 가능하면 통로 중심으로 정렬하면서 이동하고, 너무 좁거나 중심에서 크게 벗어나면 정지 또는 rescue로 넘깁니다. 이 로직은 MPPI가 costmap 때문에 보수적으로 멈추는 상황을 줄이기 위한 보조 판단입니다.

## 충전소 이동과 도킹

`mission2_loop.cpp`에는 `/mission_command`로 `start_charger_parking`을 받으면 현재 미션을 중단하고 충전소 이동을 수행하는 흐름이 있습니다.

충전소 이동은 두 단계로 나뉩니다.

1. `charger_entry`까지 Nav2 `NavigateToPose`로 이동
2. `charger_front` 최종 위치까지 L-shaped parking 수행

충전소 주변은 좁고 costmap inflation에 민감하기 때문에, 충전소 이동 시에는 global costmap inflation parameter를 일시적으로 낮추고 복구하는 로직도 포함되어 있습니다. 또한 NavigateToPose가 장애물 근처에서 stuck 상태로 보이면 corridor escape를 수행한 뒤 다시 `charger_entry` 접근을 재시도합니다.

## ArUco 기반 정밀 접근

작업 슬롯 근처에서는 topology 좌표만으로는 마지막 수 cm 단위 정렬이 부족할 수 있습니다. 이를 보완하기 위해 `aruco_marker_detector_node.cpp`가 OpenCV ArUco marker를 검출합니다.

동작 방식은 다음과 같습니다.

- `/aruco_marker/enable`이 true일 때만 카메라를 열고 검출을 시작합니다.
- OpenCV `cv::aruco::detectMarkers`로 지정된 marker ID를 찾습니다.
- 마커 중심의 이미지 x offset을 정규화해 `target.point.x`로 발행합니다.
- 마커 실제 크기와 focal length를 이용해 거리 추정값을 `target.point.y`로 발행합니다.
- `mission_loop`는 이 값을 받아 회전/전진 속도를 줄여가며 정밀 접근합니다.

이 구조는 "그래프 기반으로 큰 위치까지 이동 → ArUco로 작업 지점 최종 접근"이라는 계층형 정밀 주행 구조입니다.

## RC카 추종 주행

`rc_car_follower_node.cpp`는 TurtleBot 뒤를 따라가는 RC카 follower입니다. 핵심은 TurtleBot의 현재 위치를 바로 쫓지 않고, TurtleBot이 지나간 과거 궤적을 history로 저장한 뒤 일정 거리 뒤의 목표점을 선택한다는 점입니다.

### 왜 과거 궤적 추종인가?

현재 TurtleBot 위치만 목표로 삼으면 RC카가 코너에서 안쪽으로 파고들거나, leader가 정지했을 때 follower가 급하게 접근하는 문제가 생깁니다. 과거 궤적을 따라가면 leader가 실제로 지나간 안전한 경로를 follower가 재현하므로 더 안정적인 추종이 가능합니다.

### 입력과 출력

| 입력 | 역할 |
| --- | --- |
| `/turtlebot/pose` | leader의 map 기준 pose history |
| `/cmd_vel` | leader 이동 상태 보조 판단 |
| `/rc_car/follower_mode` | follow, stop, align 등 모드 |
| Arduino serial | 좌우 encoder tick |

| 출력 | 역할 |
| --- | --- |
| `/rc_car/odom` | encoder dead reckoning으로 계산한 RC카 pose |
| `/rc_car/command` | PWM 명령 문자열 |
| `/rc_car/encoder_ticks` | 누적 encoder tick |
| `/rc_car/serial_debug` | serial raw line |
| `/rc_car/slot_wait_status` | 특정 정지 위치 도달 상태 |

### 제어 구조

RC카는 다음 정보를 기반으로 PWM을 계산합니다.

- RC카 현재 pose: encoder tick 기반 dead reckoning
- 목표 pose: TurtleBot pose history에서 `target_distance`만큼 뒤쪽 sample
- 거리 오차: 목표점까지 얼마나 남았는지
- heading error: RC카 yaw와 목표 방향의 각도 차이

개념적으로는 다음과 같습니다.

```text
leader pose history 저장
-> RC카 현재 odom 계산
-> history에서 추종 목표점 선택
-> distance error + heading error 계산
-> 좌/우 PWM 차등 계산
-> Arduino serial로 left_pwm,right_pwm 전송
```

RC카는 절대 위치 센서가 없기 때문에 시작 pose 초기화가 중요합니다.

- `auto_initialize_odom_from_leader=true`: TurtleBot pose 기준으로 초기 RC카 위치 추정
- `auto_initialize_odom_from_leader=false`: `initial_rc_x`, `initial_rc_y`, `initial_rc_yaw`를 직접 사용

## 로봇팔 비전 픽앤플레이스

`arm_vision_gripper_node.py`는 카메라 영상에서 목표 색상 물체를 찾고, 5개 servo를 제어해 물체를 집어 drop pose로 옮기는 노드입니다.

### 비전 처리

카메라 입력은 다음 중 하나를 사용할 수 있습니다.

- 직접 카메라 device
- `/picam/image_raw`
- `/picam/image_rotated/compressed`

노드는 OpenCV 기반으로 색상 segmentation을 수행합니다. 목표 색상은 `red` 또는 `blue`로 바꿀 수 있고, 검출된 contour의 중심을 이미지 중심과 비교해 오차를 계산합니다.

```text
camera frame
-> HSV/color mask
-> contour 검출
-> object center 추정
-> image center와 error_u, error_v 계산
```

### Servo 정렬

로봇팔의 base servo와 link servo는 이미지 오차를 줄이는 방향으로 조금씩 움직입니다.

- `error_u`: 이미지 좌우 오차, base 회전 servo `s0` 보정
- `error_v`: 이미지 상하 오차, link servo `s1`, `s2`, wrist `s3` 보정
- `center_tolerance_u/v`: 이 범위 안에 들어오면 정렬 완료로 판단
- `auto_align_hold_sec`: 중심 정렬 상태를 일정 시간 유지하면 접근 단계로 전환

### Planar IK

팔의 링크 길이는 코드에 mm 단위로 정의되어 있습니다.

- `d1_mm = 110`
- `d2_mm = 70`
- `d3_mm = 105`

`forward_kinematics`는 현재 servo angle에서 end-effector의 r/z 위치를 계산하고, `solve_planar_ik`는 목표 r/z와 wrist angle을 만족하는 servo angle을 계산합니다. 이 구조로 물체에 가까워질수록 단순히 각도를 찍는 것이 아니라, 링크 기하를 고려해 점진적으로 접근합니다.

### Pick and Place Sequence

정렬과 접근이 끝나면 다음 sequence를 수행합니다.

```text
gripper open
-> wrist/link를 물체 쪽으로 삽입
-> gripper close
-> drop pose로 이동
-> gripper open
-> start pose 복귀
```

Servo 명령은 `/arm_servo_angles`에 `Int32MultiArray`로 발행됩니다. `esp32_servo_bridge_node.py`는 이 topic을 구독하고 ESP32 serial로 `angle0,angle1,angle2,angle3,gripper` 형식의 명령을 전송합니다. ESP32가 `OK` 또는 `ERR` 응답을 보내면 `/arm_servo_status`로 상태를 발행합니다.

## GUI / 백엔드 / 브릿지

`gui_system`은 로봇 관제를 위한 별도 웹 시스템입니다.

### Frontend

`gui_system/frontend`는 React + TypeScript + Vite 기반입니다.

주요 화면 구성은 다음과 같습니다.

- MissionControlCard: 목적지 선택, 미션 시작/정지/재개
- MapPanel: SLAM map, topology node, global/local path, robot marker 표시
- RobotStatusCard: TurtleBot/RC카 상태 표시
- ManipulatorCard: 로봇팔 상태 표시
- PoseEstimateCard: GUI에서 AMCL initial pose 설정
- LogsStrip: PC/폰 등 여러 클라이언트가 공유하는 이벤트 로그

지도 렌더링은 Canvas 기반입니다. `MapRenderer`가 map image, topology node, edge, robot pose, scan point, planned/local path를 직접 그립니다.

WebSocket으로 수신하는 주요 envelope은 다음과 같습니다.

- `robot_pose`: TurtleBot, RC카 위치
- `mission_state`: 미션 상태, 현재 goal, route, ETA
- `path_data`: 백엔드가 계산한 topology route
- `planned_path`: 주행 노드 또는 bridge가 보낸 전역 경로
- `local_path`: 장애물 회피 경로
- `scan_points`: LiDAR point cloud
- `alert`, `system_log`, `client_log`: 상태/로그
- `pose_estimate`: 위치 추정 명령 echo

### Backend

`gui_system/backend`는 Go + Gin 기반 서버입니다.

역할은 다음과 같습니다.

- REST API 제공
- WebSocket hub로 프론트엔드에 상태 fan-out
- topology node/edge 로딩
- 미션 명령 수락 및 idempotency 처리
- bridge가 offline이어도 미션 의도(route, goal, ETA)를 GUI에 반영
- SQLite에 미션 이력, telemetry, ETA 분석 데이터 저장
- audit log 기록

주요 API는 다음과 같습니다.

- `GET /api/v1/state/current`
- `GET /api/v1/state/robots`
- `GET /api/v1/state/map`
- `GET /api/v1/map/image`
- `POST /api/v1/missions/start`
- `POST /api/v1/missions/pause`
- `POST /api/v1/missions/resume`
- `POST /api/v1/emergency/stop`
- `POST /api/v1/pose_estimate`
- `GET /ws`

백엔드는 `target_node`를 받으면 `nodes.yaml`의 graph에서 현재 leader 위치와 목표 노드 사이 경로를 계산합니다. 이때 Dijkstra 방식으로 edge cost를 기반으로 route를 만들고, 그 route를 `mission_state`와 `path_data`로 GUI에 broadcast합니다.

### ROS2 Bridge

`gui_system/bridge`는 Python rclpy 기반 ROS2 bridge입니다.

역할은 다음과 같습니다.

- ROS2 topic을 구독해 WebSocket으로 backend에 전달
- backend 명령을 ROS2 topic/action으로 변환
- GUI 위치추정 명령을 `/initialpose`로 발행
- 충전소 이동 명령을 `/mission_command`로 발행
- Nav2 `NavigateToPose` action goal 전송

`bridge/config.yaml`에서는 다음 robot ID가 정의되어 있습니다.

- `tb3_leader`: TurtleBot leader
- `rc_car_follower`: RC카 follower, `/rc_car/odom` 기반 위치만 사용

즉 GUI는 TurtleBot과 RC카를 모두 map 위에 표시할 수 있습니다.

## 미션 예시

### A 구역 이동

```text
loading
-> intersection_1
-> a_entry
-> a_leader_slot
-> ArUco 정밀 접근
-> A_mission_start 발행
-> 로봇팔 작업
```

### B 구역 이동

```text
loading
-> intersection_2
-> b_entry
-> b_leader_slot
-> ArUco 정밀 접근
-> slot hold
```

### 충전소 이동

```text
GUI에서 charger_front 선택 또는 start_charger_parking 명령
-> /mission_command
-> charger_entry까지 NavigateToPose
-> L-shaped charger parking
-> charger_front 정렬
```

### RC카 추종

```text
TurtleBot 주행 시작
-> mission_loop가 /rc_car/follower_mode = follow 발행
-> rc_car_follower_node가 /turtlebot/pose history 저장
-> RC카가 leader 과거 궤적의 목표점을 선택
-> encoder odom + PWM 제어로 뒤따라감
-> 작업 slot 근처에서는 stop/align 모드로 전환
```

## 기술적 특징 정리

### 1. Graph 기반 미션 주행

물류 환경을 임의 좌표가 아니라 semantic node로 관리합니다. 이 방식은 작업 구역, 교차점, 충전소처럼 의미가 있는 장소를 코드 레벨에서 직접 다룰 수 있게 합니다.

### 2. 직접 제어 + Nav2 보조 구조

기본 주행은 `/cmd_vel` 직접 제어입니다. 그러나 장애물과 충전소 같은 복잡한 상황에서는 Nav2 MPPI, NavigateToPose를 선택적으로 사용합니다. 따라서 커스텀 미션 FSM과 Nav2 장점을 같이 사용합니다.

### 3. Local A*와 Pure Pursuit 결합

A*는 local detour path를 만들고, Pure Pursuit는 그 path를 부드럽게 따라갑니다. 지도 기반 정적 장애물과 LiDAR 기반 동적 장애물을 함께 반영합니다.

### 4. MPPI Rescue

Local A*가 blocked 상태에 빠지면 MPPI가 짧은 rescue path를 최적화합니다. MPPI는 cost 기반 sampling controller이므로 복잡한 장애물 구간에서 후보 궤적을 평가할 수 있습니다.

### 5. 과거 궤적 기반 추종

RC카는 leader 현재 위치가 아니라 leader가 지나간 history를 따라갑니다. 이 방식은 코너와 정지 지점에서 follower가 leader 경로를 안정적으로 재현하도록 합니다.

### 6. Encoder Dead Reckoning

RC카는 좌우 encoder tick으로 이동 거리와 yaw를 추정합니다. 절대 위치 센서가 없기 때문에 초기 pose 설정과 tick-to-meter calibration이 중요합니다.

### 7. ArUco 정밀 접근

Topology node로 큰 위치까지 이동한 뒤, 마지막 접근은 카메라 기반 ArUco marker offset과 거리 추정으로 보정합니다.

### 8. 비전 기반 로봇팔

색상 segmentation, image center error feedback, planar IK를 조합해 물체 접근과 grasp sequence를 자동화합니다.

### 9. GUI 관제

React dashboard는 지도, 로봇 pose, RC카 pose, 미션 경로, 장애물 scan, 로그, 로봇팔 상태를 하나의 화면에서 보여줍니다. WebSocket 기반이므로 PC와 모바일 PWA가 같은 상태를 공유할 수 있습니다.

### 10. Backend 신뢰성 설계

Go backend는 idempotency key, audit log, mission cache, SQLite repository를 갖습니다. bridge가 잠시 끊겨도 GUI는 미션 의도와 route를 유지합니다.

## 실행 개요

ROS2 workspace에서 빌드합니다.

```bash
cd ~/Topology_Graph_test-main
colcon build --symlink-install
source install/setup.bash
```

기본 주행 관련 launch 예시입니다.

```bash
ros2 launch amr_topology turtlebot_mission.launch.py
ros2 launch amr_topology mppi_controller.launch.py
ros2 launch amr_topology arm_vision_pick.launch.py
```

GUI 시스템은 `gui_system`에서 backend, frontend, bridge를 실행합니다.

```bash
cd gui_system
./scripts/build_frontend.sh
./scripts/run_backend.sh
./scripts/run_bridge.sh
```

ROS domain은 bridge 설정 기준 `ROS_DOMAIN_ID=27`입니다...
