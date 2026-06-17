# Topology Graph Test

ROS 2 Humble 기반 TurtleBot3 `waffle_pi` 토폴로지 주행 및 RC카 추종 주행 테스트 패키지입니다.

이 저장소의 중심 목표는 TurtleBot이 `map` 좌표계에서 토폴로지 노드를 따라 주행하고, RC카가 TurtleBot의 현재 위치가 아니라 TurtleBot이 지나간 과거 궤적을 뒤에서 따라가도록 만드는 것입니다. TurtleBot 주행은 `/cmd_vel`을 직접 발행하는 자체 제어 노드가 담당하고, RC카는 Raspberry Pi의 ROS2 노드가 좌우 PWM을 계산해 Arduino Uno에 전달하는 구조입니다.

## 전체 구조

```text
topology.yaml
-> mission_loop
-> TurtleBot /cmd_vel
-> TurtleBot pose publish: /turtlebot/pose

/turtlebot/pose
-> rc_car_follower_node
-> leader history 저장
-> RC카 목표점 선택
-> RC카 encoder odom 계산
-> left_pwm,right_pwm 계산
-> Arduino serial command
-> RC카 모터 PWM 적용
```

주요 구성은 다음과 같습니다.

- `mission_loop`
  - TurtleBot의 전체 미션 순서와 토폴로지 주행을 담당합니다.
  - TurtleBot 현재 위치를 `map` 기준 `/turtlebot/pose`로 발행합니다.
  - RC카 follower 모드를 `/rc_car/follower_mode`로 발행합니다.
  - 라이다 기반 안전 정지, A* detour, MPPI rescue, corridor pass 판단을 포함합니다.
- `rc_car_follower_node`
  - TurtleBot의 과거 pose history를 저장합니다.
  - RC카의 초기 odom과 encoder 기반 odom을 관리합니다.
  - RC카와 과거 궤적 목표점 사이의 거리/방향 오차를 계산합니다.
  - Arduino에 `left_pwm,right_pwm` 형식의 실시간 PWM 명령을 보냅니다.
  - Arduino가 출력한 누적 encoder tick을 파싱해 `/rc_car/odom`을 갱신합니다.
- `topology_follower`
  - 지정된 route만 단독으로 따라가는 단순 토폴로지 follower입니다.
  - 전체 mission loop보다 작은 범위의 경로 테스트에 사용되는 구조입니다.
- `LocalAStarPurePursuit`
  - 라이다와 map을 기반으로 local detour path를 만들고 Pure Pursuit로 따라갑니다.
  - 장애물 회피, side escape, local blocked 판단을 제공합니다.

## 디렉터리 구성

```text
src/amr_topology/
  CMakeLists.txt
  package.xml
  config/
    topology.yaml
    amcl.yaml
    mppi_controller.yaml
  include/amr_topology/
    local_astar_pure_pursuit.hpp
  launch/
    map_server.launch.py
    localization_rviz.launch.py
    mppi_controller.launch.py
  maps/
    map.yaml
    map.pgm
  rviz/
    amcl_localization.rviz
  src/
    mission_loop.cpp
    rc_car_follower_node.cpp
    topology_follower.cpp
    local_astar_pure_pursuit.cpp
```

## 토폴로지 설정

`src/amr_topology/config/topology.yaml`은 TurtleBot이 이동할 주요 지점을 `map` 좌표계 기준으로 정의합니다.

노드에는 `loading`, `intersection`, `entry`, `slot`, `standby`, `waypoint` 같은 type이 붙고, `mission_loop`는 이 type을 보고 경유 지점인지 작업 지점인지 판단합니다.

대표적인 노드 역할은 다음과 같습니다.

- `loading`
  - 미션 출발 지점입니다.
- `intersection_1`, `intersection_2`
  - 경유 교차점입니다.
  - 막혀 있을 경우 다음 목표 방향이 안전하면 스킵할 수 있습니다.
- `a_entry`, `b_entry`
  - A/B 작업 구역 진입 지점입니다.
- `a_leader_slot`, `b_leader_slot`
  - TurtleBot이 정지하거나 정밀 접근하는 작업 지점입니다.
- `charger_entry`
  - 충전 또는 도킹 진입 관련 지점입니다.

`topology.yaml`의 좌표는 TurtleBot의 목표 지점 정의입니다. RC카가 자기 위치를 알게 하려면 RC카 시작 좌표가 `/rc_car/odom`의 초기값으로 들어가야 하며, 단순히 `topology.yaml`에 좌표를 적는 것만으로 RC카 odom이 초기화되지는 않습니다.

## TurtleBot 미션 노드

`src/amr_topology/src/mission_loop.cpp`의 `MissionLoop` 클래스가 전체 TurtleBot 주행을 담당합니다.

주요 역할은 다음과 같습니다.

- topology graph 로딩
- 현재 TurtleBot pose 추정
- 목표 노드까지의 거리와 heading error 계산
- `/cmd_vel` 직접 발행
- `/turtlebot/pose` 발행
- `/rc_car/follower_mode` 발행
- 반복 미션 상태 관리
- 작업 지점 대기 및 재개 판단
- 라이다 안전 회피와 local planner 호출
- MPPI rescue handoff 판단
- corridor pass 가능 여부 판단

주요 입출력은 다음과 같습니다.

- 입력
  - TF `map -> base_footprint`
  - `/scan`
  - `/map`
  - `/mission_command`
- 출력
  - `/cmd_vel`
  - `/turtlebot/pose`
  - `/rc_car/follower_mode`

`MissionLoop`는 Nav2의 전체 navigation pipeline을 주행 기본값으로 쓰지 않습니다. 기본 주행은 목표 노드 방향을 직접 계산해서 `/cmd_vel`을 발행합니다. 다만 좁은 통로 또는 local planner가 막힌 상황에서는 MPPI controller의 `FollowPath` action을 rescue 용도로 사용할 수 있게 되어 있습니다.

## TurtleBot 주행 판단 흐름

TurtleBot의 일반 주행 흐름은 다음과 같습니다.

```text
현재 pose 확인
-> 목표 topology node 선택
-> 목표까지 거리 계산
-> 목표 방향 heading 계산
-> heading error 계산
-> angular.z 보정
-> linear.x 전진
-> 목표 tolerance 안으로 들어오면 다음 node로 전환
```

장애물이 있을 때는 다음 계층으로 판단합니다.

```text
라이다 emergency 거리 확인
-> target 방향 장애물 확인
-> 측면 근접 확인
-> Local A* + Pure Pursuit detour 시도
-> side_escape 판단
-> corridor pass 가능 폭 계산
-> MPPI rescue handoff
-> 작업 지점이면 정지 후 대기
-> 경유 지점이면 조건부 스킵
```

## Local A* + Pure Pursuit

`src/amr_topology/include/amr_topology/local_astar_pure_pursuit.hpp`와 `src/amr_topology/src/local_astar_pure_pursuit.cpp`는 TurtleBot의 local obstacle avoidance를 담당합니다.

핵심 상태는 다음과 같습니다.

- `clear`
  - 장애물 없이 일반 주행이 가능한 상태입니다.
- `stop_and_plan`
  - 목표 방향에 장애물이 있어 local detour path를 계획하는 상태입니다.
- `follow_detour`
  - A*로 만든 detour path를 Pure Pursuit 방식으로 따라가는 상태입니다.
- `side_escape`
  - 좌우 측면이 너무 가까울 때 반대 방향으로 벗어나는 상태입니다.
- `blocked`
  - local planner만으로 통과가 어렵다고 판단한 상태입니다.

local planner는 `/scan`과 `/map`을 이용해 로봇 주변의 계획 grid를 만들고, 목표 방향으로 갈 수 있는 우회 경로를 찾습니다. 찾은 path는 Pure Pursuit로 추종하며, 일정 시간 이상 막히면 `mission_loop`가 MPPI rescue 또는 정지 대기로 넘깁니다.

## Corridor Pass

`mission_loop.cpp`에는 좁은 벽-장애물 사이를 통과할 수 있는지 판단하는 corridor pass 로직이 있습니다.

판단 기준은 라이다에서 얻은 좌우 여유 거리입니다.

```text
corridor width = left_min + right_min
```

주요 파라미터는 다음과 같은 의미를 가집니다.

- `corridor_robot_width`
  - TurtleBot 실제 폭입니다.
- `corridor_side_clearance`
  - 양옆 안전 여유입니다.
- `corridor_min_passage_width`
  - 통과 가능하다고 보는 최소 폭입니다.
- `corridor_hard_stop_width`
  - 이보다 좁으면 즉시 정지해야 하는 폭입니다.
- `corridor_max_lateral_offset`
  - 통로 중심에서 벗어난 정도의 허용값입니다.

통과 가능하다고 판단되면 통로 중심을 향하는 path를 만들어 MPPI controller에 넘깁니다. 통과 불가능하면 `/cmd_vel`을 정지로 유지합니다.

## MPPI Rescue

MPPI는 기본 주행기가 아니라 rescue 수단으로 사용됩니다.

`mission_loop`가 local planner의 blocked 상태, stop-and-plan 지속 시간, side escape 지속 시간을 보고 MPPI handoff를 결정합니다. handoff가 되면 `mppi_controller.launch.py`로 실행되는 Nav2 `controller_server`의 `FollowPath` action에 짧은 path를 전달합니다.

이 구조의 목적은 다음과 같습니다.

- 평소에는 topology 기반 직접 제어를 유지합니다.
- 복잡한 장애물 구간에서만 MPPI의 local trajectory optimization을 사용합니다.
- costmap inflation 때문에 지나갈 수 있는 통로를 포기하는 상황을 줄이기 위해 corridor pass 판단을 먼저 적용합니다.

## RC카 Follower 노드

`src/amr_topology/src/rc_car_follower_node.cpp`의 `RcCarFollowerNode` 클래스가 RC카 추종 주행을 담당합니다.

RC카 follower의 기본 방향은 TurtleBot 현재 위치를 바로 따라가는 것이 아니라, TurtleBot이 지나간 과거 궤적을 뒤에서 따라가는 것입니다.

주요 입력은 다음과 같습니다.

- `/turtlebot/pose`
  - TurtleBot의 `map` 기준 현재 pose입니다.
  - follower는 이 pose를 시간순으로 history에 저장합니다.
- `/cmd_vel`
  - TurtleBot 속도 추정 보조 입력입니다.
- `/rc_car/follower_mode`
  - `stop`, `follow`, `return`, `align`, `turn_prepare_left` 같은 follower 모드를 받습니다.
- Arduino serial
  - 누적 encoder tick 문자열을 받습니다.

주요 출력은 다음과 같습니다.

- `/rc_car/odom`
  - RC카의 `map` 기준 추정 odom입니다.
- `/rc_car/command`
  - Arduino에 보낸 PWM 명령을 ROS topic으로도 발행합니다.
- `/rc_car/encoder_ticks`
  - Arduino에서 받은 좌우 누적 tick을 ROS topic으로 발행합니다.
- `/rc_car/serial_debug`
  - Arduino serial raw line 확인용 debug topic입니다.
- Arduino serial command
  - 실제 모터 제어 명령입니다.

## RC카 초기 Odom

RC카는 절대 위치 센서가 없기 때문에 시작할 때의 `map` 기준 위치를 시스템에 알려줘야 합니다.

관련 파라미터는 다음과 같습니다.

- `auto_initialize_odom_from_leader`
  - `true`이면 TurtleBot pose를 기준으로 RC카 초기 위치를 추정합니다.
  - `false`이면 `initial_rc_x`, `initial_rc_y`, `initial_rc_yaw`를 직접 사용합니다.
- `initial_rc_x`
  - RC카 시작 x 좌표입니다.
- `initial_rc_y`
  - RC카 시작 y 좌표입니다.
- `initial_rc_yaw`
  - RC카 시작 yaw입니다.

`auto_initialize_odom_from_leader=false`일 때 follower는 시작 즉시 다음 값을 설정합니다.

```text
odom_pose_.x = initial_rc_x
odom_pose_.y = initial_rc_y
odom_pose_.yaw = initial_rc_yaw
follower_pose_ = odom_pose_
odom_initialized_ = true
```

이후 `/rc_car/odom`은 이 초기 위치에서 encoder delta를 누적해 갱신됩니다. 시작 좌표가 실제 RC카 위치와 다르면 이후 odom 전체가 그만큼 틀어진 상태로 누적됩니다.

## RC카 Encoder Odom

Arduino는 좌우 encoder tick의 누적값을 계속 출력합니다. follower 노드는 이전 tick과 현재 tick의 차이를 계산해 RC카 이동량을 구합니다.

```text
delta_left_ticks = current_left - previous_left
delta_right_ticks = current_right - previous_right
delta_left = delta_left_ticks * left_meters_per_tick
delta_right = delta_right_ticks * right_meters_per_tick
delta_distance = (delta_left + delta_right) / 2
delta_yaw = (delta_right - delta_left) / wheel_base
```

그 결과로 `odom_pose_`의 x, y, yaw가 갱신됩니다.

중요한 기준은 다음과 같습니다.

- encoder tick은 주행 중 리셋하지 않습니다.
- Arduino는 누적 tick만 출력합니다.
- ROS2 follower가 delta tick을 계산합니다.
- 전진 시 `/rc_car/odom`이 실제 전진 방향으로 증가해야 합니다.
- 실제 차는 앞으로 가는데 odom만 뒤로 가면 tick 부호 또는 `meters_per_tick` 부호가 틀린 것입니다.
- 양수 PWM에서 실제 차가 뒤로 가면 Arduino 모터 방향 기준이 틀린 것입니다.

## RC카 과거 궤적 추종

`rc_car_follower_node`는 `/turtlebot/pose`를 받을 때마다 `leader_history_`에 저장합니다.

history는 다음 기준으로 관리됩니다.

- `leader_history_max_seconds`
  - history를 유지할 최대 시간입니다.
- `leader_history_min_spacing`
  - 너무 촘촘한 pose 저장을 막기 위한 최소 간격입니다.
- `leader_path_follow_distance`
  - RC카가 따라갈 TurtleBot 과거 궤적의 뒤쪽 거리입니다.

추종 목표점 선택 흐름은 다음과 같습니다.

```text
TurtleBot pose history 저장
-> 현재 RC카 위치 확인
-> TurtleBot 현재 위치보다 뒤쪽의 과거 궤적 목표점 선택
-> RC카와 목표점 사이 거리 계산
-> RC카 heading과 목표 방향 차이 계산
-> left_pwm,right_pwm 계산
```

이 구조에서는 TurtleBot이 좌회전하자마자 RC카도 즉시 좌회전하는 것이 아닙니다. RC카는 TurtleBot이 실제로 지나간 궤적을 뒤에서 따라가다가, 같은 회전 구간에 도달했을 때 회전합니다.

## RC카 PWM 제어

RC카 follower는 Arduino에 짧은 이동 명령을 보내지 않습니다. 매 control cycle마다 좌우 PWM을 계산해 serial로 보냅니다.

명령 형식은 다음과 같습니다.

```text
left_pwm,right_pwm
```

예시는 다음과 같습니다.

```text
100,100
80,120
-90,90
0,0
```

PWM 기준은 다음과 같습니다.

- 양수 PWM
  - 해당 바퀴 전진
- 음수 PWM
  - 해당 바퀴 후진
- `0`
  - 해당 바퀴 정지
- 범위
  - `-255`부터 `255`까지

거리/방향 제어는 다음 값을 사용합니다.

- `target_distance`
  - TurtleBot과 유지하려는 목표 거리입니다.
- `too_close_distance`
  - 너무 가까운 거리로 판단하는 기준입니다.
- `emergency_stop_distance`
  - 이보다 가까우면 정지합니다.
- `distance_deadband`
  - 목표 거리 주변의 허용 범위입니다.
- `heading_deadband`
  - 방향 오차 허용 범위입니다.
- `slow_forward_pwm`
  - 가까운 거리에서 정지 대신 천천히 진행할 때 쓰는 PWM입니다.
- `base_forward_pwm`, `min_forward_pwm`, `max_forward_pwm`
  - 일반 전진 PWM 계산 범위입니다.
- `turn_pwm_delta`, `near_turn_pwm_delta`
  - 좌우 PWM 차이를 만들어 방향을 보정하는 값입니다.

현재 follower 로직은 일반적인 가까움 상황에서 바로 후진하지 않고, `emergency_stop_distance` 이하일 때만 정지하도록 설계되어 있습니다. 너무 가까운 구간이지만 emergency는 아니면 낮은 전진 PWM과 작은 방향 보정으로 거리 유지가 이어지도록 합니다.

## Arduino 역할

Arduino Uno는 저수준 모터 드라이버 역할만 담당합니다.

Arduino가 담당하는 일은 다음과 같습니다.

- serial로 `left_pwm,right_pwm` 명령 수신
- 좌우 모터 PWM 즉시 적용
- `S` 또는 `STOP` 명령 수신 시 정지
- 일정 시간 명령이 없으면 안전 정지
- 좌우 encoder tick 누적
- 누적 tick 주기 출력

Arduino가 담당하지 않는 일은 다음과 같습니다.

- TurtleBot 좌표 해석
- topology graph 해석
- 과거 궤적 저장
- 거리 오차 계산
- 방향 오차 계산
- 몇 cm 이동할지 결정
- 몇 도 회전할지 결정

Arduino encoder 출력 형식은 follower가 파싱할 수 있는 누적 tick 문자열이어야 합니다.

```text
L_Tick:123 R_Tick:128
```

또는 comma 형식도 follower 파서가 처리할 수 있습니다.

```text
123,128
```

## RC카 방향 기준

RC카 제어에서 가장 중요한 기준은 양수 PWM, encoder tick, odom 증가 방향이 서로 맞아야 한다는 점입니다.

정상 기준은 다음과 같습니다.

```text
100,100 명령
-> 실제 RC카 전진
-> L_Tick 증가
-> R_Tick 증가
-> /rc_car/odom이 실제 진행 방향으로 이동
```

만약 `100,100`에서 실제 RC카가 뒤로 움직이면 Arduino의 모터 방향 기준을 수정해야 합니다. 실제 RC카는 앞으로 움직이는데 tick이 음수로 감소하면 encoder interrupt 방향 또는 `left_meters_per_tick`, `right_meters_per_tick`의 부호를 수정해야 합니다.

## Intersection 1 연동

`mission_loop.cpp`는 TurtleBot이 `intersection_1`에 가까워질 때 RC카 follower에 `turn_prepare_left` 모드를 보낼 수 있습니다.

관련 파라미터는 다음과 같습니다.

- `rc_car_intersection_1_prepare_distance`
  - TurtleBot이 `intersection_1` 목표점에 이 거리 이하로 가까워졌을 때 RC카 회전 준비 모드를 요청합니다.

`rc_car_follower_node.cpp`는 `turn_prepare_left` 모드에서 RC카가 회전 구간에 들어가기 전 속도와 방향 보정을 별도로 적용할 수 있도록 구성되어 있습니다.

이 기능의 목적은 `intersection_1` 회전 구간에서 RC카가 너무 빠르게 벽 쪽으로 밀고 들어가지 않게 하는 것입니다. 다만 회전 성공 여부는 RC카 초기 x, y, yaw와 encoder odom이 실제 위치와 맞는지에 크게 의존합니다.

## 주요 ROS Topic

TurtleBot 관련 topic:

- `/cmd_vel`
  - TurtleBot 주행 명령입니다.
- `/turtlebot/pose`
  - TurtleBot의 `map` 기준 pose입니다.
- `/scan`
  - 라이다 입력입니다.
- `/map`
  - OccupancyGrid map입니다.
- `/amcl_pose`
  - AMCL localization 결과입니다.

RC카 관련 topic:

- `/rc_car/follower_mode`
  - RC카 follower 상태 명령입니다.
- `/rc_car/odom`
  - RC카 encoder 기반 odom입니다.
- `/rc_car/command`
  - follower가 Arduino에 보낸 PWM 명령입니다.
- `/rc_car/encoder_ticks`
  - Arduino에서 받은 좌우 누적 tick입니다.
- `/rc_car/serial_debug`
  - Arduino serial raw line 확인용 topic입니다.

## 주요 파일

`src/amr_topology/src/mission_loop.cpp`

- TurtleBot 전체 미션 루프입니다.
- topology node를 따라 `/cmd_vel`을 생성합니다.
- `/turtlebot/pose`를 발행합니다.
- RC카 follower mode를 발행합니다.
- 라이다 안전 회피, A* detour, MPPI rescue, corridor pass를 통합합니다.

`src/amr_topology/src/rc_car_follower_node.cpp`

- RC카 follower 핵심 노드입니다.
- TurtleBot 과거 궤적을 저장합니다.
- RC카 초기 odom과 encoder odom을 관리합니다.
- 실시간 좌우 PWM을 계산합니다.
- Arduino serial 입출력을 담당합니다.

`src/amr_topology/src/topology_follower.cpp`

- 지정된 route를 단독으로 따라가는 간단한 토폴로지 follower입니다.
- 전체 mission loop 없이 특정 구간만 확인할 때 사용할 수 있는 구조입니다.

`src/amr_topology/include/amr_topology/local_astar_pure_pursuit.hpp`

- local planner의 데이터 구조, 옵션, 상태, public API를 정의합니다.

`src/amr_topology/src/local_astar_pure_pursuit.cpp`

- 라이다 기반 장애물 판단, local A* planning, Pure Pursuit 추종, side escape 판단을 구현합니다.

`src/amr_topology/config/topology.yaml`

- TurtleBot 주행 노드 좌표와 type을 정의합니다.

`src/amr_topology/config/mppi_controller.yaml`

- MPPI controller server 설정입니다.

## 현재 개발 상태

현재 코드는 TurtleBot topology 주행과 RC카 follower의 기본 구조를 연결한 상태입니다.

구현된 주요 항목은 다음과 같습니다.

- TurtleBot topology mission loop
- TurtleBot pose 발행
- RC카 follower mode 발행
- RC카 초기 odom 파라미터
- RC카 encoder tick 파싱
- RC카 encoder 기반 `/rc_car/odom`
- TurtleBot pose history 저장
- 과거 궤적 기반 목표점 선택
- 거리/방향 오차 기반 left/right PWM 생성
- Arduino serial command 송신
- Arduino serial debug topic
- RC카 command topic
- RC카 encoder tick topic
- intersection_1 회전 준비 mode 연동
- TurtleBot local obstacle avoidance
- MPPI rescue handoff
- corridor pass 판단

남은 핵심 튜닝 항목은 다음과 같습니다.

- Arduino 양수 PWM 방향과 실제 전진 방향 일치
- encoder tick 증가 방향과 odom 진행 방향 일치
- `left_meters_per_tick`, `right_meters_per_tick` 실측 보정
- RC카 wheel base 보정
- RC카 초기 yaw 반복 세팅 기준 확정
- `target_distance`, `leader_path_follow_distance` 튜닝
- 회전 구간 PWM과 속도 튜닝
- intersection_1 저속 반복 테스트
