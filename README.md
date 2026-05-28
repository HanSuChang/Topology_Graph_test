# AMR Drive Arm Workspace

ROS 2 workspace for topology-graph based TurtleBot3 mission-loop testing.

## Package

- `amr_topology`: map server launch, topology YAML, waypoint mission loop, and reverse docking prototype.

## Build

```bash
colcon build --packages-select amr_topology
```

## Run Mission Loop

```bash
ros2 run amr_topology mission_loop --ros-args \
  -p base_frame:=base_footprint \
  -p enable_lidar_safety:=true \
  -p scan_topic:=/scan \
  -p rear_stop_distance:=0.18
```








L0: 상차/복귀 위치에서 시작하고 I1: 첫 번째 교차점로 갈 때 직진으로 쭉 가다가
I1: 첫 번째 교차점 쯤에 도착하면 제자리회전을 하는게 아니라 커브길 돌듯 살짝 회전하면서 A_in: A공간 입구 쪽 방향으로 직진을 한다.

직진 하면서 A_slot_leader: 터틀봇 정차 위치 (A-A지점)에 도착하면 무조건 거기에 3초동안 멈추고 제자리 회전 후 A_in: A공간 입구쪽으로 직진한다. A_in: A공간 입구에서 제자리 회전해서 B_in: B공간 입구로 가는게 아니라 
 A_in: A공간 입구에서 회전하면서 돌고 B_in: B공간 입구로 간다.또 커브길 돌듯이 회전후 B_slot_leader: 터틀봇 정차 위치 (B-A지점)로 간다. 마찬가지로 3초 동안 멈추고 제자리 회전 후 B_in: B공간 입구로 간다. B_in: B공간 입구에서 A_in: A공간 입구로 갈때 제자리회전이 아닌 회전하면서 간다. 커브길 돌듯이



제자리 회전은 A_slot_leader: 터틀봇 정차 위치 (A-A지점), B_slot_leader: 터틀봇 정차 위치 (B-A지점)
이 두 곳만 하는거다.


















