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
