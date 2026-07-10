from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mission_params = {
        "base_frame": "base_footprint",
        "enable_lidar_safety": True,
        "scan_topic": "/scan",
        "map_topic": "/map",
        "leader_pose_topic": "/turtlebot/pose",
        "rc_car_mode_topic": "/rc_car/follower_mode",
        "rc_car_slot_status_topic": "/rc_car/slot_wait_status",
        "repeat_count": 1,
        "precision_goal_tolerance": 0.04,
        "enable_a_slot_aruco_follow": True,
        "rc_intersection2_stop_x": 2.547372817993164,
        "rc_intersection2_stop_y": -1.605921983718872,
        "rc_intersection2_stop_tolerance": 0.12,
        "aruco_enable_topic": "/aruco_marker/enable",
        "aruco_target_topic": "/aruco_marker/target",
        "aruco_search_angular_speed": 0.18,
        "aruco_approach_linear_speed": 0.045,
        "aruco_approach_angular_gain": 0.40,
        "aruco_stop_distance": 0.30,
        "aruco_detection_timeout_seconds": 0.6,
        "post_aruco_turtle_target_x": 1.6706253290176392,
        "post_aruco_turtle_target_y": -0.020037246868014336,
        "post_aruco_turtle_target_yaw": 0.0,
        "obstacle_avoid_linear_speed": 0.075,
        "obstacle_avoid_angular_speed": 0.22,
        "obstacle_stop_and_scan_seconds": 2.0,
        "obstacle_emergency_stop_distance": 0.22,
        "obstacle_side_stop_distance": 0.20,
        "obstacle_target_stop_distance": 0.42,
        "obstacle_rotate_in_place_heading_threshold": 0.35,
        "obstacle_rotate_in_place_resume_threshold": 0.18,
        "obstacle_rotate_in_place_max_heading": 1.05,
        "enable_mppi_rescue": True,
        "mppi_follow_path_action": "follow_path",
        "mppi_rescue_timeout_seconds": 18.0,
        "mppi_rescue_stop_and_plan_seconds": 2.0,
        "mppi_rescue_min_handoff_seconds": 5.0,
        "mppi_rescue_handoff_front_clearance": 0.48,
        "mppi_rescue_handoff_side_clearance": 0.30,
        "mppi_rescue_slow_handoff_seconds": 8.0,
        "mppi_rescue_slow_handoff_front_clearance": 0.38,
        "mppi_rescue_side_escape_seconds": 3.0,
        "enable_corridor_pass": True,
        "corridor_robot_width": 0.306,
        "corridor_side_clearance": 0.07,
        "corridor_min_passage_width": 0.45,
        "corridor_hard_stop_width": 0.40,
        "corridor_max_lateral_offset": 0.24,
    }

    return LaunchDescription(
        [
            Node(
                package="amr_topology",
                executable="mission_loop",
                name="mission_loop",
                output="screen",
                parameters=[mission_params],
            ),
        ]
    )
