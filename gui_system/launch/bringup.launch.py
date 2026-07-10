"""Full system bringup — extends gui.launch.py with the upstream ROS2
nodes (Nav2, robot drivers, mission manager) needed in the Should
stage. Until those packages are wired, this file just delegates to
gui.launch.py so `ros2 launch gui_system bringup.launch.py` is a valid
single entry point.
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    here = os.path.dirname(__file__)
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(here, "gui.launch.py")),
        ),
        # Should: add Nav2 / robot bringup / mission manager Include
        # statements here so a single ros2 launch starts everything.
    ])
