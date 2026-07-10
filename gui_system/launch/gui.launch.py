"""ROS2 launch file for the GUI stack.

Launches the Go backend (gui_main) and the Python ROS2 bridge in the
same process group so a single Ctrl+C tears both down. The bridge needs
`ROS_DOMAIN_ID=27` to see the TurtleBot3 fleet; the launch file injects
it automatically so operators don't have to remember to export it.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


# Default ROS_DOMAIN_ID for the project — keep this in sync with the
# value baked into bridge/config.yaml. Both the bridge and any tooling
# that calls ros2 CLI commands from this launch share it.
DEFAULT_DOMAIN_ID = "27"


def generate_launch_description():
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    default_config = os.path.join(repo, "backend", "configs", "config.yaml")
    backend_binary = os.path.join(repo, "backend", "bin", "gui_main")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Path to backend/configs/config.yaml",
    )
    domain_arg = DeclareLaunchArgument(
        "domain_id",
        default_value=os.environ.get("ROS_DOMAIN_ID", DEFAULT_DOMAIN_ID),
        description="ROS_DOMAIN_ID exported to the bridge process",
    )

    # We inherit + override env so any user-set ROS_DOMAIN_ID can still
    # take precedence via the launch argument.
    env = {**os.environ, "ROS_DOMAIN_ID": LaunchConfiguration("domain_id").perform(None)}  # type: ignore

    backend = ExecuteProcess(
        cmd=[backend_binary, "--config", LaunchConfiguration("config")],
        name="gui_main",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        cwd=os.path.join(repo, "backend"),
    )

    bridge = ExecuteProcess(
        cmd=["python3", "-m", "ros2_bridge", "--config", LaunchConfiguration("config")],
        name="gui_bridge",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        cwd=os.path.join(repo, "bridge"),
        additional_env={"ROS_DOMAIN_ID": LaunchConfiguration("domain_id")},
    )

    return LaunchDescription([config_arg, domain_arg, backend, bridge])
