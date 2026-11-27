"""Launch the baseline JANUS ROS nodes with a mission-local Gazebo world."""

from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    sim = get_package_share_path("arche_sim")
    description = get_package_share_path("arche_description")
    gz = get_package_share_path("ros_gz_sim")
    config = sim / "config"
    robot_description = xacro.process_file(str(description / "urdf/arche.urdf.xacro")).toxml()
    sim_time = {"use_sim_time": True}
    return LaunchDescription([
        DeclareLaunchArgument("world_sdf"),
        DeclareLaunchArgument("bridge"),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(sim / "models")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gz / "launch/gz_sim.launch.py")),
            launch_arguments={
                "gz_args": ["-r -s --headless-rendering -v 4 --seed 42 --physics-engine gz-physics-dartsim-plugin ", LaunchConfiguration("world_sdf")],
                "on_exit_shutdown": "true",
            }.items(),
        ),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="janus_bridge",
             parameters=[{"config_file": LaunchConfiguration("bridge")}], output="screen"),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="janus_camera_bridge",
             parameters=[{"config_file": str(config / "camera_bridge.yaml"),
                          "override_frame_id": "camera_optical_frame"}], output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[sim_time, {"robot_description": robot_description}], output="screen"),
        Node(package="arche_experiments", executable="command_guard", parameters=[sim_time], output="screen"),
        ExecuteProcess(cmd=[sys.executable, str(Path(__file__).with_name("lab_qr_observer.py")),
                            "--ros-args", "-p", "use_sim_time:=true"], output="screen"),
    ])
