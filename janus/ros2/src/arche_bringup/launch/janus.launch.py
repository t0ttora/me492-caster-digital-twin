from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    sim = get_package_share_path("arche_sim")
    description = get_package_share_path("arche_description")
    gz_sim = get_package_share_path("ros_gz_sim")
    config = sim / "config"
    robot_description = xacro.process_file(str(description / "urdf/arche.urdf.xacro")).toxml()
    sim_time = {"use_sim_time": True}
    return LaunchDescription([
        DeclareLaunchArgument("gz_flags", default_value="-r -s --headless-rendering -v 3"),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(sim / "models")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gz_sim / "launch/gz_sim.launch.py")),
            launch_arguments={
                "gz_args": [LaunchConfiguration("gz_flags"), " ", str(sim / "worlds/janus.sdf")],
                "on_exit_shutdown": "true",
            }.items(),
        ),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="janus_bridge",
             parameters=[{"config_file": str(config / "bridge.yaml") }], output="screen"),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="janus_camera_bridge",
             parameters=[{"config_file": str(config / "camera_bridge.yaml"),
                          "override_frame_id": "camera_optical_frame"}], output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[sim_time, {"robot_description": robot_description}], output="screen"),
        Node(package="tf2_ros", executable="static_transform_publisher", name="janus_map_to_odom",
             arguments=["--frame-id", "map", "--child-frame-id", "odom"], parameters=[sim_time]),
        Node(package="arche_experiments", executable="command_guard", parameters=[sim_time],
             output="screen"),
        Node(package="arche_experiments", executable="qr_observer",
             parameters=[sim_time, {"target_u_px": 320.0, "target_id": "JANUS-DOCK-01"}],
             output="screen"),
    ])
