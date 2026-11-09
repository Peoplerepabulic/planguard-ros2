"""Gazebo Harmonic + robot + bridge + cruise controller + safety shield.

ros2 launch planguard_description sim.launch.py                       # headless, CBF shield
ros2 launch planguard_description sim.launch.py gui:=true rviz:=true  # with Gazebo GUI and RViz
ros2 launch planguard_description sim.launch.py shield_mode:=off world:=/abs/path/world.sdf
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _setup(context, *args, **kwargs):
    desc = get_package_share_directory("planguard_description")
    ctrl = get_package_share_directory("planguard_control")
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731

    world = lc("world") or os.path.join(desc, "worlds", "arena.sdf")
    gui = lc("gui").lower() == "true"
    gz_args = f"-r {world}" if gui else f"-r -s --headless-rendering {world}"
    x, y, yaw = float(lc("x")), float(lc("y")), float(lc("yaw"))

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items(),
    )

    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(desc, "urdf", "robot.urdf.xacro")]), value_type=str)
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               parameters=[{"robot_description": robot_description, "use_sim_time": True}],
               output="log")

    spawn = Node(package="ros_gz_sim", executable="create", output="log",
                 arguments=["-topic", "robot_description", "-name", "planguard",
                            "-x", str(x), "-y", str(y), "-z", "0.005", "-Y", str(yaw)])

    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="log",
                  parameters=[{"config_file": os.path.join(desc, "config", "bridge.yaml"),
                               "use_sim_time": True}])

    params = os.path.join(ctrl, "config", "control.yaml")
    controller = Node(package="planguard_control", executable="cruise_controller", output="screen",
                      parameters=[params, {"use_sim_time": True,
                                           "goal_x": float(lc("goal_x")), "goal_y": float(lc("goal_y")),
                                           "start_x": x, "start_y": y, "start_yaw": yaw}])
    shield_over = {"use_sim_time": True, "mode": lc("shield_mode")}
    if lc("cbf_alpha"):
        shield_over["cbf.alpha"] = float(lc("cbf_alpha"))  # sensitivity runs only
    shield = Node(package="planguard_control", executable="safety_shield", output="screen",
                  parameters=[params, shield_over])

    actions = [gz, rsp, spawn, bridge, controller, shield]
    if lc("rviz").lower() == "true":
        actions.append(Node(package="rviz2", executable="rviz2", output="log",
                            arguments=["-d", os.path.join(desc, "rviz", "planguard.rviz")],
                            parameters=[{"use_sim_time": True}]))
    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument("world", default_value="", description="SDF world path (default: arena.sdf)"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("shield_mode", default_value="cbf", description="off | brake | cbf"),
        DeclareLaunchArgument("x", default_value="-3.0"),
        DeclareLaunchArgument("y", default_value="-3.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("goal_x", default_value="3.0"),
        DeclareLaunchArgument("goal_y", default_value="3.0"),
        DeclareLaunchArgument("cbf_alpha", default_value="", description="override cbf.alpha (optional)"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
