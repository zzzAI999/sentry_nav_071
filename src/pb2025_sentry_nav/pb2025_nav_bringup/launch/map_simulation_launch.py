# Copyright 2026 sentry_nav_071 contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Run the real Nav2 planner/controller against a saved map and a loopback base."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")
    map_file = LaunchConfiguration("map")
    use_rviz = LaunchConfiguration("use_rviz")
    enable_route = LaunchConfiguration("enable_two_point_route")
    params_file = LaunchConfiguration("params_file")
    route_x1 = LaunchConfiguration("route_x1")
    route_y1 = LaunchConfiguration("route_y1")
    route_x2 = LaunchConfiguration("route_x2")
    route_y2 = LaunchConfiguration("route_y2")
    declarations = [
        DeclareLaunchArgument("map", description="Saved map YAML to simulate"),
        DeclareLaunchArgument("use_rviz", default_value="True"),
        DeclareLaunchArgument("enable_two_point_route", default_value="False"),
        DeclareLaunchArgument("route_x1", default_value=""),
        DeclareLaunchArgument("route_y1", default_value=""),
        DeclareLaunchArgument("route_x2", default_value=""),
        DeclareLaunchArgument("route_y2", default_value=""),
        DeclareLaunchArgument("params_file", default_value=os.path.join(
            bringup_dir, "config/reality/nav2_params.yaml")),
    ]
    # Use wall time throughout; the loopback process does not depend on /clock.
    real_params = ParameterFile(RewrittenYaml(
        source_file=params_file, param_rewrites={"use_sim_time": "False"},
        convert_types=True), allow_substs=True)
    sim_params = os.path.join(bringup_dir, "config/simulation/nav2_overlay.yaml")
    parameters = [real_params, sim_params, {"use_sim_time": False}]

    nodes = [
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             output="screen", parameters=parameters + [{
                 "yaml_filename": ParameterValue(map_file, value_type=str)}]),
        Node(package="nav2_loopback_sim", executable="loopback_simulator", name="loopback_simulator",
             output="screen", parameters=parameters,
             remappings=[("cmd_vel", "sim/cmd_vel"), ("odom", "odometry"), ("scan", "obstacle_scan")]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="sim_scan_frame",
             output="screen", arguments=["--x", "-0.07", "--y", "0", "--z", "0.52",
                                         "--roll", "0", "--pitch", "0", "--yaw", "0",
                                         "--frame-id", "base_footprint", "--child-frame-id", "sim_scan"]),
        Node(package="fake_vel_transform", executable="fake_vel_transform_node", name="fake_vel_transform",
             output="screen", parameters=parameters),
    ]
    server_specs = [
        ("nav2_controller", "controller_server", [("cmd_vel", "cmd_vel_controller")]),
        ("nav2_smoother", "smoother_server", []),
        ("nav2_planner", "planner_server", []),
        ("nav2_behaviors", "behavior_server", [("cmd_vel", "cmd_vel_controller")]),
        ("nav2_bt_navigator", "bt_navigator", [("cmd_vel", "cmd_vel_controller")]),
        ("nav2_waypoint_follower", "waypoint_follower", []),
        ("nav2_velocity_smoother", "velocity_smoother",
         [("cmd_vel", "cmd_vel_controller"), ("cmd_vel_smoothed", "cmd_vel_nav2_result")]),
    ]
    for package, executable, remappings in server_specs:
        nodes.append(Node(package=package, executable=executable, name=executable,
                          output="screen", parameters=parameters, remappings=remappings))
    nodes.extend([
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_simulation", output="screen", parameters=[{
                 "use_sim_time": False, "autostart": True,
                 "node_names": ["map_server"] + [item[1] for item in server_specs]}]),
        Node(package="pb2025_sentry_nav", executable="rm071_two_point_route.py",
             name="rm071_two_point_route", condition=IfCondition(enable_route),
             output="screen", parameters=[{"use_sim_time": False}],
             arguments=["--", route_x1, route_y1, route_x2, route_y2]),
        Node(package="rviz2", executable="rviz2", condition=IfCondition(use_rviz),
             output="screen", arguments=["-d", os.path.join(bringup_dir, "rviz/nav2_map_simulation.rviz")]),
    ])
    robot = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        bringup_dir, "launch/robot_state_publisher_launch.py")),
        launch_arguments={"namespace": "", "use_sim_time": "False"}.items())
    return LaunchDescription(declarations + [robot] + nodes)
