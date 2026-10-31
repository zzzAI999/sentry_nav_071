# Copyright 2025 Lihan Chen
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")

    launch_dir = os.path.join(bringup_dir, "launch")

    workspace_dir = os.path.abspath(os.path.join(bringup_dir, "..", "..", "..", ".."))

    livox_sdk_lib_dir = os.path.join(
        workspace_dir,
        "src",
        "pb2025_sentry_nav",
        "livox_ros_driver2",
        "Livox-SDK2",
        "lib",
        "amd64",
    )

    ld_library_entries = [
        "/usr/lib/x86_64-linux-gnu",
        "/lib/x86_64-linux-gnu",
        livox_sdk_lib_dir,
    ]

    current_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")

    if current_ld_library_path:
        ld_library_entries.extend(
            entry for entry in current_ld_library_path.split(":") if entry
        )

    livox_runtime_env_cmd = SetEnvironmentVariable(
        "LD_LIBRARY_PATH",
        ":".join(dict.fromkeys(ld_library_entries)),
    )

    fastdds_udp_transport_env_cmd = SetEnvironmentVariable(
        "FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"
    )

    def true_expr(config):
        return ["'", config, "'.lower() in ['true', '1', 'yes', 'on']"]

    def mode_expr(mode_value):
        if mode_value == "mapping":
            return [
                "('", mode, "' == 'mapping' or ('", mode, "' == 'slam_arg' and ",
                *true_expr(slam), "))",
            ]
        if mode_value == "navigation":
            return [
                "('", mode, "' == 'navigation' or ('", mode, "' == 'slam_arg' and not (",
                *true_expr(slam), ")))",
            ]
        return ["'", mode, "' == '", mode_value, "'"]

    def is_mode(mode_value):
        return PythonExpression(mode_expr(mode_value))

    def mode_and_true(mode_value, *configs):
        expression = mode_expr(mode_value)
        for config in configs:
            expression.extend(
                [" and '", config, "'.lower() in ['true', '1', 'yes', 'on']"]
            )
        return PythonExpression(expression)

    def mode_and_true_or_true(mode_value, mode_config, *configs):
        expression = ["(", *mode_expr(mode_value)]
        expression.extend(
            [" and '", mode_config, "'.lower() in ['true', '1', 'yes', 'on'])"]
        )
        for config in configs:
            expression.extend(
                [" or '", config, "'.lower() in ['true', '1', 'yes', 'on']"]
            )
        return PythonExpression(expression)

    def mode_and_true_any(*mode_configs):
        expression = []
        for index, mode_config in enumerate(mode_configs):
            mode_value, *configs = mode_config
            if index:
                expression.append(" or ")
            expression.append("(")
            expression.extend(mode_expr(mode_value))
            for config in configs:
                expression.extend(
                    [" and '", config, "'.lower() in ['true', '1', 'yes', 'on']"]
                )
            expression.append(")")
        return PythonExpression(expression)

    namespace = LaunchConfiguration("namespace")

    mode = LaunchConfiguration("mode")

    slam = LaunchConfiguration("slam")

    forward_only_enabled = LaunchConfiguration("forward_only_enabled")

    world = LaunchConfiguration("world")

    map_yaml_file = LaunchConfiguration("map")

    prior_pcd_file = LaunchConfiguration("prior_pcd_file")

    use_sim_time = LaunchConfiguration("use_sim_time")

    params_file = LaunchConfiguration("params_file")

    autostart = LaunchConfiguration("autostart")

    use_composition = LaunchConfiguration("use_composition")

    use_respawn = LaunchConfiguration("use_respawn")

    rviz_config_file = LaunchConfiguration("rviz_config_file")

    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")

    mapping_use_robot_state_pub = LaunchConfiguration("mapping_use_robot_state_pub")

    mapping_bringup_delay_sec = LaunchConfiguration("mapping_bringup_delay_sec")

    use_rviz = LaunchConfiguration("use_rviz")

    enable_pgm_initial_pose = LaunchConfiguration("enable_pgm_initial_pose")

    enable_auto_explore = LaunchConfiguration("enable_auto_explore")

    explore_params_file = LaunchConfiguration("explore_params_file")

    explore_start_paused = LaunchConfiguration("explore_start_paused")

    explore_start_delay_sec = LaunchConfiguration("explore_start_delay_sec")

    explore_auto_resume_on_map_ready = LaunchConfiguration("explore_auto_resume_on_map_ready")

    explore_map_ready_min_free_cells = LaunchConfiguration("explore_map_ready_min_free_cells")

    explore_map_ready_min_unknown_cells = LaunchConfiguration("explore_map_ready_min_unknown_cells")

    explore_resume_topic = LaunchConfiguration("explore_resume_topic")

    enable_mcu_status_nav = LaunchConfiguration("enable_mcu_status_nav")

    mapping_enable_mcu_serial = LaunchConfiguration("mapping_enable_mcu_serial")

    mcu_serial_port = LaunchConfiguration("mcu_serial_port")

    mcu_serial_baudrate = LaunchConfiguration("mcu_serial_baudrate")

    mcu_serial_read_mode = LaunchConfiguration("mcu_serial_read_mode")

    mcu_line_numeric_mode = LaunchConfiguration("mcu_line_numeric_mode")

    mcu_cmd_send_rate_hz = LaunchConfiguration("mcu_cmd_send_rate_hz")

    mcu_status_topic = LaunchConfiguration("mcu_status_topic")

    nav_command_topic = LaunchConfiguration("nav_command_topic")

    declare_mode_cmd = DeclareLaunchArgument(
        "mode",
        default_value="slam_arg",
        description=(
            "Runtime mode: mapping only starts SLAM/explore; navigation starts "
            "localization/Nav2. Default slam_arg keeps "
            "backward compatibility with slam:=True/False."
        ),
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Whether run a SLAM. If True, it will disable small_gicp and send static tf (map->odom)",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="rmul_2024",
        description="Select world: 'rmul_2024' or 'rmuc_2024' (map file share the same name as the this parameter)",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=[
            TextSubstitution(text=os.path.join(bringup_dir, "map", "reality", "")),
            world,
            TextSubstitution(text=".yaml"),
        ],
        description="Full path to map file to load",
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file",
        default_value=[
            TextSubstitution(text=os.path.join(bringup_dir, "pcd", "reality", "")),
            world,
            TextSubstitution(text=".pcd"),
        ],
        description="Full path to prior pcd file to load",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            bringup_dir, "config", "reality", "nav2_params.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="False",
        description="Whether to start the robot state publisher",
    )

    declare_mapping_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "mapping_use_robot_state_pub",
        default_value="True",
        description=(
            "Whether mapping mode should start robot_state_publisher to provide static "
            "base/lidar TF when no external robot bringup is running"
        ),
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_mapping_bringup_delay_sec_cmd = DeclareLaunchArgument(
        "mapping_bringup_delay_sec",
        default_value="3.0",
        description="Delay mapping bringup so the Livox driver can enter normal publishing before Point-LIO/SLAM starts",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    declare_forward_only_enabled_cmd = DeclareLaunchArgument(
        "forward_only_enabled", default_value="False",
        description="Optional forward-only driving; disabled for the omnidirectional chassis",
    )

    declare_enable_pgm_initial_pose_cmd = DeclareLaunchArgument(
        "enable_pgm_initial_pose", default_value="True",
        description="Run the PGM initial-pose helper once after navigation starts",
    )

    declare_enable_auto_explore_cmd = DeclareLaunchArgument(
        "enable_auto_explore",
        default_value="True",
        description="Whether to start frontier-based autonomous exploration node in mapping mode",
    )

    declare_explore_params_file_cmd = DeclareLaunchArgument(
        "explore_params_file",
        default_value=os.path.join(bringup_dir, "config", "common", "explore_params.yaml"),
        description="Full path to the explore_lite params file",
    )

    declare_explore_start_paused_cmd = DeclareLaunchArgument(
        "explore_start_paused",
        default_value="True",
        description=(
            "Start explore node paused. In mapping mode the map-ready starter "
            "resumes it automatically when /map is valid."
        ),
    )

    declare_explore_start_delay_sec_cmd = DeclareLaunchArgument(
        "explore_start_delay_sec",
        default_value="6.0",
        description=(
            "Delay before starting explore_lite in mapping mode, giving SLAM, "
            "TF and costmaps time to publish a valid initial map"
        ),
    )

    declare_explore_auto_resume_on_map_ready_cmd = DeclareLaunchArgument(
        "explore_auto_resume_on_map_ready",
        default_value="True",
        description="Automatically publish explore/resume once /map has enough free and unknown cells",
    )

    declare_explore_map_ready_min_free_cells_cmd = DeclareLaunchArgument(
        "explore_map_ready_min_free_cells",
        default_value="20",
        description="Minimum free cells required before auto-resuming explore",
    )

    declare_explore_map_ready_min_unknown_cells_cmd = DeclareLaunchArgument(
        "explore_map_ready_min_unknown_cells",
        default_value="20",
        description="Minimum unknown cells required before auto-resuming explore",
    )

    declare_explore_resume_topic_cmd = DeclareLaunchArgument(
        "explore_resume_topic",
        default_value="explore/resume",
        description="Topic used to resume autonomous exploration",
    )

    declare_enable_mcu_status_nav_cmd = DeclareLaunchArgument(
        "enable_mcu_status_nav",
        default_value="True",
        description="Whether navigation mode starts the MCU serial velocity bridge",
    )

    declare_mapping_enable_mcu_serial_cmd = DeclareLaunchArgument(
        "mapping_enable_mcu_serial",
        default_value="True",
        description=(
            "Whether mapping mode should start the MCU serial velocity bridge so "
            "autonomous exploration cmd_vel can drive the real chassis"
        ),
    )

    declare_mcu_serial_port_cmd = DeclareLaunchArgument(
        "mcu_serial_port",
        default_value="/dev/serial_port",
        description="MCU serial device path",
    )

    declare_mcu_serial_baudrate_cmd = DeclareLaunchArgument(
        "mcu_serial_baudrate",
        default_value="115200",
        description="MCU serial baudrate",
    )

    declare_mcu_serial_read_mode_cmd = DeclareLaunchArgument(
        "mcu_serial_read_mode",
        default_value="line",
        description="MCU serial read mode: line or binary",
    )

    declare_mcu_line_numeric_mode_cmd = DeclareLaunchArgument(
        "mcu_line_numeric_mode",
        default_value="command",
        description="How line-mode numeric text is handled: command, status, or both",
    )

    declare_mcu_cmd_send_rate_hz_cmd = DeclareLaunchArgument(
        "mcu_cmd_send_rate_hz",
        default_value="50.0",
        description="MCU serial velocity command send rate in Hz",
    )

    declare_mcu_status_topic_cmd = DeclareLaunchArgument(
        "mcu_status_topic",
        default_value="/mcu/status",
        description="Status topic published by serial receiver",
    )

    declare_nav_command_topic_cmd = DeclareLaunchArgument(
        "nav_command_topic",
        default_value="/app/nav_command",
        description="Optional local command topic published by the MCU serial bridge",
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "robot_state_publisher_launch.py")
        ),
        # NOTE: This startup file is only used when the navigation module is standalone
        condition=IfCondition(
            mode_and_true_or_true(
                "mapping", mapping_use_robot_state_pub, use_robot_state_pub
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    start_livox_ros_driver2_node = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_ros_driver2",
        output="screen",
        namespace=namespace,
        parameters=[configured_params],
    )

    start_valid_map_filter_node = Node(
        package="pb2025_sentry_nav",
        executable="rm071_valid_map_filter.py",
        name="rm071_valid_map_filter",
        output="screen",
        namespace=namespace,
        parameters=[
            {
                "raw_map_topic": "/map",
                "valid_map_topic": "/rm071/valid_map",
                "min_known_cells": 1,
                "min_free_cells": ParameterValue(
                    PythonExpression([
                        "int(", explore_map_ready_min_free_cells, ") if ",
                        *mode_expr("mapping"),
                        " else 1",
                    ]),
                    value_type=int,
                ),
                "min_unknown_cells": ParameterValue(
                    PythonExpression([
                        "int(", explore_map_ready_min_unknown_cells, ") if ",
                        *mode_expr("mapping"),
                        " else 0",
                    ]),
                    value_type=int,
                ),
                "warn_every_invalid": 20,
            }
        ],
    )

    start_valid_scan_filter_node = Node(
        condition=IfCondition(is_mode("mapping")),
        package="pb2025_sentry_nav",
        executable="rm071_valid_scan_filter.py",
        name="rm071_valid_scan_filter",
        output="screen",
        namespace=namespace,
        parameters=[
            {
                "raw_scan_topic": "obstacle_scan",
                "valid_scan_topic": "/rm071/valid_obstacle_scan",
                "min_valid_ranges": 80,
                "min_consecutive_valid_scans": 3,
                "warn_every_invalid": 20,
            }
        ],
    )

    start_map_recorder_node = Node(
        condition=IfCondition(is_mode("mapping")),
        package="pb2025_sentry_nav",
        executable="rm071_map_recorder.py",
        name="rm071_map_recorder",
        output="screen",
        namespace=namespace,
        sigterm_timeout="60.0",
        parameters=[{"map_topic": "/map", "map_yaml_file": ParameterValue(map_yaml_file, value_type=str)}],
    )

    initial_pose_cmd = TimerAction(
        condition=IfCondition(mode_and_true("navigation", enable_pgm_initial_pose)),
        period=6.0,
        actions=[Node(
            package="pb2025_sentry_nav",
            executable="rm071_pgm_initial_pose.py",
            name="rm071_pgm_initial_pose",
            output="screen",
            namespace=namespace,
            parameters=[{"map_yaml": ParameterValue(map_yaml_file, value_type=str)}],
        )],
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "bringup_launch.py")),
        condition=IfCondition(is_mode("navigation")),
        launch_arguments={
            "namespace": namespace,
            "slam": "False",
            "map": map_yaml_file,
            "prior_pcd_file": prior_pcd_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "forward_only_enabled": forward_only_enabled,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
        }.items(),
    )

    mapping_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "bringup_launch.py")),
        launch_arguments={
            "namespace": namespace,
            "slam": "True",
            "map": map_yaml_file,
            "prior_pcd_file": prior_pcd_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "forward_only_enabled": forward_only_enabled,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
        }.items(),
    )

    delayed_mapping_bringup_cmd = TimerAction(
        condition=IfCondition(is_mode("mapping")),
        period=mapping_bringup_delay_sec,
        actions=[mapping_bringup_cmd],
    )

    explore_node_with_custom_params = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        namespace=namespace,
        output="screen",
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        parameters=[
            explore_params_file,
            {"use_sim_time": use_sim_time, "start_paused": explore_start_paused},
        ],
    )

    delayed_explore_node_cmd = TimerAction(
        condition=IfCondition(mode_and_true("mapping", enable_auto_explore)),
        period=explore_start_delay_sec,
        actions=[explore_node_with_custom_params],
    )

    start_explore_map_ready_starter_node = Node(
        condition=IfCondition(
            mode_and_true(
                "mapping", enable_auto_explore, explore_auto_resume_on_map_ready
            )
        ),
        package="pb2025_sentry_nav",
        executable="rm071_explore_map_ready_starter.py",
        name="rm071_explore_map_ready_starter",
        output="screen",
        namespace=namespace,
        parameters=[
            {
                "map_topic": "/rm071/valid_map",
                "resume_topic": explore_resume_topic,
                "startup_delay_sec": explore_start_delay_sec,
                "min_free_cells": explore_map_ready_min_free_cells,
                "min_unknown_cells": explore_map_ready_min_unknown_cells,
            }
        ],
    )

    start_mcu_serial_receiver_node = Node(
        condition=IfCondition(
            mode_and_true_any(
                ("mapping", mapping_enable_mcu_serial),
                ("navigation", enable_mcu_status_nav),
            )
        ),
        package="pb2025_sentry_nav",
        executable="rm071_mcu_serial_receiver.py",
        name="rm071_mcu_serial_receiver",
        output="screen",
        namespace=namespace,
        parameters=[
            {
                "port": mcu_serial_port,
                "baudrate": mcu_serial_baudrate,
                "status_topic": mcu_status_topic,
                "command_topic": nav_command_topic,
                "cmd_vel_topic": "/cmd_vel",
                "read_mode": mcu_serial_read_mode,
                "line_numeric_mode": mcu_line_numeric_mode,
                "cmd_send_rate_hz": mcu_cmd_send_rate_hz,
            }
        ],
    )

    ld = LaunchDescription()

    ld.add_action(livox_runtime_env_cmd)

    ld.add_action(fastdds_udp_transport_env_cmd)

    ld.add_action(declare_mode_cmd)

    ld.add_action(declare_namespace_cmd)

    ld.add_action(declare_slam_cmd)

    ld.add_action(declare_world_cmd)

    ld.add_action(declare_map_yaml_cmd)

    ld.add_action(declare_prior_pcd_file_cmd)

    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(declare_params_file_cmd)

    ld.add_action(declare_autostart_cmd)

    ld.add_action(declare_use_composition_cmd)

    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(declare_use_robot_state_pub_cmd)

    ld.add_action(declare_mapping_use_robot_state_pub_cmd)

    ld.add_action(declare_mapping_bringup_delay_sec_cmd)

    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(declare_forward_only_enabled_cmd)

    ld.add_action(declare_enable_pgm_initial_pose_cmd)

    ld.add_action(declare_enable_auto_explore_cmd)

    ld.add_action(declare_explore_params_file_cmd)

    ld.add_action(declare_explore_start_paused_cmd)

    ld.add_action(declare_explore_start_delay_sec_cmd)

    ld.add_action(declare_explore_auto_resume_on_map_ready_cmd)

    ld.add_action(declare_explore_map_ready_min_free_cells_cmd)

    ld.add_action(declare_explore_map_ready_min_unknown_cells_cmd)

    ld.add_action(declare_explore_resume_topic_cmd)

    ld.add_action(declare_use_respawn_cmd)

    ld.add_action(declare_enable_mcu_status_nav_cmd)

    ld.add_action(declare_mapping_enable_mcu_serial_cmd)

    ld.add_action(declare_mcu_serial_port_cmd)

    ld.add_action(declare_mcu_serial_baudrate_cmd)

    ld.add_action(declare_mcu_serial_read_mode_cmd)

    ld.add_action(declare_mcu_line_numeric_mode_cmd)

    ld.add_action(declare_mcu_cmd_send_rate_hz_cmd)

    ld.add_action(declare_mcu_status_topic_cmd)

    ld.add_action(declare_nav_command_topic_cmd)

    ld.add_action(start_robot_state_publisher_cmd)

    ld.add_action(start_livox_ros_driver2_node)

    ld.add_action(start_valid_map_filter_node)

    ld.add_action(start_valid_scan_filter_node)

    ld.add_action(start_map_recorder_node)

    ld.add_action(bringup_cmd)

    ld.add_action(initial_pose_cmd)

    ld.add_action(delayed_mapping_bringup_cmd)

    ld.add_action(delayed_explore_node_cmd)

    ld.add_action(start_explore_map_ready_starter_node)

    ld.add_action(start_mcu_serial_receiver_node)

    ld.add_action(rviz_cmd)

    return ld
