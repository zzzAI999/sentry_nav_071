#!/usr/bin/env python3
"""Check installed mapping dependencies without launching or moving the robot."""

import argparse
import importlib
import os
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET


EXECUTABLES = {
    "livox_ros_driver2": "livox_ros_driver2_node",
    "point_lio": "pointlio_mapping",
    "loam_interface": "loam_interface_node",
    "sensor_scan_generation": "sensor_scan_generation_node",
    "fake_vel_transform": "fake_vel_transform_node",
    "terrain_analysis": "terrainAnalysis",
    "terrain_analysis_ext": "terrainAnalysisExt",
    "pointcloud_to_laserscan": "pointcloud_to_laserscan_node",
    "slam_toolbox": "sync_slam_toolbox_node",
    "explore_lite": "explore",
    "robot_state_publisher": "robot_state_publisher",
    "joint_state_publisher": "joint_state_publisher",
    "rclcpp_components": "component_container_isolated",
    "tf2_ros": "static_transform_publisher",
    "nav2_map_server": "map_saver_server",
    "nav2_controller": "controller_server",
    "nav2_planner": "planner_server",
    "nav2_smoother": "smoother_server",
    "nav2_behaviors": "behavior_server",
    "nav2_bt_navigator": "bt_navigator",
    "nav2_waypoint_follower": "waypoint_follower",
    "nav2_velocity_smoother": "velocity_smoother",
    "nav2_lifecycle_manager": "lifecycle_manager",
    "pb2025_sentry_nav": "rm071_map_recorder.py",
}


def check(map_root):
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from sdformat_tools.urdf_generator import UrdfGenerator
    from xmacro.xmacro4sdf import XMLMacro4sdf

    for module in ("rclpy", "numpy", "cv2", "yaml", "serial", "nav2_common.launch"):
        importlib.import_module(module)
    for package, executable in EXECUTABLES.items():
        path = Path(get_package_prefix(package)) / "lib" / package / executable
        if not path.is_file() or not os.access(path, os.X_OK):
            raise RuntimeError(f"Missing executable: {path}; rebuild/install dependencies and source install/setup.bash")
    bringup = Path(get_package_share_directory("pb2025_nav_bringup"))
    for name in ("config/reality/nav2_params.yaml", "config/reality/mid360_user_config.json",
                 "config/common/explore_params.yaml"):
        if not (bringup / name).is_file():
            raise RuntimeError(f"Missing configuration: {bringup / name}")
    macro = XMLMacro4sdf()
    macro.set_xml_file(str(Path(get_package_share_directory("pb2025_robot_description")) /
                           "resource/xmacro/pb2025_sentry_robot.sdf.xmacro"))
    macro.generate()
    generator = UrdfGenerator()
    generator.parse_from_sdf_string(macro.to_string())
    model = ET.fromstring(generator.to_string())
    links = {link.get("name") for link in model.findall("link")}
    for frame in ("base_footprint", "gimbal_yaw", "front_mid360"):
        if frame not in links:
            raise RuntimeError(f"Generated robot description has no {frame} link")
    map_root = Path(map_root).expanduser()
    map_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".mapping-check-", dir=map_root) as directory:
        path = Path(directory) / "write-check"
        path.write_bytes(b"mapping check\n")
        os.replace(path, path.with_suffix(".ok"))
    print("PASS: mapping executables, Python dependencies, robot model and map output directory.")
    print("Next: ./start_mapping.sh; inspect live LiDAR/IMU, TF and the expanding map in RViz.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-root", required=True)
    options = parser.parse_args()
    try:
        check(options.map_root)
    except Exception as exc:
        print(f"Mapping check failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
