#!/usr/bin/env python3
"""Check the generated robot TF without starting the lidar or chassis."""

import math
import os
import signal
import subprocess
import time

import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def main():
    if "ROS_DOMAIN_ID" not in os.environ:
        raise RuntimeError("Set an isolated ROS_DOMAIN_ID before running this test")
    process = subprocess.Popen(
        ["ros2", "launch", "pb2025_nav_bringup", "robot_state_publisher_launch.py"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True,
    )
    rclpy.init()
    node = rclpy.create_node("rm071_test_lidar_tf")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    try:
        deadline = time.monotonic() + 30.0
        transform = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if process.poll() is not None:
                raise RuntimeError("robot description process exited early")
            try:
                transform = buffer.lookup_transform("base_footprint", "front_mid360", Time())
                break
            except TransformException:
                pass
        if transform is None:
            raise RuntimeError("base_footprint -> front_mid360 TF was not received")
        position = transform.transform.translation
        rotation = transform.transform.rotation
        expected_roll = 3.14
        if not all(abs(actual - expected) < 1e-5 for actual, expected in
                   ((position.x, -0.07), (position.y, 0.0), (position.z, 0.52))):
            raise RuntimeError(f"unexpected lidar translation: {position}")
        if (abs(abs(rotation.x) - abs(math.sin(expected_roll / 2))) > 1e-5
                or abs(abs(rotation.w) - abs(math.cos(expected_roll / 2))) > 1e-5
                or abs(rotation.y) > 1e-5 or abs(rotation.z) > 1e-5):
            raise RuntimeError(f"unexpected lidar orientation: {rotation}")
        print("TF base_footprint -> front_mid360: x=-0.07 y=0 z=0.52 roll=3.14 rad")
    finally:
        del listener
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate()
        if process.returncode not in (0, -signal.SIGINT):
            print(output)


if __name__ == "__main__":
    main()
