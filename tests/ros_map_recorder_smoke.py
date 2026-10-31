#!/usr/bin/env python3
"""Publish a synthetic map and verify graceful recorder shutdown in an isolated ROS domain."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import cv2
import rclpy
from ament_index_python.packages import get_package_prefix
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
import yaml


def main():
    if "ROS_DOMAIN_ID" not in os.environ:
        raise RuntimeError("Set an isolated ROS_DOMAIN_ID before running this test")
    executable = Path(get_package_prefix("pb2025_sentry_nav")) / "lib/pb2025_sentry_nav/rm071_map_recorder.py"
    with tempfile.TemporaryDirectory(prefix="rm071_map_smoke_") as directory:
        path = Path(directory) / "map.yaml"
        rclpy.init()
        node = rclpy.create_node("rm071_test_map_publisher")
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = node.create_publisher(OccupancyGrid, "/map", qos)
        process = subprocess.Popen([str(executable), "--ros-args", "-p", f"map_yaml_file:={path}"],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                   start_new_session=True)
        try:
            deadline = time.monotonic() + 20.0
            while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if process.poll() is not None:
                    raise RuntimeError("recorder exited before subscribing")
            if publisher.get_subscription_count() == 0:
                raise RuntimeError("recorder did not subscribe to /map")
            message = OccupancyGrid()
            message.header.frame_id = "map"
            message.info.width = 8
            message.info.height = 8
            message.info.resolution = 0.05
            message.info.origin.orientation.w = 1.0
            message.data = [100, -1] + [0] * 62
            publisher.publish(message)
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.1)
            process.send_signal(signal.SIGINT)
            output, _ = process.communicate(timeout=20)
            if process.returncode != 0:
                raise RuntimeError(f"recorder exited {process.returncode}: {output}")
            metadata = yaml.safe_load(path.read_text(encoding="utf-8"))
            image = cv2.imread(str(path.parent / metadata["image"]), cv2.IMREAD_GRAYSCALE)
            if image is None or image.shape != (8, 8) or int(image[-1, 0]) != 0:
                raise RuntimeError("saved occupancy map is invalid")
            received = []
            subscription = node.create_subscription(
                OccupancyGrid, "/rm071_test_reloaded_map", received.append, qos)
            server_executable = Path(get_package_prefix("nav2_map_server")) / "lib/nav2_map_server/map_server"
            server = subprocess.Popen(
                [str(server_executable), "--ros-args", "-r", "__node:=rm071_test_map_server",
                 "-r", "map:=/rm071_test_reloaded_map", "-p", f"yaml_filename:={path}"],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            try:
                client = node.create_client(ChangeState, "/rm071_test_map_server/change_state")
                if not client.wait_for_service(timeout_sec=15.0):
                    raise RuntimeError("map server lifecycle service unavailable")
                for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
                    request = ChangeState.Request()
                    request.transition.id = transition
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
                    if not future.done() or not future.result().success:
                        raise RuntimeError(f"map server transition {transition} failed")
                deadline = time.monotonic() + 15.0
                while not received and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                if not received or list(received[-1].data) != list(message.data):
                    raise RuntimeError("Nav2 map server did not preserve occupied, unknown and free cells")
                print("ROS recorder saved PGM/YAML and Nav2 preserved all cells on reload")
            finally:
                node.destroy_subscription(subscription)
                if server.poll() is None:
                    server.terminate()
                try:
                    server.communicate(timeout=10)
                except subprocess.TimeoutExpired:
                    server.kill()
                    server.communicate()
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
