#!/usr/bin/env python3
"""Start simulated Nav2 after the user places the virtual robot on the map."""

import time

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


class SimulationStartup(Node):
    def __init__(self):
        super().__init__("rm071_sim_startup")
        self.ready = False
        self.ready_at = float("-inf")
        self.future = None
        self.finished = False
        self.failed = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = self.create_client(
            ManageLifecycleNodes, "lifecycle_manager_navigation/manage_nodes")
        self.subscription = self.create_subscription(
            Bool, "localization_ready", self.on_ready,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer = self.create_timer(0.2, self.tick)
        self.get_logger().info(
            "Map loaded first; waiting for RViz 2D Pose Estimate before starting Nav2")

    def on_ready(self, message):
        self.ready = message.data
        self.ready_at = time.monotonic()

    def tick(self):
        if self.finished:
            return
        if self.future is not None:
            if not self.future.done():
                return
            try:
                response = self.future.result()
                if response is None or not response.success:
                    raise RuntimeError("navigation lifecycle startup was rejected")
            except Exception as exc:
                self.failed = True
                self.get_logger().error(f"Simulation Nav2 startup failed: {exc}")
            else:
                self.get_logger().info("Simulation Nav2 active; navigation goals can now run")
            self.finished = True
            return
        if not self.ready or time.monotonic() - self.ready_at > 2.0:
            return
        try:
            transform = self.tf_buffer.lookup_transform("map", "gimbal_yaw_fake", Time())
        except TransformException:
            return
        stamp = transform.header.stamp
        age = (self.get_clock().now().nanoseconds / 1e9
               - stamp.sec - stamp.nanosec / 1e9)
        if not -0.2 <= age <= 1.0 or not self.client.service_is_ready():
            return
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        self.future = self.client.call_async(request)
        self.get_logger().info("Initial pose and TF ready; starting simulation navigation")


def main(args=None):
    rclpy.init(args=args)
    node = SimulationStartup()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
        return 1 if node.failed else 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
