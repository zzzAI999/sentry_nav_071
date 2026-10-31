#!/usr/bin/env python3

import time
from collections import Counter

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class ExploreMapReadyStarter(Node):
    def __init__(self) -> None:
        super().__init__("rm071_explore_map_ready_starter")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("resume_topic", "explore/resume")
        self.declare_parameter("startup_delay_sec", 5.0)
        self.declare_parameter("min_free_cells", 20)
        self.declare_parameter("min_unknown_cells", 20)
        self.declare_parameter("required_consecutive_maps", 3)
        self.declare_parameter("publish_count", 1)
        self.declare_parameter("publish_period_sec", 0.25)

        self.map_topic = str(self.get_parameter("map_topic").value)
        self.resume_topic = str(self.get_parameter("resume_topic").value)
        self.startup_delay_sec = float(self.get_parameter("startup_delay_sec").value)
        self.min_free_cells = int(self.get_parameter("min_free_cells").value)
        self.min_unknown_cells = int(self.get_parameter("min_unknown_cells").value)
        self.required_consecutive_maps = max(
            1, int(self.get_parameter("required_consecutive_maps").value)
        )
        self.publish_count = max(1, int(self.get_parameter("publish_count").value))
        self.publish_period_sec = max(
            0.05, float(self.get_parameter("publish_period_sec").value)
        )

        self.start_time = time.monotonic()
        self.ready_count = 0
        self.resume_started = False
        self.resume_sent = 0
        self.last_wait_log = 0.0
        self.latest_map_ready = False

        self.resume_pub = self.create_publisher(Bool, self.resume_topic, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self._map_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.publish_timer = self.create_timer(
            self.publish_period_sec, self._publish_resume_if_ready
        )

        self.get_logger().info(
            "map-ready explore starter waiting: "
            f"map={self.map_topic}, resume={self.resume_topic}, "
            f"min_free={self.min_free_cells}, min_unknown={self.min_unknown_cells}"
        )

    def _map_callback(self, msg: OccupancyGrid) -> None:
        if self.resume_started:
            return
        expected = int(msg.info.width) * int(msg.info.height)
        if msg.info.width == 0 or msg.info.height == 0 or len(msg.data) != expected:
            self.ready_count = 0
            self.latest_map_ready = False
            self._log_waiting(
                f"invalid map width={msg.info.width} height={msg.info.height} "
                f"data={len(msg.data)} expected={expected}"
            )
            return

        counts = Counter(msg.data)
        free_cells = counts.get(0, 0)
        unknown_cells = counts.get(-1, 0)
        if free_cells < self.min_free_cells or unknown_cells < self.min_unknown_cells:
            self.ready_count = 0
            self.latest_map_ready = False
            self._log_waiting(
                f"map not ready: free={free_cells}, unknown={unknown_cells}, "
                f"required free>={self.min_free_cells}, unknown>={self.min_unknown_cells}"
            )
            return

        self.latest_map_ready = True

    def _publish_resume_if_ready(self) -> None:
        # A retained valid map is readiness state; it need not be republished
        # three times after this helper's startup delay.
        if not self.resume_started:
            if (time.monotonic() - self.start_time < self.startup_delay_sec
                    or not self.latest_map_ready):
                return
            self.ready_count += 1
            if self.ready_count < self.required_consecutive_maps:
                return
            self.resume_started = True
            self.get_logger().info("cached map readiness is stable; resuming explore")
        if not self.resume_started or self.resume_sent >= self.publish_count:
            return
        if self.resume_pub.get_subscription_count() <= 0:
            self._log_waiting("map ready, waiting for explore/resume subscriber")
            return

        msg = Bool()
        msg.data = True
        self.resume_pub.publish(msg)
        self.resume_sent += 1
        if self.resume_sent == 1:
            self.get_logger().info("published explore resume command")
        if self.resume_sent >= self.publish_count:
            self.get_logger().info("explore resume command completed; starter will stay idle")

    def _log_waiting(self, text: str) -> None:
        now = time.monotonic()
        if now - self.last_wait_log >= 2.0:
            self.get_logger().info(text)
            self.last_wait_log = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExploreMapReadyStarter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
