#!/usr/bin/env python3
"""Republish only valid OccupancyGrid maps for Nav2/explore startup stability."""

from __future__ import annotations

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class ValidMapFilter(Node):
    def __init__(self) -> None:
        super().__init__("rm071_valid_map_filter")

        self.declare_parameter("raw_map_topic", "/map")
        self.declare_parameter("valid_map_topic", "/rm071/valid_map")
        self.declare_parameter("min_known_cells", 1)
        self.declare_parameter("min_free_cells", 0)
        self.declare_parameter("min_unknown_cells", 0)
        self.declare_parameter("warn_every_invalid", 20)

        self.raw_map_topic = str(self.get_parameter("raw_map_topic").value)
        self.valid_map_topic = str(self.get_parameter("valid_map_topic").value)
        self.min_known_cells = int(self.get_parameter("min_known_cells").value)
        self.min_free_cells = int(self.get_parameter("min_free_cells").value)
        self.min_unknown_cells = int(self.get_parameter("min_unknown_cells").value)
        self.warn_every_invalid = max(1, int(self.get_parameter("warn_every_invalid").value))

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher = self.create_publisher(OccupancyGrid, self.valid_map_topic, map_qos)
        self.subscription = self.create_subscription(
            OccupancyGrid, self.raw_map_topic, self._map_callback, map_qos
        )
        self.invalid_count = 0
        self.valid_count = 0

        self.get_logger().info(
            f"valid map filter forwarding {self.raw_map_topic} -> {self.valid_map_topic}"
        )

    def _map_callback(self, msg: OccupancyGrid) -> None:
        width = int(msg.info.width)
        height = int(msg.info.height)
        expected = width * height
        data_len = len(msg.data)

        if width <= 0 or height <= 0 or data_len != expected:
            self._log_invalid(width, height, data_len, expected, "bad dimensions/data length")
            return

        known_cells = 0
        free_cells = 0
        unknown_cells = 0
        for cell in msg.data:
            if cell < -1 or cell > 100:
                self._log_invalid(width, height, data_len, expected, "cell value outside [-1, 100]")
                return
            if cell == -1:
                unknown_cells += 1
            else:
                known_cells += 1
                if cell == 0:
                    free_cells += 1

        if known_cells < self.min_known_cells:
            self._log_invalid(width, height, data_len, expected, "not enough known cells")
            return
        if free_cells < self.min_free_cells:
            self._log_invalid(width, height, data_len, expected, "not enough free cells")
            return
        if unknown_cells < self.min_unknown_cells:
            self._log_invalid(width, height, data_len, expected, "not enough unknown cells")
            return

        self.publisher.publish(msg)
        self.valid_count += 1
        if self.valid_count == 1:
            self.get_logger().info(
                "first valid map forwarded: width=%d, height=%d, known=%d, free=%d, unknown=%d"
                % (width, height, known_cells, free_cells, unknown_cells)
            )

    def _log_invalid(self, width: int, height: int, data_len: int, expected: int, reason: str) -> None:
        self.invalid_count += 1
        if self.invalid_count == 1 or self.invalid_count % self.warn_every_invalid == 0:
            self.get_logger().warn(
                "dropping invalid map #%d: width=%d height=%d data=%d expected=%d reason=%s"
                % (self.invalid_count, width, height, data_len, expected, reason)
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ValidMapFilter()
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
