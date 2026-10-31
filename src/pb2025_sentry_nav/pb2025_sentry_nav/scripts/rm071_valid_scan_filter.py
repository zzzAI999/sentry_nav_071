#!/usr/bin/env python3
"""Forward only LaserScan messages with enough usable ranges for SLAM startup."""

from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ValidScanFilter(Node):
    def __init__(self) -> None:
        super().__init__("rm071_valid_scan_filter")

        self.declare_parameter("raw_scan_topic", "obstacle_scan")
        self.declare_parameter("valid_scan_topic", "/rm071/valid_obstacle_scan")
        self.declare_parameter("min_valid_ranges", 80)
        self.declare_parameter("min_consecutive_valid_scans", 3)
        self.declare_parameter("warn_every_invalid", 20)

        self.raw_scan_topic = str(self.get_parameter("raw_scan_topic").value)
        self.valid_scan_topic = str(self.get_parameter("valid_scan_topic").value)
        self.min_valid_ranges = int(self.get_parameter("min_valid_ranges").value)
        self.min_consecutive_valid_scans = max(
            1, int(self.get_parameter("min_consecutive_valid_scans").value)
        )
        self.warn_every_invalid = max(1, int(self.get_parameter("warn_every_invalid").value))

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(LaserScan, self.valid_scan_topic, scan_qos)
        self.subscription = self.create_subscription(
            LaserScan, self.raw_scan_topic, self._scan_callback, scan_qos
        )
        self.invalid_count = 0
        self.valid_count = 0
        self.consecutive_valid_count = 0

        self.get_logger().info(
            f"valid scan filter forwarding {self.raw_scan_topic} -> {self.valid_scan_topic}"
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        valid_ranges = 0
        for value in msg.ranges:
            if not math.isfinite(value):
                continue
            if msg.range_min <= value <= msg.range_max:
                valid_ranges += 1

        if valid_ranges < self.min_valid_ranges:
            self.consecutive_valid_count = 0
            self.invalid_count += 1
            if self.invalid_count == 1 or self.invalid_count % self.warn_every_invalid == 0:
                self.get_logger().warn(
                    "dropping invalid scan #%d: valid_ranges=%d required=%d total=%d"
                    % (self.invalid_count, valid_ranges, self.min_valid_ranges, len(msg.ranges))
                )
            return

        self.consecutive_valid_count += 1
        if self.consecutive_valid_count < self.min_consecutive_valid_scans:
            if self.consecutive_valid_count == 1:
                self.get_logger().info(
                    "waiting for stable scan stream: valid_ranges=%d required=%d consecutive=%d/%d"
                    % (
                        valid_ranges,
                        self.min_valid_ranges,
                        self.consecutive_valid_count,
                        self.min_consecutive_valid_scans,
                    )
                )
            return

        self.publisher.publish(msg)
        self.valid_count += 1
        if self.valid_count == 1:
            self.get_logger().info(
                "first stable valid scan forwarded: valid_ranges=%d total=%d consecutive_required=%d"
                % (valid_ranges, len(msg.ranges), self.min_consecutive_valid_scans)
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ValidScanFilter()
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
