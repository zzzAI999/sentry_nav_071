#!/usr/bin/env python3
"""Cache the current occupancy grid and save it locally on graceful shutdown."""

import math
import os
from pathlib import Path
import signal
import sys
import tempfile

import numpy as np
import yaml


def usable_map(message):
    width, height = int(message.info.width), int(message.info.height)
    if width <= 0 or height <= 0 or len(message.data) != width * height:
        return False
    if not math.isfinite(float(message.info.resolution)) or message.info.resolution <= 0:
        return False
    cells = np.asarray(message.data, dtype=np.int16)
    return bool(np.all((cells >= -1) & (cells <= 100)) and np.any((cells >= 0) & (cells <= 25)))


def atomic_write(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=".map-", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def save_occupancy_grid(message, yaml_path):
    if not usable_map(message):
        raise ValueError("no usable occupancy grid")
    yaml_path = Path(yaml_path).expanduser().resolve()
    if yaml_path.suffix.lower() not in (".yaml", ".yml"):
        raise ValueError("map path must end in .yaml or .yml")
    pgm_path = yaml_path.with_suffix(".pgm")
    width, height = int(message.info.width), int(message.info.height)
    cells = np.asarray(message.data, dtype=np.int16).reshape(height, width)
    # Keep unknown cells between the free and occupied thresholds in map_server.
    image = np.full((height, width), 127, dtype=np.uint8)
    image[(cells >= 0) & (cells <= 25)] = 254
    image[cells >= 65] = 0

    origin = message.info.origin
    q = origin.orientation
    values = (origin.position.x, origin.position.y, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("map origin contains non-finite values")
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1e-8:
        raise ValueError("map origin orientation is invalid")
    x, y, z, w = (q.x / norm, q.y / norm, q.z / norm, q.w / norm)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    metadata = {
        "image": pgm_path.name,
        "mode": "trinary",
        "resolution": float(message.info.resolution),
        "origin": [float(origin.position.x), float(origin.position.y), yaw],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    atomic_write(pgm_path, header + image[::-1].tobytes())
    atomic_write(yaml_path, yaml.safe_dump(metadata, sort_keys=False).encode("utf-8"))
    return pgm_path, yaml_path


def main():
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions

    class MapRecorder(Node):
        def __init__(self):
            super().__init__("rm071_map_recorder")
            self.declare_parameter("map_topic", "/map")
            self.declare_parameter("map_yaml_file", "")
            self.yaml_path = str(self.get_parameter("map_yaml_file").value)
            if not self.yaml_path:
                raise ValueError("map_yaml_file is required")
            self.latest = None
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.subscription = self.create_subscription(
                OccupancyGrid, str(self.get_parameter("map_topic").value), self.on_map, qos)
            self.get_logger().info(f"Caching map for shutdown: {self.yaml_path}")

        def on_map(self, message):
            if usable_map(message):
                self.latest = message

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = MapRecorder()
    stopping = False

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    result = 1
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node.latest is None:
                raise RuntimeError("no usable map received; previous map is unchanged")
            pgm_path, yaml_path = save_occupancy_grid(node.latest, node.yaml_path)
            print(f"Saved occupancy map: {pgm_path}, {yaml_path}", flush=True)
            result = 0
        except Exception as exc:
            print(f"Map save failed: {exc}", file=sys.stderr, flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == "__main__":
    sys.exit(main())
