#!/usr/bin/env python3
"""Visit two map coordinates once, using Nav2 for all motion."""

import argparse
import math
from pathlib import Path
import signal
import sys
import time


FIRST_RADIUS = 0.50
FINAL_RADIUS = 0.20


def finite_float(value):
    number = float(value)
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError("coordinates must be finite")
    return number


def positive_float(value):
    number = finite_float(value)
    if number <= 0.0:
        raise argparse.ArgumentTypeError("value must be positive")
    return number


def parse_arguments(arguments):
    parser = argparse.ArgumentParser(description=(
        "Visit (x1, y1), switch within 0.5 m, then stop within 0.2 m of (x2, y2). "
        "Coordinates use the map frame and metres. Start navigation first."))
    parser.add_argument("x1", type=finite_float)
    parser.add_argument("y1", type=finite_float)
    parser.add_argument("x2", type=finite_float)
    parser.add_argument("y2", type=finite_float)
    parser.add_argument("--goal-timeout", type=positive_float, default=300.0,
                        help="maximum seconds for each point (default: 300)")
    parser.add_argument("--wait-timeout", type=positive_float, default=30.0,
                        help="maximum seconds to wait for Nav2 and TF (default: 30)")
    return parser.parse_args(arguments)


def execute_route(navigator, points, goal_timeout):
    """Only advance after reaching the requested coordinate and ending its action."""
    for index, (target, radius) in enumerate(zip(points, (FIRST_RADIUS, FINAL_RADIUS)), 1):
        if navigator.stopping:
            raise InterruptedError("route interrupted")
        navigator.info(f"Point {index}/2: ({target[0]:.3f}, {target[1]:.3f}), radius={radius:.2f} m")
        navigator.start(target)
        deadline = navigator.now() + goal_timeout
        missing_pose_since = None
        try:
            while True:
                navigator.spin(0.05)
                if navigator.stopping:
                    raise InterruptedError("route interrupted")
                status = navigator.status()
                if status not in (None, "succeeded"):
                    raise RuntimeError(f"point {index} navigation ended with {status}")
                position = navigator.position()
                if position is None:
                    if missing_pose_since is None:
                        missing_pose_since = navigator.now()
                    if navigator.now() - missing_pose_since > 2.0:
                        raise RuntimeError("fresh map-to-robot TF unavailable for 2 seconds")
                else:
                    missing_pose_since = None
                    distance = math.hypot(position[0] - target[0], position[1] - target[1])
                    if distance <= radius:
                        # Wait for this action to end before issuing the next one.
                        navigator.cancel()
                        navigator.info(f"Point {index} reached: distance={distance:.3f} m")
                        break
                    if status == "succeeded":
                        raise RuntimeError(
                            f"Nav2 finished {distance:.3f} m from the requested point {index}; "
                            "the requested radius was not reached (check goal projection/localization)")
                if navigator.now() >= deadline:
                    raise TimeoutError(f"point {index} exceeded {goal_timeout:.0f} seconds")
        except BaseException:
            navigator.cancel()
            raise
    navigator.info("Two-point route complete. No further goals will be sent.")


def main(arguments=None):
    options = parse_arguments(sys.argv[1:] if arguments is None else arguments)
    import rclpy
    from action_msgs.msg import GoalStatus
    from ament_index_python.packages import get_package_share_directory
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    class Navigator(Node):
        def __init__(self):
            super().__init__("rm071_two_point_route")
            self.stopping = False
            self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.handle = None
            self.pending = None
            self.result = None
            self.tree = str(Path(get_package_share_directory("pb2025_nav_bringup")) /
                            "behavior_trees/navigate_two_point_leg.xml")
            if not Path(self.tree).is_file():
                raise RuntimeError("two-point behavior tree is missing; rebuild the workspace")

        def info(self, message):
            self.get_logger().info(message)

        def now(self):
            return time.monotonic()

        def spin(self, timeout):
            if not rclpy.ok():
                self.stopping = True
                raise InterruptedError("ROS context stopped")
            rclpy.spin_once(self, timeout_sec=timeout)

        def wait(self, future, timeout):
            deadline = self.now() + timeout
            while not future.done():
                if self.now() >= deadline:
                    raise TimeoutError("Nav2 action response timed out")
                self.spin(0.05)
            return future.result()

        def position(self):
            try:
                transform = self.tf_buffer.lookup_transform("map", "gimbal_yaw_fake", Time())
            except TransformException:
                return None
            stamp = transform.header.stamp
            stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
            position = transform.transform.translation
            if not (-0.2 <= age <= 1.0) or not all(
                    math.isfinite(value) for value in (position.x, position.y)):
                return None
            return position.x, position.y

        def wait_ready(self):
            self.info("Waiting for Nav2 and fresh map->gimbal_yaw_fake TF")
            deadline = self.now() + options.wait_timeout
            while not (self.client.server_is_ready() and self.position() is not None):
                if self.stopping:
                    raise InterruptedError("route interrupted")
                if self.now() >= deadline:
                    raise TimeoutError("navigation is not ready; start navigation and check localization")
                self.spin(0.1)

        def start(self, target):
            self.handle = None
            self.result = None
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = target
            goal.pose.pose.orientation.w = 1.0
            goal.behavior_tree = self.tree
            self.pending = self.client.send_goal_async(goal)
            # Resolve an in-flight request even after Ctrl+C so it can be canceled.
            self.handle = self.wait(self.pending, 10.0)
            self.pending = None
            if not self.handle.accepted:
                self.handle = None
                raise RuntimeError("Nav2 rejected the goal")
            self.result = self.handle.get_result_async()

        def status(self):
            if self.result is None or not self.result.done():
                return None
            code = self.result.result().status
            return {GoalStatus.STATUS_SUCCEEDED: "succeeded",
                    GoalStatus.STATUS_CANCELED: "canceled",
                    GoalStatus.STATUS_ABORTED: "aborted"}.get(code, f"status {code}")

        def cancel(self):
            if self.pending is not None:
                self.handle = self.wait(self.pending, 5.0)
                self.pending = None
                if self.handle.accepted:
                    self.result = self.handle.get_result_async()
            if self.handle is None or not self.handle.accepted:
                return
            if self.result is not None and self.result.done():
                self.handle = None
                return
            response = self.wait(self.handle.cancel_goal_async(), 5.0)
            if not response.goals_canceling and not self.result.done():
                raise RuntimeError("Nav2 did not confirm cancellation; no next goal was sent")
            self.wait(self.result, 10.0)
            if self.status() not in ("succeeded", "canceled"):
                raise RuntimeError("navigation failed while ending the current goal")
            self.handle = None

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    navigator = None
    result = 1
    previous_handlers = {}
    try:
        navigator = Navigator()

        def request_stop(_signum, _frame):
            navigator.stopping = True

        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, request_stop)
        navigator.wait_ready()
        execute_route(navigator, ((options.x1, options.y1), (options.x2, options.y2)),
                      options.goal_timeout)
        result = 0
    except InterruptedError as exc:
        print(str(exc), file=sys.stderr, flush=True)
        result = 130
    except Exception as exc:
        print(f"Two-point route failed: {exc}", file=sys.stderr, flush=True)
    finally:
        if navigator is not None:
            try:
                navigator.cancel()
            except Exception as exc:
                print(f"Cancellation could not be confirmed: {exc}", file=sys.stderr, flush=True)
                result = 1
            navigator.destroy_node()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == "__main__":
    sys.exit(main())
