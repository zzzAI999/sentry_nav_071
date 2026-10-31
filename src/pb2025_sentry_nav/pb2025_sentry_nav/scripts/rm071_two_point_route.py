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
    try:
        if isinstance(value, bool):
            raise ValueError
        number = float(value)
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError("coordinates must be numbers") from None
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError("coordinates must be finite")
    return number


def parse_arguments(arguments):
    parser = argparse.ArgumentParser(description=(
        "Visit (x1, y1), switch within 0.5 m, then stop within 0.2 m of (x2, y2). "
        "Coordinates use the map frame and metres. Normally started by start_navigation.sh."))
    parser.add_argument("coordinates", nargs="*", type=finite_float, metavar="X/Y")
    parser.add_argument("--config", type=Path, help="YAML file with first: [x, y], second: [x, y]")
    parser.add_argument("--print-points", action="store_true",
                        help="validate and print coordinates without starting ROS")
    options = parser.parse_args(arguments)
    if options.coordinates and options.config:
        parser.error("use four coordinates or --config, not both")
    if options.config:
        import yaml
        try:
            config = yaml.safe_load(options.config.read_text(encoding="utf-8"))
            if not isinstance(config, dict) or any(
                    not isinstance(config.get(key), list) or len(config[key]) != 2
                    for key in ("first", "second")):
                raise ValueError("expected first: [x, y] and second: [x, y]")
            options.coordinates = [finite_float(value) for key in ("first", "second")
                                   for value in config[key]]
        except (OSError, ValueError, argparse.ArgumentTypeError, yaml.YAMLError) as exc:
            parser.error(f"invalid route config {options.config}: {exc}")
    if len(options.coordinates) != 4:
        parser.error("provide exactly four coordinates: x1 y1 x2 y2, or use --config")
    return options


def execute_route(navigator, points):
    """Only advance after reaching the requested coordinate and ending its action."""
    for index, (target, radius) in enumerate(zip(points, (FIRST_RADIUS, FINAL_RADIUS)), 1):
        if navigator.stopping:
            raise InterruptedError("route interrupted")
        navigator.info(f"Point {index}/2: ({target[0]:.3f}, {target[1]:.3f}), radius={radius:.2f} m")
        missing_pose_since = None
        retry_at = None
        try:
            navigator.start(target)
            while True:
                navigator.spin(0.05)
                if navigator.stopping:
                    raise InterruptedError("route interrupted")
                status = navigator.status()
                if status == "canceled":
                    raise InterruptedError(f"point {index} canceled; route stopped")
                if status not in (None, "succeeded", "aborted"):
                    raise RuntimeError(f"point {index} navigation ended with {status}")
                position = navigator.position()
                if position is None:
                    if missing_pose_since is None:
                        missing_pose_since = navigator.now()
                    if navigator.now() - missing_pose_since > 2.0:
                        navigator.cancel()
                        navigator.info(
                            f"Fresh localization/TF unavailable; point {index} paused and retained until recovery")
                        navigator.wait_ready()
                        navigator.start(target)
                        missing_pose_since = None
                        retry_at = None
                        continue
                else:
                    missing_pose_since = None
                    distance = math.hypot(position[0] - target[0], position[1] - target[1])
                    if distance <= radius:
                        # Wait for this action to end before issuing the next one.
                        navigator.cancel()
                        if navigator.stopping:
                            raise InterruptedError("route interrupted while ending the current goal")
                        if index == 2:
                            stopped_position = navigator.position()
                            if stopped_position is None:
                                navigator.info("Final position unavailable; retaining the final target until recovery")
                                navigator.wait_ready()
                                navigator.start(target)
                                continue
                            distance = math.hypot(stopped_position[0] - target[0],
                                                  stopped_position[1] - target[1])
                            if distance > radius:
                                navigator.info(
                                    f"Final point is still {distance:.3f} m away after cancellation; correcting")
                                navigator.start(target)
                                continue
                        navigator.info(f"Point {index} reached: distance={distance:.3f} m")
                        break
                if status in ("succeeded", "aborted"):
                    if retry_at is None:
                        navigator.cancel()
                        navigator.info(
                            f"Point {index} not reached ({status}); retaining the same target and retrying")
                        retry_at = navigator.now() + 1.0
                if retry_at is not None and navigator.now() >= retry_at and position is not None:
                    navigator.start(target)
                    retry_at = None
        except BaseException:
            navigator.cancel()
            raise
    navigator.info("Two-point route complete. No further goals will be sent.")


def main(arguments=None):
    arguments = list(sys.argv[1:] if arguments is None else arguments)
    application_args = arguments
    if "--ros-args" in arguments:
        from rclpy.utilities import remove_ros_args
        application_args = remove_ros_args(args=[sys.argv[0], *arguments])[1:]
    options = parse_arguments(application_args)
    if options.print_points:
        for coordinate in options.coordinates:
            print(repr(coordinate))
        return 0
    import rclpy
    from action_msgs.msg import GoalStatus
    from ament_index_python.packages import get_package_share_directory
    from nav2_msgs.action import NavigateToPose
    from lifecycle_msgs.msg import State
    from lifecycle_msgs.srv import GetState
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener
    from std_msgs.msg import Bool

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
            self.localization_ready = False
            self.localization_received_at = float("-inf")
            self.state_client = self.create_client(GetState, "bt_navigator/get_state")
            self.state_request = None
            self.localization_subscription = self.create_subscription(
                Bool, "localization_ready", self.on_localization,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
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
            if not self.localization_ready or self.now() - self.localization_received_at > 2.0:
                return None
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

        def on_localization(self, message):
            self.localization_ready = message.data
            self.localization_received_at = self.now()

        def wait_ready(self):
            self.info("Waiting for accepted localization, map alignment, active Nav2 and fresh TF")
            next_report = self.now() + 10.0
            next_state_query = self.now()
            state_request_sent_at = self.now()
            while True:
                if self.stopping:
                    raise InterruptedError("route interrupted")
                self.spin(0.1)
                if self.state_request is not None and self.state_request.done():
                    response = self.state_request.result()
                    self.state_request = None
                    if (response is not None and response.current_state.id == State.PRIMARY_STATE_ACTIVE
                            and self.client.server_is_ready() and self.position() is not None):
                        self.info("Localization and Nav2 ready; starting the two-point route")
                        return
                elif self.state_request is not None and self.now() - state_request_sent_at > 2.0:
                    self.state_client.remove_pending_request(self.state_request)
                    self.state_request = None
                if (self.state_request is None and self.now() >= next_state_query
                        and self.state_client.service_is_ready()):
                    self.state_request = self.state_client.call_async(GetState.Request())
                    state_request_sent_at = self.now()
                    next_state_query = self.now() + 0.5
                if self.now() >= next_report:
                    self.info("Still waiting for localization/map alignment and active Nav2; no goal sent")
                    next_report = self.now() + 10.0

        def start(self, target):
            if self.stopping:
                raise InterruptedError("route interrupted before sending the next goal")
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

    rclpy.init(args=arguments, signal_handler_options=SignalHandlerOptions.NO)
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
        execute_route(navigator, (tuple(options.coordinates[:2]), tuple(options.coordinates[2:])))
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
