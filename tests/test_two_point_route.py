"""Exercise route sequencing and its real navigation configuration without hardware."""

import argparse
import ast
from contextlib import redirect_stderr
import importlib.util
import io
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
BRINGUP = ROOT / "src/pb2025_sentry_nav/pb2025_nav_bringup"
SCRIPT = ROOT / "src/pb2025_sentry_nav/pb2025_sentry_nav/scripts/rm071_two_point_route.py"
spec = importlib.util.spec_from_file_location("two_point_route", SCRIPT)
route = importlib.util.module_from_spec(spec)
spec.loader.exec_module(route)


class FakeNavigator:
    def __init__(self, samples, cancellation_fails=False):
        self.samples = samples
        self.cancellation_fails = cancellation_fails
        self.stopping = False
        self.events = []
        self.started = []
        self.clock = 0.0
        self.current = (None, None)

    def now(self):
        return self.clock

    def info(self, _message):
        pass

    def start(self, target):
        self.events.append(("start", target))
        self.started.append(target)
        self.pending = iter(self.samples[len(self.started) - 1])
        self.current = (None, None)

    def spin(self, _timeout):
        self.clock += 0.1
        self.current = next(self.pending, self.current)

    def status(self):
        return self.current[1]

    def position(self):
        return self.current[0]

    def cancel(self):
        self.events.append(("cancel", self.started[-1]))
        if self.cancellation_fails:
            raise RuntimeError("cancellation not confirmed")


class TwoPointRouteTest(unittest.TestCase):
    points = ((0.0, 0.0), (2.0, 0.0))

    def test_switches_at_half_metre_and_stops_at_second_point(self):
        navigator = FakeNavigator([
            [((0.7, 0.0), None), ((0.51, 0.0), None), ((0.5, 0.0), None)],
            [((1.7, 0.0), None), ((1.79, 0.0), None), ((1.81, 0.0), None)],
        ])
        route.execute_route(navigator, self.points, 10.0)
        self.assertEqual(navigator.events, [
            ("start", self.points[0]), ("cancel", self.points[0]),
            ("start", self.points[1]), ("cancel", self.points[1]),
        ])
        self.assertAlmostEqual(navigator.clock, 0.6)

    def test_nav2_success_outside_requested_radius_does_not_skip_first_point(self):
        navigator = FakeNavigator([[((0.51, 0.0), "succeeded")]])
        with self.assertRaisesRegex(RuntimeError, "requested radius was not reached"):
            route.execute_route(navigator, self.points, 10.0)
        self.assertEqual(navigator.started, [self.points[0]])

    def test_failure_does_not_send_second_goal(self):
        navigator = FakeNavigator([[((1.0, 0.0), "aborted")]])
        with self.assertRaisesRegex(RuntimeError, "aborted"):
            route.execute_route(navigator, self.points, 10.0)
        self.assertEqual(navigator.started, [self.points[0]])
        self.assertEqual(navigator.events[-1][0], "cancel")

    def test_unconfirmed_cancellation_never_starts_second_goal(self):
        navigator = FakeNavigator([[((0.4, 0.0), None)]], cancellation_fails=True)
        with self.assertRaisesRegex(RuntimeError, "cancellation not confirmed"):
            route.execute_route(navigator, self.points, 10.0)
        self.assertEqual(navigator.started, [self.points[0]])

    def test_missing_pose_and_timeout_cancel_current_goal(self):
        for samples, timeout, exception in (
                ([[(None, None)]], 10.0, RuntimeError),
                ([[((1.0, 0.0), None)]], 0.3, TimeoutError)):
            with self.subTest(exception=exception):
                navigator = FakeNavigator(samples)
                with self.assertRaises(exception):
                    route.execute_route(navigator, self.points, timeout)
                self.assertEqual(navigator.started, [self.points[0]])
                self.assertEqual(navigator.events[-1][0], "cancel")

    def test_coordinates_are_explicit_and_finite(self):
        args = route.parse_arguments(["-1", "2", "3", "4"])
        self.assertEqual((args.x1, args.y1, args.x2, args.y2), (-1.0, 2.0, 3.0, 4.0))
        for invalid in ("nan", "inf", "-inf"):
            with self.assertRaises(argparse.ArgumentTypeError):
                route.finite_float(invalid)
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            route.parse_arguments([])

    def test_task_checker_matches_stop_radius_and_default_trees_keep_their_checker(self):
        config = yaml.safe_load((BRINGUP / "config/reality/nav2_params.yaml").read_text(encoding="utf-8"))
        controller = config["controller_server"]["ros__parameters"]
        self.assertEqual(controller["two_point_goal_checker"]["xy_goal_tolerance"], route.FINAL_RADIUS)
        self.assertLess(controller["FollowPath"]["final_stop_distance"], route.FINAL_RADIUS)
        self.assertGreater(controller["two_point_goal_checker"]["yaw_goal_tolerance"], 3.14159)
        for filename, checker in (
                ("navigate_two_point_leg.xml", "two_point_goal_checker"),
                ("navigate_to_pose_w_replanning_and_recovery.xml", "general_goal_checker"),
                ("navigate_through_poses_w_replanning_and_recovery.xml", "general_goal_checker")):
            follow = ET.parse(BRINGUP / "behavior_trees" / filename).find(".//FollowPath")
            self.assertEqual(follow.get("goal_checker_id"), checker)
            self.assertIn(checker, controller["goal_checker_plugins"])

    def test_omnidirectional_mode_survives_all_launch_defaults(self):
        config = yaml.safe_load((BRINGUP / "config/reality/nav2_params.yaml").read_text(encoding="utf-8"))
        self.assertFalse(config["fake_vel_transform"]["ros__parameters"]["forward_only_enabled"])
        self.assertFalse(config["controller_server"]["ros__parameters"]["FollowPath"]["use_rotate_to_heading"])
        self.assertGreaterEqual(
            config["controller_server"]["ros__parameters"]["FollowPath"]["min_approach_linear_velocity"],
            config["fake_vel_transform"]["ros__parameters"]["forward_min_linear_speed"])
        smoother = config["velocity_smoother"]["ros__parameters"]
        self.assertGreater(smoother["max_velocity"][1], 0.0)
        self.assertLess(smoother["min_velocity"][1], 0.0)
        for name in ("rm_navigation_reality_launch.py", "bringup_launch.py", "navigation_launch.py"):
            tree = ast.parse((BRINGUP / "launch" / name).read_text(encoding="utf-8"))
            declarations = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
                            and isinstance(node.func, ast.Name) and node.func.id == "DeclareLaunchArgument"
                            and node.args and isinstance(node.args[0], ast.Constant)
                            and node.args[0].value == "forward_only_enabled"]
            self.assertEqual(len(declarations), 1)
            default = next(item.value for item in declarations[0].keywords if item.arg == "default_value")
            self.assertEqual(ast.literal_eval(default).lower(), "false")
        main_launch = (BRINGUP / "launch/rm_navigation_reality_launch.py").read_text(encoding="utf-8")
        self.assertIn('forward_only_enabled = LaunchConfiguration("forward_only_enabled")', main_launch)


if __name__ == "__main__":
    unittest.main()
