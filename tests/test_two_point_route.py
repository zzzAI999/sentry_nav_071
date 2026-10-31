"""Exercise route sequencing and its real navigation configuration without hardware."""

import argparse
import ast
from contextlib import redirect_stderr
import importlib.util
import io
from pathlib import Path
import tempfile
import types
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
    def __init__(self, samples, cancellation_fails=False, cancel_hook=None, wait_hook=None):
        self.samples = samples
        self.cancellation_fails = cancellation_fails
        self.cancel_hook = cancel_hook
        self.wait_hook = wait_hook
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

    def wait_ready(self):
        self.events.append(("wait_ready", self.started[-1]))
        if self.wait_hook:
            self.wait_hook(self)
        if self.stopping:
            raise InterruptedError("route interrupted during localization recovery")

    def cancel(self):
        self.events.append(("cancel", self.started[-1]))
        if self.cancel_hook:
            self.cancel_hook(self)
        if self.cancellation_fails:
            raise RuntimeError("cancellation not confirmed")


class TwoPointRouteTest(unittest.TestCase):
    points = ((0.0, 0.0), (2.0, 0.0))

    def test_switches_at_half_metre_and_stops_at_second_point(self):
        navigator = FakeNavigator([
            [((0.7, 0.0), None), ((0.51, 0.0), None), ((0.5, 0.0), None)],
            [((1.7, 0.0), None), ((1.79, 0.0), None), ((1.81, 0.0), None)],
        ])
        route.execute_route(navigator, self.points)
        self.assertEqual(navigator.events, [
            ("start", self.points[0]), ("cancel", self.points[0]),
            ("start", self.points[1]), ("cancel", self.points[1]),
        ])
        self.assertAlmostEqual(navigator.clock, 0.6)

    def test_nav2_success_outside_requested_radius_does_not_skip_first_point(self):
        navigator = FakeNavigator([
            [((0.51, 0.0), "succeeded")],
            [((0.49, 0.0), None)],
            [((1.9, 0.0), None)],
        ])
        route.execute_route(navigator, self.points)
        self.assertEqual(navigator.started, [self.points[0], self.points[0], self.points[1]])

    def test_aborted_attempt_retries_the_same_target_and_finishes(self):
        navigator = FakeNavigator([
            [((1.0, 0.0), "aborted")],
            [((1.0, 0.0), "aborted")],
            [((0.4, 0.0), None)],
            [((1.9, 0.0), None)],
        ])
        route.execute_route(navigator, self.points)
        self.assertEqual(navigator.started, [self.points[0]] * 3 + [self.points[1]])
        self.assertGreaterEqual(navigator.clock, 2.0)

    def test_explicit_cancellation_does_not_restart_or_send_second_goal(self):
        navigator = FakeNavigator([[((1.0, 0.0), "canceled")]])
        with self.assertRaises(InterruptedError):
            route.execute_route(navigator, self.points)
        self.assertEqual(navigator.started, [self.points[0]])
        self.assertEqual(navigator.events[-1][0], "cancel")

    def test_unconfirmed_cancellation_never_starts_second_goal(self):
        navigator = FakeNavigator([[((0.4, 0.0), None)]], cancellation_fails=True)
        with self.assertRaisesRegex(RuntimeError, "cancellation not confirmed"):
            route.execute_route(navigator, self.points)
        self.assertEqual(navigator.started, [self.points[0]])

    def test_pose_loss_stops_waits_then_retries_the_same_target(self):
        navigator = FakeNavigator([
            [(None, None)],
            [((0.4, 0.0), None)],
            [((1.9, 0.0), None)],
        ])
        route.execute_route(navigator, self.points)
        self.assertEqual(navigator.started, [self.points[0], self.points[0], self.points[1]])
        self.assertEqual(navigator.events[:4], [
            ("start", self.points[0]), ("cancel", self.points[0]),
            ("wait_ready", self.points[0]), ("start", self.points[0])])

    def test_localization_wait_remains_cancelable(self):
        navigator = FakeNavigator([[(None, None)]],
                                  wait_hook=lambda node: setattr(node, "stopping", True))
        with self.assertRaises(InterruptedError):
            route.execute_route(navigator, self.points)
        self.assertEqual(navigator.started, [self.points[0]])

    def test_long_blockage_does_not_expire_or_skip_the_goal(self):
        navigator = FakeNavigator([
            [((1.0, 0.0), None)] * 4000 + [((0.4, 0.0), None)],
            [((1.9, 0.0), None)],
        ])
        route.execute_route(navigator, self.points)
        self.assertGreater(navigator.clock, 400.0)
        self.assertEqual(navigator.started, list(self.points))
        self.assertEqual([kind for kind, _ in navigator.events], ["start", "cancel", "start", "cancel"])

    def test_final_position_is_checked_again_after_cancellation(self):
        def overshoot(navigator):
            if len(navigator.started) == 2:
                navigator.current = ((1.7, 0.0), None)

        navigator = FakeNavigator([
            [((0.4, 0.0), None)],
            [((1.9, 0.0), None)],
            [((1.9, 0.0), None)],
        ], cancel_hook=overshoot)
        route.execute_route(navigator, self.points)
        self.assertEqual(navigator.started, [self.points[0], self.points[1], self.points[1]])

    def test_interrupt_during_cancellation_cannot_report_success(self):
        for interrupted_leg in (1, 2):
            def request_stop(navigator):
                if len(navigator.started) == interrupted_leg:
                    navigator.stopping = True

            navigator = FakeNavigator([
                [((0.4, 0.0), None)], [((1.9, 0.0), None)]], cancel_hook=request_stop)
            with self.subTest(interrupted_leg=interrupted_leg), self.assertRaises(InterruptedError):
                route.execute_route(navigator, self.points)
            self.assertEqual(navigator.started, list(self.points[:interrupted_leg]))

    def test_coordinates_are_explicit_and_finite(self):
        args = route.parse_arguments(["-1", "2", "3", "4"])
        self.assertEqual(args.coordinates, [-1.0, 2.0, 3.0, 4.0])
        for invalid in ("nan", "inf", "-inf"):
            with self.assertRaises(argparse.ArgumentTypeError):
                route.finite_float(invalid)
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            route.parse_arguments([])

    def test_config_coordinates_and_invalid_placeholders(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "route.yaml"
            config.write_text("first: [-1, 2]\nsecond: [3, 4]\n", encoding="utf-8")
            self.assertEqual(route.parse_arguments(["--config", str(config)]).coordinates,
                             [-1.0, 2.0, 3.0, 4.0])
            for content in ("first: [null, null]\nsecond: [null, null]\n",
                            "first: [true, 0]\nsecond: [1, 2]\n",
                            "first: [0]\nsecond: [1, 2]\n", "[]\n"):
                config.write_text(content, encoding="utf-8")
                with self.subTest(content=content), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    route.parse_arguments(["--config", str(config)])

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
            retry = ET.parse(BRINGUP / "behavior_trees" / filename).find(".//RetryUntilSuccessful")
            self.assertEqual(retry.get("num_attempts"), "-1")
            delay = retry.find("Fallback/Delay")
            self.assertGreater(float(delay.get("delay_msec")), 0.0)
            self.assertIsNotNone(delay.find("AlwaysFailure"))

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


class NavigationReadinessTest(unittest.TestCase):
    """Run the ROS adapter's actual readiness methods with controlled TF/lifecycle inputs."""

    def adapter(self):
        tree = ast.parse(SCRIPT.read_text(encoding="utf-8"))
        original = next(node for node in ast.walk(tree) if isinstance(node, ast.ClassDef)
                        and node.name == "Navigator")
        methods = [node for node in original.body if isinstance(node, ast.FunctionDef)
                   and node.name in ("on_localization", "position", "wait_ready")]
        cls = ast.ClassDef(name="Adapter", bases=[], keywords=[], body=methods, decorator_list=[])
        module = ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[]))
        scope = {"math": route.math, "Time": lambda: None, "TransformException": LookupError,
                 "State": types.SimpleNamespace(PRIMARY_STATE_ACTIVE=3),
                 "GetState": types.SimpleNamespace(Request=lambda: None)}
        exec(compile(module, str(SCRIPT), "exec"), scope)
        node = scope["Adapter"]()
        node.clock = 0.0
        node.now = lambda: node.clock
        node.localization_ready = False
        node.localization_received_at = float("-inf")
        node.on_localization(types.SimpleNamespace(data=False))
        node.get_clock = lambda: types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(nanoseconds=round(node.clock * 1e9)))
        node.stale_tf = False

        def transform(*_args):
            stamp_ns = round(node.clock * 1e9) - (5_000_000_000 if node.stale_tf else 0)
            return types.SimpleNamespace(
                header=types.SimpleNamespace(stamp=types.SimpleNamespace(
                    sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)),
                transform=types.SimpleNamespace(translation=types.SimpleNamespace(x=1.0, y=2.0)))

        node.tf_buffer = types.SimpleNamespace(lookup_transform=transform)
        return node

    def test_initial_identity_tf_is_insufficient_and_heartbeat_must_stay_fresh(self):
        node = self.adapter()
        self.assertIsNone(node.position())
        node.on_localization(types.SimpleNamespace(data=True))
        self.assertEqual(node.position(), (1.0, 2.0))
        node.clock += 3.0
        self.assertIsNone(node.position())

    def test_waits_for_localization_active_nav2_and_fresh_tf_together(self):
        node = self.adapter()
        node.stopping = False
        node.state_request = None
        node.info = lambda _message: None
        node.tick = 0
        node.client = types.SimpleNamespace(server_is_ready=lambda: True)
        future = types.SimpleNamespace(
            done=lambda: True,
            result=lambda: types.SimpleNamespace(current_state=types.SimpleNamespace(
                id=1 if 5 <= node.tick < 9 else 3)))
        node.state_client = types.SimpleNamespace(
            service_is_ready=lambda: True, call_async=lambda _request: future)

        def spin(_timeout):
            node.tick += 1
            self.assertLess(node.tick, 20, "readiness never became true")
            node.clock += 0.1
            node.on_localization(types.SimpleNamespace(data=node.tick >= 5))
            node.stale_tf = node.tick == 9

        node.spin = spin
        node.wait_ready()
        self.assertGreaterEqual(node.tick, 10)

    def test_readiness_query_recovers_when_a_lifecycle_response_is_lost(self):
        node = self.adapter()
        node.stopping = False
        node.state_request = None
        node.info = lambda _message: None
        node.client = types.SimpleNamespace(server_is_ready=lambda: True)
        stalled = types.SimpleNamespace(done=lambda: False)
        active = types.SimpleNamespace(done=lambda: True, result=lambda: types.SimpleNamespace(
            current_state=types.SimpleNamespace(id=3)))
        requests = []
        removed = []

        def call(_request):
            future = stalled if not requests else active
            requests.append(future)
            return future

        node.state_client = types.SimpleNamespace(
            service_is_ready=lambda: True, call_async=call, remove_pending_request=removed.append)

        def spin(_timeout):
            node.clock += 0.5
            self.assertLess(node.clock, 10.0)
            node.on_localization(types.SimpleNamespace(data=True))

        node.spin = spin
        node.wait_ready()
        self.assertEqual(requests, [stalled, active])
        self.assertEqual(removed, [stalled])


if __name__ == "__main__":
    unittest.main()
