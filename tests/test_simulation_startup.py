"""Verify that simulated Nav2 waits for a placed robot and fresh transforms."""

import ast
from pathlib import Path
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "src/pb2025_sentry_nav/pb2025_sentry_nav/scripts/rm071_sim_startup.py"


class SimulationStartupTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        tree = ast.parse(SCRIPT.read_text(encoding="utf-8"))
        original = next(item for item in tree.body if isinstance(item, ast.ClassDef))
        body = [item for item in original.body if isinstance(item, ast.FunctionDef)
                and item.name in {"on_ready", "tick"}]
        module = ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(
            name="StartupMethods", bases=[], keywords=[], body=body, decorator_list=[])],
            type_ignores=[]))

        class Request:
            STARTUP = 0

        scope = {"time": time, "Time": lambda: None, "TransformException": LookupError,
                 "ManageLifecycleNodes": SimpleNamespace(Request=Request)}
        exec(compile(module, str(SCRIPT), "exec"), scope)
        cls.startup_type = scope["StartupMethods"]

    def starter(self):
        node = self.startup_type()
        node.ready, node.ready_at = False, float("-inf")
        node.finished, node.failed, node.future = False, False, None
        node.client = Mock()
        node.client.service_is_ready.return_value = True
        node.client.call_async.return_value.done.return_value = False
        node.client.call_async.return_value.result.return_value = SimpleNamespace(success=True)
        node.tf_buffer = Mock()
        node.tf_buffer.lookup_transform.return_value = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)))
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10e9))
        node.get_logger = Mock()
        return node

    def test_no_initial_pose_or_revoked_readiness_keeps_navigation_idle(self):
        node = self.starter()
        with patch.object(time, "monotonic", return_value=20):
            node.tick()
            node.on_ready(SimpleNamespace(data=True))
            node.on_ready(SimpleNamespace(data=False))
            node.tick()
        node.client.call_async.assert_not_called()
        self.assertFalse(node.finished)

    def test_missing_or_stale_transforms_and_readiness_do_not_start_navigation(self):
        node = self.starter()
        node.ready, node.ready_at = True, 20
        with patch.object(time, "monotonic", return_value=20):
            node.tf_buffer.lookup_transform.side_effect = LookupError("map not available yet")
            node.tick()
            node.tf_buffer.lookup_transform.side_effect = None
            stamp = node.tf_buffer.lookup_transform.return_value.header.stamp
            stamp.sec = 7
            node.tick()
            stamp.sec = 10
        with patch.object(time, "monotonic", return_value=23):
            node.tick()
        node.client.call_async.assert_not_called()

    def test_fresh_pose_starts_once_and_waits_for_the_lifecycle_reply(self):
        node = self.starter()
        with patch.object(time, "monotonic", return_value=20):
            node.on_ready(SimpleNamespace(data=True))
            node.tick()
            node.tick()
            node.client.call_async.assert_called_once()
            self.assertEqual(node.client.call_async.call_args.args[0].command, 0)
            node.future.done.return_value = True
            node.tick()
            node.tick()
        self.assertTrue(node.finished)
        self.assertFalse(node.failed)
        node.client.call_async.assert_called_once()

    def test_rejected_startup_reports_failure_without_duplicate_requests(self):
        node = self.starter()
        with patch.object(time, "monotonic", return_value=20):
            node.on_ready(SimpleNamespace(data=True))
            node.tick()
            node.future.done.return_value = True
            node.future.result.return_value = SimpleNamespace(success=False)
            node.tick()
            node.tick()
        self.assertTrue(node.finished)
        self.assertTrue(node.failed)
        node.client.call_async.assert_called_once()
        node.get_logger.return_value.error.assert_called_once()


if __name__ == "__main__":
    unittest.main()
