"""Check graceful mapping completion and retained exploration readiness without ROS."""

import ast
from collections import Counter
import importlib.util
from pathlib import Path
import signal
import struct
import sys
import tempfile
import types
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "src/pb2025_sentry_nav/pb2025_sentry_nav/scripts"


def load_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


manifest = load_file("startup_manifest", ROOT / "tools/map_manifest.py")
with patch.dict(sys.modules, {"map_manifest": manifest}):
    session = load_file("mapping_session", ROOT / "tools/mapping_session.py")


def write_map(folder, complete=True):
    folder.mkdir(parents=True, exist_ok=True)
    yaml_path, pcd_path = folder / "map.yaml", folder / "map.pcd"
    yaml_path.write_text("image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n", encoding="utf-8")
    yaml_path.with_suffix(".pgm").write_bytes(b"P5\n8 8\n255\n" + bytes([254] * 64))
    if complete:
        header = ("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                  "WIDTH 1000\nHEIGHT 1\nPOINTS 1000\nDATA binary\n")
        pcd_path.write_bytes(header.encode("ascii") + struct.pack("<fff", 1, 2, 3) * 1000)
    return yaml_path, pcd_path


class MappingSessionTest(unittest.TestCase):
    def run_case(self, folder, stopped=True, complete=True, exit_status=0):
        latest = folder / "latest.json"
        old_yaml, old_pcd = write_map(folder / "old")
        manifest.record_map(latest, old_yaml, old_pcd)
        previous = latest.read_bytes()
        yaml_path, pcd_path = folder / "new/map.yaml", folder / "new/map.pcd"
        handlers, events = {}, []

        def set_handler(signum, handler):
            old = handlers.get(signum, signal.SIG_DFL)
            handlers[signum] = handler
            return old

        class Process:
            pid = 4567
            finished = False

            def poll(self):
                return exit_status if self.finished else None

            def wait(self):
                if stopped:
                    handlers[signal.SIGTERM](signal.SIGTERM, None)
                    handlers[signal.SIGINT](signal.SIGINT, None)
                self_test.assertEqual(latest.read_bytes(), previous)
                write_map(folder / "new", complete)
                events.append("saved")
                self.finished = True
                return exit_status

        def record(*args):
            events.append("manifest")
            return manifest.record_map(*args)

        self_test = self
        with patch.object(session.signal, "signal", side_effect=set_handler), \
                patch.object(session.subprocess, "Popen", return_value=Process()) as popen, \
                patch.object(session.os, "killpg", create=True) as stop, \
                patch.object(session, "record_map", side_effect=record):
            result = session.run_session(["ros2", "launch", "test"], latest, yaml_path, pcd_path)
        self.assertTrue(popen.call_args.kwargs["start_new_session"])
        if stopped:
            stop.assert_called_once_with(4567, signal.SIGINT)
        else:
            stop.assert_not_called()
        return result, latest, previous, events

    def test_term_is_forwarded_once_and_manifest_waits_for_saved_pcd(self):
        with tempfile.TemporaryDirectory() as directory:
            result, latest, previous, events = self.run_case(Path(directory))
            self.assertEqual(result, 0)
            self.assertNotEqual(latest.read_bytes(), previous)
            self.assertEqual(events, ["saved", "manifest"])

    def test_incomplete_save_and_abnormal_exit_keep_previous_map(self):
        for kwargs in ({"complete": False}, {"stopped": False, "exit_status": 1}):
            with self.subTest(kwargs=kwargs), tempfile.TemporaryDirectory() as directory:
                result, latest, previous, _events = self.run_case(Path(directory), **kwargs)
                self.assertEqual(result, 1)
                self.assertEqual(latest.read_bytes(), previous)

    def test_closed_output_cannot_prevent_manifest_selection(self):
        for exception in (ValueError("I/O operation on closed file"), BrokenPipeError()):
            with self.subTest(exception=type(exception).__name__), \
                    tempfile.TemporaryDirectory() as directory, \
                    patch("builtins.print", side_effect=exception):
                result, latest, previous, events = self.run_case(Path(directory))
                self.assertEqual(result, 0)
                self.assertNotEqual(latest.read_bytes(), previous)
                self.assertEqual(events, ["saved", "manifest"])


def methods_from(path, class_name, names, namespace):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    original = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == class_name)
    methods = [node for node in original.body if isinstance(node, ast.FunctionDef) and node.name in names]
    cls = ast.ClassDef(name="Methods", bases=[], keywords=[], body=methods, decorator_list=[])
    module = ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[]))
    exec(compile(module, str(path), "exec"), namespace)
    return namespace["Methods"]()


class MappingReadinessTest(unittest.TestCase):
    def starter(self):
        clock = types.SimpleNamespace(value=0.0)
        node = methods_from(SCRIPTS / "rm071_explore_map_ready_starter.py", "ExploreMapReadyStarter",
                            {"_map_callback", "_publish_resume_if_ready", "_log_waiting"},
                            {"time": types.SimpleNamespace(monotonic=lambda: clock.value),
                             "Counter": Counter, "OccupancyGrid": object, "Bool": types.SimpleNamespace})
        node.start_time = 0.0
        node.startup_delay_sec = 5.0
        node.min_free_cells = node.min_unknown_cells = 20
        node.required_consecutive_maps = 3
        node.ready_count = node.resume_sent = 0
        node.publish_count = 1
        node.latest_map_ready = node.resume_started = False
        node.last_wait_log = 0.0
        node.get_logger = lambda: types.SimpleNamespace(info=lambda _message: None)
        sent = []
        node.resume_pub = types.SimpleNamespace(get_subscription_count=lambda: 1, publish=sent.append)
        return node, clock, sent

    def test_retained_map_received_before_delay_can_resume_without_new_maps(self):
        node, clock, sent = self.starter()
        node._map_callback(types.SimpleNamespace(info=types.SimpleNamespace(width=8, height=8),
                                                data=[0] * 32 + [-1] * 32))
        for moment in (1.0, 4.0):
            clock.value = moment
            node._publish_resume_if_ready()
        self.assertEqual(sent, [])
        for moment in (5.0, 5.25, 5.5, 5.75):
            clock.value = moment
            node._publish_resume_if_ready()
        self.assertEqual(len(sent), 1)
        self.assertTrue(sent[0].data)

    def test_invalid_map_revokes_cached_readiness(self):
        node, clock, sent = self.starter()
        clock.value = 6.0
        node._map_callback(types.SimpleNamespace(info=types.SimpleNamespace(width=8, height=8),
                                                data=[0] * 32 + [-1] * 32))
        node._publish_resume_if_ready()
        node._map_callback(types.SimpleNamespace(info=types.SimpleNamespace(width=0, height=0), data=[]))
        for _ in range(4):
            node._publish_resume_if_ready()
        self.assertEqual(sent, [])

    def test_completed_map_keeps_flowing_to_navigation(self):
        launch = ROOT / "src/pb2025_sentry_nav/pb2025_nav_bringup/launch/rm_navigation_reality_launch.py"
        tree = ast.parse(launch.read_text(encoding="utf-8"))
        call = next(node.value for node in ast.walk(tree) if isinstance(node, ast.Assign)
                    and any(isinstance(target, ast.Name) and target.id == "start_valid_map_filter_node"
                            for target in node.targets))
        parameters = ast.literal_eval(next(kw.value for kw in call.keywords if kw.arg == "parameters"))[0]
        node = methods_from(SCRIPTS / "rm071_valid_map_filter.py", "ValidMapFilter",
                            {"_map_callback", "_log_invalid"}, {"OccupancyGrid": object})
        for name in ("min_known_cells", "min_free_cells", "min_unknown_cells"):
            setattr(node, name, parameters[name])
        node.valid_count = node.invalid_count = 0
        node.warn_every_invalid = 20
        node.get_logger = lambda: types.SimpleNamespace(info=lambda _message: None, warn=lambda _message: None)
        sent = []
        node.publisher = types.SimpleNamespace(publish=sent.append)
        node._map_callback(types.SimpleNamespace(info=types.SimpleNamespace(width=8, height=8), data=[0] * 64))
        self.assertEqual(len(sent), 1)


if __name__ == "__main__":
    unittest.main()
