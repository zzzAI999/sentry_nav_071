"""Exercise the single startup command with real validation and a fake ros2 executable."""

import importlib.util
import os
from pathlib import Path
import shlex
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
BASH = os.environ.get("SENTRY_TEST_BASH") or (shutil.which("bash") if os.name != "nt" else None)


def bash_path(path):
    value = str(Path(path).resolve()).replace("\\", "/")
    return "/" + value[0].lower() + value[2:] if os.name == "nt" else value


@unittest.skipUnless(BASH, "requires bash; on Windows set SENTRY_TEST_BASH to Git Bash")
class NavigationEntrypointTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        root = Path(self.temporary.name)
        self.workspace = root / "workspace with spaces"
        self.commands = root / "commands"
        self.commands.mkdir()
        for name in ("start_navigation.sh", "start_navigation_sim.sh", "config/two_point_route.yaml", "tools/map_manifest.py",
                     "src/pb2025_sentry_nav/pb2025_sentry_nav/scripts/rm071_two_point_route.py"):
            target = self.workspace / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / name, target)
        self.write_command("python3", f"exec {shlex.quote(bash_path(sys.executable))} \"$@\"\n")
        self.write_command("ros2", 'printf "%s\\n" "$@" > "$SENTRY_TEST_CAPTURE"\n'
                           'printf "%s\\n" "${ROS_DOMAIN_ID:-0}" > "$SENTRY_TEST_DOMAIN"\n')
        if os.name == "nt":
            # Git Bash lacks flock. Linux runs the real lock implementation.
            self.write_command("flock", "exit 0\n")
        self.setup = self.commands / "setup.bash"
        self.setup.write_text('export PATH="$(dirname -- "${BASH_SOURCE[0]}"):$PATH"\n',
                              encoding="utf-8", newline="\n")
        self.map_root = root / "maps"
        saved = root / "saved map"
        saved.mkdir()
        self.yaml_path = saved / "map.yaml"
        self.pcd_path = saved / "map.pcd"
        self.yaml_path.write_text("image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n", encoding="utf-8")
        self.yaml_path.with_suffix(".pgm").write_bytes(b"P5\n8 8\n255\n" + bytes([254] * 64))
        header = ("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                  "WIDTH 1000\nHEIGHT 1\nPOINTS 1000\nDATA binary\n")
        self.pcd_path.write_bytes(header.encode("ascii") + struct.pack("<fff", 1, 2, 3) * 1000)
        spec = importlib.util.spec_from_file_location("entrypoint_manifest", ROOT / "tools/map_manifest.py")
        manifest = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(manifest)
        manifest.record_map(self.map_root / "latest.json", self.yaml_path, self.pcd_path)
        self.capture = root / "ros_arguments.txt"
        self.domain_capture = root / "domain.txt"
        self.env = {**os.environ, "SENTRY_ROS_SETUP": bash_path(self.setup),
                    "SENTRY_INSTALL_SETUP": bash_path(self.setup),
                    "SENTRY_MAP_ROOT": bash_path(self.map_root),
                    "SENTRY_USE_RVIZ": "False", "SENTRY_ENABLE_MCU_SERIAL": "False",
                    "SENTRY_TEST_CAPTURE": bash_path(self.capture),
                    "SENTRY_TEST_DOMAIN": bash_path(self.domain_capture),
                    "ROS_DOMAIN_ID": "0", "SENTRY_SIM_DOMAIN_ID": "71"}
        self.env.pop("SENTRY_ROUTE_CONFIG", None)

    def write_command(self, name, body):
        path = self.commands / name
        path.write_text("#!/usr/bin/env bash\n" + body, encoding="utf-8", newline="\n")
        path.chmod(0o755)

    def run_navigation(self, *args, expected=0, script="start_navigation.sh"):
        self.capture.unlink(missing_ok=True)
        result = subprocess.run([BASH, bash_path(self.workspace / script), *args],
                                env=self.env, capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, expected, result.stdout + result.stderr)
        return self.capture.read_text(encoding="utf-8").splitlines() if self.capture.exists() else []

    def test_one_command_selects_map_and_enables_both_points(self):
        args = self.run_navigation("-1", "2", "3", "4", "use_composition:=False")
        for value in ("mode:=navigation", f"map:={self.yaml_path}",
                      f"prior_pcd_file:={self.pcd_path}", "enable_two_point_route:=True",
                      "route_x1:=-1.0", "route_y1:=2.0", "route_x2:=3.0", "route_y2:=4.0",
                      "enable_mcu_status_nav:=False", "use_composition:=False"):
            self.assertIn(value, args)

    def test_no_arguments_uses_saved_points(self):
        (self.workspace / "config/two_point_route.yaml").write_text(
            "first: [1, 2]\nsecond: [3, 4]\n", encoding="utf-8")
        args = self.run_navigation()
        self.assertIn("enable_two_point_route:=True", args)
        self.assertIn("route_x1:=1.0", args)
        self.assertIn("route_y2:=4.0", args)

    def test_invalid_points_and_overrides_do_not_launch_navigation(self):
        for args in ((), ("1", "2"), ("nan", "2", "3", "4"),
                     ("1", "2", "3", "4", "enable_two_point_route:=False"),
                     ("1", "2", "3", "4", "map:=other.yaml")):
            with self.subTest(args=args):
                self.assertEqual(self.run_navigation(*args, expected=2), [])

    def test_missing_map_or_setup_does_not_launch_navigation(self):
        (self.map_root / "latest.json").unlink()
        self.assertEqual(self.run_navigation("1", "2", "3", "4", expected=1), [])
        self.env["SENTRY_ROS_SETUP"] = bash_path(self.map_root / "missing.bash")
        self.assertEqual(self.run_navigation("1", "2", "3", "4", expected=2), [])

    def test_shell_syntax_and_line_endings(self):
        for name in ("start_mapping.sh", "start_navigation.sh", "start_navigation_sim.sh"):
            with self.subTest(name=name):
                source = ROOT / name
                self.assertNotIn(b"\r\n", source.read_bytes())
                result = subprocess.run([BASH, "-n", bash_path(source)], capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)

    def test_simulation_uses_latest_map_and_an_isolated_domain(self):
        args = self.run_navigation(script="start_navigation_sim.sh")
        self.assertIn("map_simulation_launch.py", args)
        self.assertIn(f"map:={self.yaml_path}", args)
        self.assertIn("enable_two_point_route:=False", args)
        self.assertFalse(any(arg.endswith(":=") for arg in args), args)
        self.assertFalse(any(arg.startswith("route_") for arg in args), args)
        self.assertEqual(self.domain_capture.read_text(encoding="utf-8").strip(), "71")

    def test_simulation_can_start_the_same_two_point_task(self):
        args = self.run_navigation("1", "2", "3", "4", script="start_navigation_sim.sh")
        self.assertIn("enable_two_point_route:=True", args)
        self.assertIn("route_x1:=1.0", args)
        self.assertIn("route_y2:=4.0", args)

    def test_simulation_refuses_the_live_domain_or_hardware_options(self):
        self.env["SENTRY_SIM_DOMAIN_ID"] = "0"
        self.assertEqual(self.run_navigation(script="start_navigation_sim.sh", expected=2), [])
        self.env["SENTRY_SIM_DOMAIN_ID"] = "71"
        self.assertEqual(self.run_navigation("enable_mcu_status_nav:=True",
                                            script="start_navigation_sim.sh", expected=2), [])


if __name__ == "__main__":
    unittest.main()
