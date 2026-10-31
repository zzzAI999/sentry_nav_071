import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]

FAKE_ROS = r'''#!/usr/bin/env python3
import os
from pathlib import Path
import signal
import struct
import sys

args = sys.argv[1:]
values = dict(arg.split(":=", 1) for arg in args if ":=" in arg)
if values.get("mode") == "mapping":
    yaml_path = Path(values["map"])
    pcd_path = Path(values["prior_pcd_file"])
    yaml_path.write_text("image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n", encoding="utf-8")
    yaml_path.with_suffix(".pgm").write_bytes(b"P5\n8 8\n255\n" + bytes([254] * 64))
    if not os.environ.get("FAKE_INCOMPLETE"):
        header = ("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                  "WIDTH 1000\nHEIGHT 1\nPOINTS 1000\nDATA binary\n")
        pcd_path.write_bytes(header.encode("ascii") + struct.pack("<fff", 1, 2, 3) * 1000)
    if os.environ.get("FAKE_CTRL_C"):
        os.kill(os.getppid(), signal.SIGINT)
        sys.exit(130)
elif values.get("mode") == "navigation":
    Path(os.environ["FAKE_NAV_ARGS"]).write_text("\n".join(args), encoding="utf-8")
else:
    sys.exit(2)
'''


@unittest.skipIf(sys.platform == "win32" or not shutil.which("bash"), "requires Linux bash")
class ShellWorkflowTest(unittest.TestCase):
    def test_ctrl_c_saves_and_next_navigation_uses_latest_complete_map(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "ros2").write_text(FAKE_ROS, encoding="utf-8")
            (root / "ros2").chmod(0o755)
            setup = root / "setup.bash"
            setup.write_text("# isolated test setup\n", encoding="utf-8")
            env = os.environ.copy()
            env.update({"PATH": f"{root}:{env['PATH']}", "SENTRY_ROS_SETUP": str(setup),
                        "SENTRY_INSTALL_SETUP": str(setup), "SENTRY_MAP_ROOT": str(root / "maps"),
                        "SENTRY_ENABLE_MCU_SERIAL": "False", "SENTRY_AUTO_EXPLORE": "False",
                        "SENTRY_USE_RVIZ": "False", "FAKE_NAV_ARGS": str(root / "nav_args"),
                        "FAKE_CTRL_C": "1"})
            mapping = subprocess.run(["bash", str(ROOT / "start_mapping.sh")], env=env,
                                     capture_output=True, text=True, timeout=30)
            self.assertEqual(mapping.returncode, 0, mapping.stdout + mapping.stderr)
            latest = root / "maps/latest.json"
            previous = latest.read_bytes()
            saved = json.loads(previous)["files"]
            navigation = subprocess.run(["bash", str(ROOT / "start_navigation.sh"), "1", "2", "3", "4"], env=env,
                                        capture_output=True, text=True, timeout=30)
            self.assertEqual(navigation.returncode, 0, navigation.stdout + navigation.stderr)
            args = (root / "nav_args").read_text(encoding="utf-8").splitlines()
            self.assertIn(f"map:={saved['yaml']['path']}", args)
            self.assertIn(f"prior_pcd_file:={saved['pcd']['path']}", args)
            self.assertIn("enable_mcu_status_nav:=False", args)
            self.assertIn("enable_two_point_route:=True", args)
            self.assertIn("route_x1:=1.0", args)
            self.assertIn("route_y2:=4.0", args)

            env["FAKE_INCOMPLETE"] = "1"
            failed = subprocess.run(["bash", str(ROOT / "start_mapping.sh")], env=env,
                                    capture_output=True, text=True, timeout=30)
            self.assertNotEqual(failed.returncode, 0)
            self.assertEqual(latest.read_bytes(), previous)


if __name__ == "__main__":
    unittest.main()
