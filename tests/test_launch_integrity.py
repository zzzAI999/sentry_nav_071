import ast
import builtins
from pathlib import Path
import re
import symtable
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
NAV = ROOT / "src/pb2025_sentry_nav"
LAUNCH = NAV / "pb2025_nav_bringup/launch/rm_navigation_reality_launch.py"


class LaunchIntegrityTest(unittest.TestCase):
    def setUp(self):
        self.source = LAUNCH.read_text(encoding="utf-8")
        self.tree = ast.parse(self.source, filename=str(LAUNCH))

    def test_launch_has_no_undefined_globals(self):
        table = symtable.symtable(self.source, str(LAUNCH), "exec")
        known = set(table.get_identifiers()) | set(dir(builtins))
        function = next(child for child in table.get_children()
                        if child.get_name() == "generate_launch_description")
        missing = sorted(symbol.get_name() for symbol in function.get_symbols()
                         if symbol.is_global() and symbol.is_referenced()
                         and symbol.get_name() not in known)
        self.assertEqual(missing, [])

    def test_all_launch_configurations_are_declared(self):
        configurations = set()
        declared = set()
        for node in ast.walk(self.tree):
            if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
                continue
            if not node.args or not isinstance(node.args[0], ast.Constant):
                continue
            if node.func.id == "LaunchConfiguration":
                configurations.add(node.args[0].value)
            elif node.func.id == "DeclareLaunchArgument":
                declared.add(node.args[0].value)
        self.assertEqual(configurations - declared, set())

    def test_installed_local_scripts_exist(self):
        package = NAV / "pb2025_sentry_nav"
        cmake = (package / "CMakeLists.txt").read_text(encoding="utf-8")
        scripts = re.findall(r"\bscripts/([\w.]+\.py)", cmake)
        self.assertTrue(scripts)
        for script in scripts:
            with self.subTest(script=script):
                self.assertTrue((package / "scripts" / script).is_file())
        for node in ast.walk(self.tree):
            if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name) or node.func.id != "Node":
                continue
            values = {kw.arg: kw.value.value for kw in node.keywords
                      if isinstance(kw.value, ast.Constant)}
            if values.get("package") == "pb2025_sentry_nav":
                self.assertIn(values.get("executable"), scripts)

    def test_ros_package_names_are_unique(self):
        names = [ET.parse(path).getroot().findtext("name")
                 for path in (ROOT / "src").rglob("package.xml")]
        self.assertEqual(len(names), len(set(names)))
        for required in ("explore_lite", "pb2025_nav_bringup", "point_lio",
                         "small_gicp_relocalization", "livox_ros_driver2"):
            self.assertIn(required, names)

    def test_sentry_lidar_mount_relative_to_ground(self):
        path = ROOT / "src/pb2025_robot_description/resource/xmacro/pb2025_sentry_robot.sdf.xmacro"
        model = ET.parse(path).getroot()
        chassis_height = float(next(item.get("value") for item in model.iter("xmacro_define_value")
                                    if item.get("name") == "chassis_height"))
        lidar = next(item for item in model.iter("xmacro_block") if item.get("name") == "livox")
        self.assertEqual(lidar.get("parent"), "chassis")
        x, y, z, roll, pitch, yaw = map(float, lidar.get("pose").split())
        self.assertAlmostEqual(x, -0.07)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(chassis_height + z, 0.52)
        self.assertAlmostEqual(roll, 3.14)
        self.assertAlmostEqual(pitch, 0.0)
        self.assertAlmostEqual(yaw, 0.0)


if __name__ == "__main__":
    unittest.main()
