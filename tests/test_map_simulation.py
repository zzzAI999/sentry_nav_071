"""Exercise the vendored simulator's motion and map code without a ROS installation."""

import ast
import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / "src/nav2_loopback_sim/nav2_loopback_sim"
BRINGUP = ROOT / "src/pb2025_sentry_nav/pb2025_nav_bringup"


def vector():
    return types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


def quaternion(yaw=0.0):
    return types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def transform():
    return types.SimpleNamespace(translation=vector(), rotation=quaternion())


def twist():
    return types.SimpleNamespace(linear=vector(), angular=vector())


def odometry():
    return types.SimpleNamespace(header=types.SimpleNamespace(), child_frame_id="",
                                 pose=types.SimpleNamespace(pose=types.SimpleNamespace(
                                     position=vector(), orientation=quaternion())),
                                 twist=types.SimpleNamespace(twist=twist()))


def load_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TimePoint(float):
    def to_msg(self):
        return self


class SimulationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        messages = types.ModuleType("geometry_msgs.msg")
        messages.Quaternion, messages.Transform = quaternion, transform
        tf = types.ModuleType("tf_transformations")
        tf.euler_from_quaternion = lambda q: Rotation.from_quat(q).as_euler("xyz")
        tf.quaternion_from_euler = lambda x, y, z: Rotation.from_euler("xyz", [x, y, z]).as_quat()
        tf.quaternion_multiply = lambda a, b: (Rotation.from_quat(a) * Rotation.from_quat(b)).as_quat()
        with patch.dict(sys.modules, {"geometry_msgs": types.ModuleType("geometry_msgs"),
                                     "geometry_msgs.msg": messages, "tf_transformations": tf}):
            cls.utils = load_file("sim_utils", SIM / "utils.py")
        tree = ast.parse((SIM / "loopback_simulator.py").read_text(encoding="utf-8"))
        original = next(node for node in tree.body if isinstance(node, ast.ClassDef))
        names = {"timerCallback", "publishOdometry", "publishReady", "getLaserScan", "mapCallback"}
        body = [node for node in original.body if isinstance(node, ast.FunctionDef) and node.name in names]
        module = ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(
            name="SimulatorMethods", bases=[], keywords=[], body=body, decorator_list=[])], type_ignores=[]))
        scope = {"math": math, "Duration": lambda seconds: seconds, "Twist": twist, "Odometry": odometry,
                 "Bool": types.SimpleNamespace, "raycastMap": cls.utils.raycastMap,
                 "addYawToQuat": cls.utils.addYawToQuat, "tf_transformations": tf}
        exec(compile(module, str(SIM / "loopback_simulator.py"), "exec"), scope)
        cls.simulator_type = scope["SimulatorMethods"]

    def simulator(self):
        node = self.simulator_type()
        node.get_logger = lambda: types.SimpleNamespace(error=lambda _msg: None)
        node.info = lambda _msg: None
        node.get_clock = lambda: types.SimpleNamespace(now=lambda: TimePoint(5.0))
        node.curr_cmd_vel_time = 5.0
        node.curr_cmd_vel = twist()
        node.update_dur = 0.1
        node.mat_base_to_laser = np.eye(4)
        node.t_map_to_odom = types.SimpleNamespace(transform=transform())
        node.t_odom_to_base_link = types.SimpleNamespace(transform=transform())
        node.publishTransforms = lambda *_args: None
        node.odom_frame_id, node.base_frame_id = "odom", "base_footprint"
        node.odom_samples = []
        node.odom_pub = types.SimpleNamespace(publish=node.odom_samples.append)
        return node

    def test_omnidirectional_motion_is_integrated_in_body_coordinates(self):
        node = self.simulator()
        node.t_odom_to_base_link.transform.rotation = quaternion(math.pi / 2)
        node.curr_cmd_vel.linear.x, node.curr_cmd_vel.linear.y = 1.0, 0.5
        node.curr_cmd_vel.angular.z = 0.1
        node.timerCallback()
        pose = node.t_odom_to_base_link.transform
        self.assertAlmostEqual(pose.translation.x, -0.05)
        self.assertAlmostEqual(pose.translation.y, 0.1)
        self.assertEqual(node.odom_samples[-1].child_frame_id, "base_footprint")
        self.assertEqual(node.odom_samples[-1].header.frame_id, "odom")

    def test_expired_command_stops_motion_and_keeps_zero_odometry_alive(self):
        node = self.simulator()
        node.curr_cmd_vel.linear.x = 1.0
        node.curr_cmd_vel_time = 3.0
        node.timerCallback()
        self.assertEqual(node.t_odom_to_base_link.transform.translation.x, 0.0)
        self.assertEqual(node.odom_samples[-1].twist.twist.linear.x, 0.0)

    def test_world_to_grid_honours_origin_translation_and_rotation(self):
        grid = types.SimpleNamespace(info=types.SimpleNamespace(resolution=1.0, origin=types.SimpleNamespace(
            position=types.SimpleNamespace(x=10.0, y=20.0), orientation=quaternion(math.pi / 2))))
        self.assertEqual(self.utils.worldToMap(9.5, 20.5, grid), (0, 0))

    def test_scan_hits_a_wall_from_a_map_edge_cell(self):
        node = self.simulator()
        info = types.SimpleNamespace(width=10, height=10, resolution=1.0,
                                     origin=types.SimpleNamespace(position=vector(), orientation=quaternion()))
        node.map = types.SimpleNamespace(info=info, data=[0] * 100)
        for y in range(10):
            node.map.data[y * 10 + 7] = 100
        node.initial_pose, node.use_inf = object(), True
        node.getLaserPose = lambda: (0.5, 3.5, 0.0)
        node.scan_msg = types.SimpleNamespace(angle_min=0.0, angle_increment=math.pi / 2,
                                              range_max=12.0, ranges=[0.0] * 4)
        node.getLaserScan(4)
        self.assertEqual(node.scan_msg.ranges[0], 6.5)
        self.assertTrue(math.isinf(node.scan_msg.ranges[2]))

    def test_steep_rays_hit_every_crossed_wall_cell(self):
        node = self.simulator()
        info = types.SimpleNamespace(width=100, height=100, resolution=0.05,
                                     origin=types.SimpleNamespace(position=vector(), orientation=quaternion()))
        node.map = types.SimpleNamespace(info=info, data=[0] * 10000)
        for x in range(100):
            node.map.data[60 * 100 + x] = 100
            node.map.data[40 * 100 + x] = 100
        node.initial_pose, node.use_inf = object(), True
        node.getLaserPose = lambda: (2.525, 2.525, 0.0)
        node.scan_msg = types.SimpleNamespace(angle_min=math.radians(89),
                                              angle_increment=math.radians(-178),
                                              range_max=10.0, ranges=[0.0] * 2)
        node.getLaserScan(2)
        for distance in node.scan_msg.ranges:
            self.assertAlmostEqual(distance, 0.475 / math.sin(math.radians(89)))

    def test_raycast_honours_rotated_map_and_maximum_range(self):
        info = types.SimpleNamespace(width=10, height=10, resolution=1.0,
                                     origin=types.SimpleNamespace(
                                         position=types.SimpleNamespace(x=10.0, y=20.0),
                                         orientation=quaternion(math.pi / 2)))
        grid = types.SimpleNamespace(info=info, data=[0] * 100)
        for y in range(10):
            grid.data[y * 10 + 7] = 100
        self.assertAlmostEqual(self.utils.raycastMap(6.5, 20.5, math.pi / 2, 12.0, grid), 6.5)
        self.assertTrue(math.isinf(self.utils.raycastMap(6.5, 20.5, math.pi / 2, 6.0, grid)))

    def test_readiness_waits_for_initial_pose_map_and_scan_tf(self):
        node = self.simulator()
        node.initial_pose, node.map, node.mat_base_to_laser = None, None, None
        sent = []
        node.ready_pub = types.SimpleNamespace(publish=sent.append)
        node.publishReady()
        node.initial_pose = object()
        node.mapCallback(types.SimpleNamespace(info=types.SimpleNamespace(
            resolution=0.05, width=2, height=2), data=[0] * 4))
        node.publishReady()
        node.mat_base_to_laser = np.eye(4)
        node.publishReady()
        self.assertEqual([item.data for item in sent], [False, False, True])

    def test_simulation_configuration_keeps_real_geometry_and_isolates_velocity(self):
        real = yaml.safe_load((BRINGUP / "config/reality/nav2_params.yaml").read_text(encoding="utf-8"))
        overlay = yaml.safe_load((BRINGUP / "config/simulation/nav2_overlay.yaml").read_text(encoding="utf-8"))
        self.assertEqual(overlay["fake_vel_transform"]["ros__parameters"]["output_cmd_vel_topic"], "sim/cmd_vel")
        for key in ("local_costmap", "global_costmap"):
            settings = overlay[key][key]["ros__parameters"]
            self.assertEqual(settings["static_layer"]["map_topic"], "/map")
            self.assertEqual(settings["obstacle_layer"]["scan"]["topic"], "/obstacle_scan")
            self.assertNotIn("intensity_voxel_layer", settings["plugins"])
            self.assertNotIn("robot_radius", settings)
            self.assertEqual(real[key][key]["ros__parameters"]["robot_radius"], 0.32)


if __name__ == "__main__":
    unittest.main()
