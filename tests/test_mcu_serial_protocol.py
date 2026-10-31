"""Check on-wire velocity commands without opening a physical serial device."""

import ast
import math
from pathlib import Path
import struct
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "src/pb2025_sentry_nav/pb2025_sentry_nav/scripts/rm071_mcu_serial_receiver.py"


def twist():
    return SimpleNamespace(linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                           angular=SimpleNamespace(x=0.0, y=0.0, z=0.0))


def reference_crc(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc


class McuSerialProtocolTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        tree = ast.parse(SCRIPT.read_text(encoding="utf-8"))
        definition = next(item for item in tree.body if isinstance(item, ast.ClassDef))
        scope = {"Node": object, "Twist": twist, "math": math,
                 "time": time, "struct": struct}
        module = ast.fix_missing_locations(ast.Module(body=[definition], type_ignores=[]))
        exec(compile(module, str(SCRIPT), "exec"), scope)
        cls.receiver_type = scope["McuSerialReceiver"]

    def receiver(self):
        node = self.receiver_type.__new__(self.receiver_type)
        node.current_vx, node.current_vy, node.current_wz = 0.12, -0.16, 0.02
        node.last_cmd_time, node.cmd_vel_timeout_sec = 10.0, 0.5
        node.deadzone_compensation_enabled = True
        node.linear_deadzone_speed, node.angular_deadzone_speed = 0.3, 0.1
        node.ser = Mock()
        node.ser.write.side_effect = len
        node.effective_cmd_pub = Mock()
        return node

    def decode(self, frame):
        self.assertEqual(len(frame), 15)
        self.assertEqual(frame[0], 0xA5)
        self.assertEqual(struct.unpack_from("<H", frame, 13)[0], reference_crc(frame[:13]))
        return struct.unpack_from("<fff", frame, 1)

    def test_crc_matches_known_vector(self):
        self.assertEqual(self.receiver_type._crc16(b"123456789"), 0x6F91)

    def test_explicit_velocity_and_zero_override_cached_forward_speed(self):
        node = self.receiver()
        frame = node._build_cmd_frame(1.25, -0.5, 0.125)
        self.assertEqual(self.decode(frame), (1.25, -0.5, 0.125))
        self.assertEqual(self.decode(node._build_cmd_frame(0, 0, 0)), (0.0, 0.0, 0.0))

    def test_timeout_sends_zero_in_all_three_wire_fields(self):
        node = self.receiver()
        with patch.object(time, "monotonic", return_value=11.0):
            node._send_current_cmd()
        self.assertEqual(self.decode(node.ser.write.call_args.args[0]), (0.0, 0.0, 0.0))
        effective = node.effective_cmd_pub.publish.call_args.args[0]
        self.assertEqual((effective.linear.x, effective.linear.y, effective.angular.z),
                         (0.0, 0.0, 0.0))

    def test_deadzone_compensation_matches_the_transmitted_velocity(self):
        node = self.receiver()
        with patch.object(time, "monotonic", return_value=10.1):
            node._send_current_cmd()
        actual = self.decode(node.ser.write.call_args.args[0])
        for value, expected in zip(actual, (0.18, -0.24, 0.1)):
            self.assertAlmostEqual(value, expected, places=6)
        effective = node.effective_cmd_pub.publish.call_args.args[0]
        for value, reported in zip(actual, (effective.linear.x, effective.linear.y, effective.angular.z)):
            self.assertAlmostEqual(value, reported, places=6)

    def test_shutdown_writes_only_stop_frames(self):
        node = self.receiver()
        node.shutdown_stop_frame_count, node.shutdown_stop_frame_interval_sec = 3, 0
        with patch.object(time, "sleep"):
            node._send_stop_cmd()
        self.assertEqual(node.ser.write.call_count, 3)
        for call in node.ser.write.call_args_list:
            self.assertEqual(self.decode(call.args[0]), (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
