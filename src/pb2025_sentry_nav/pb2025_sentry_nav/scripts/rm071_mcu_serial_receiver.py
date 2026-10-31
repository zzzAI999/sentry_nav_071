#!/usr/bin/env python3

import math
import time
import struct

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

try:
    import serial
except ImportError:
    serial = None


class McuSerialReceiver(Node):
    CRC16_INIT = 0xFFFF
    W_CRC_TABLE = (
        0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
        0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
        0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
        0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
        0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
        0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
        0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
        0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
        0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
        0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
        0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
        0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
        0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
        0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
        0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
        0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
        0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
        0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
        0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
        0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
        0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
        0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
        0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
        0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
        0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
        0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
        0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
        0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
        0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
        0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
        0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
        0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78,
    )

    def __init__(self) -> None:
        super().__init__("rm071_mcu_serial_receiver")

        self.declare_parameter("port", "/dev/serial_port")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("status_topic", "/mcu/status")
        self.declare_parameter("command_topic", "/app/nav_command")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("effective_cmd_vel_topic", "/mcu/cmd_vel_effective")
        self.declare_parameter("read_mode", "line")
        self.declare_parameter("line_numeric_mode", "command")
        self.declare_parameter("reconnect_interval_sec", 1.0)
        self.declare_parameter("cmd_send_rate_hz", 50.0)
        self.declare_parameter("shutdown_stop_frame_count", 8)
        self.declare_parameter("shutdown_stop_frame_interval_sec", 0.02)
        self.declare_parameter("exclusive", False)
        self.declare_parameter("cmd_vel_timeout_sec", 0.5)
        self.declare_parameter("deadzone_compensation_enabled", True)
        self.declare_parameter("linear_deadzone_speed", 0.3)
        self.declare_parameter("angular_deadzone_speed", 0.1)

        self.port = str(self.get_parameter("port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.topic = str(self.get_parameter("status_topic").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.effective_cmd_vel_topic = str(
            self.get_parameter("effective_cmd_vel_topic").value
        )
        self.read_mode = str(self.get_parameter("read_mode").value).lower()
        self.line_numeric_mode = str(self.get_parameter("line_numeric_mode").value).lower()
        if self.line_numeric_mode not in ("command", "status", "both"):
            self.get_logger().warn(
                f"unknown line_numeric_mode='{self.line_numeric_mode}', fallback to 'command'"
            )
            self.line_numeric_mode = "command"
        self.reconnect_interval_sec = float(
            self.get_parameter("reconnect_interval_sec").value
        )
        self.cmd_send_rate_hz = float(self.get_parameter("cmd_send_rate_hz").value)
        if self.cmd_send_rate_hz <= 0.0:
            self.get_logger().warn(
                f"cmd_send_rate_hz={self.cmd_send_rate_hz} is invalid, fallback to 50.0"
            )
            self.cmd_send_rate_hz = 50.0
        self.shutdown_stop_frame_count = int(
            self.get_parameter("shutdown_stop_frame_count").value
        )
        self.shutdown_stop_frame_interval_sec = float(
            self.get_parameter("shutdown_stop_frame_interval_sec").value
        )
        self.exclusive = bool(self.get_parameter("exclusive").value)
        self.cmd_vel_timeout_sec = max(
            0.0, float(self.get_parameter("cmd_vel_timeout_sec").value)
        )
        self.deadzone_compensation_enabled = bool(
            self.get_parameter("deadzone_compensation_enabled").value
        )
        self.linear_deadzone_speed = max(
            0.0, float(self.get_parameter("linear_deadzone_speed").value)
        )
        self.angular_deadzone_speed = max(
            0.0, float(self.get_parameter("angular_deadzone_speed").value)
        )

        self.pub = self.create_publisher(Int32, self.topic, 10)
        self.command_pub = self.create_publisher(String, self.command_topic, 10)
        self.effective_cmd_pub = self.create_publisher(
            Twist, self.effective_cmd_vel_topic, 10
        )
        self.ser = None
        self.rx_buffer = bytearray()
        self.last_open_attempt = 0.0
        self.last_read_error_log = 0.0
        self.last_write_error_log = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.last_cmd_time = time.monotonic()

        self.sub_cmd = self.create_subscription(
            Twist, self.cmd_vel_topic, self._cmd_callback, 10
        )

        if serial is None:
            self.get_logger().error(
                "pyserial not found. Install with: sudo apt install python3-serial"
            )
        else:
            self._open_serial()

        self.timer = self.create_timer(0.01, self._read_once)
        self.cmd_timer = self.create_timer(
            1.0 / self.cmd_send_rate_hz, self._send_current_cmd
        )
        self.get_logger().info(
            f"mcu_serial_receiver started: port={self.port}, baud={self.baudrate}, mode={self.read_mode}, "
            f"line_numeric_mode={self.line_numeric_mode}, status_pub={self.topic}, "
            f"command_pub={self.command_topic}, cmd_sub={self.cmd_vel_topic}, "
            f"effective_cmd_pub={self.effective_cmd_vel_topic}, "
            f"cmd_send_rate={self.cmd_send_rate_hz:.1f}Hz, "
            f"cmd_timeout={self.cmd_vel_timeout_sec:.2f}s, "
            f"deadzone_compensation={self.deadzone_compensation_enabled}, "
            f"linear_floor={self.linear_deadzone_speed:.2f}, "
            f"angular_floor={self.angular_deadzone_speed:.2f}"
        )

    def _close_serial(self) -> None:
        if self.ser is None:
            return
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None

    def _open_serial(self) -> None:
        self.last_open_attempt = time.monotonic()
        try:
            kwargs = dict(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            if self.exclusive:
                # Linux 下可选独占打开，避免多进程同时占用同一串口。
                try:
                    self.ser = serial.Serial(exclusive=True, **kwargs)
                except TypeError:
                    self.get_logger().warn(
                        "pyserial does not support 'exclusive', fallback to normal open"
                    )
                    self.ser = serial.Serial(**kwargs)
            else:
                self.ser = serial.Serial(**kwargs)

            self.rx_buffer.clear()
            self.get_logger().info("serial opened successfully")
        except Exception as ex:
            self.ser = None
            self.get_logger().error(f"failed to open serial: {ex}")

    def _try_reopen_if_needed(self) -> None:
        if self.ser is not None or serial is None:
            return
        now = time.monotonic()
        if now - self.last_open_attempt < self.reconnect_interval_sec:
            return
        self._open_serial()

    def _handle_read_error(self, ex: Exception) -> None:
        now = time.monotonic()
        if now - self.last_read_error_log >= 2.0:
            self.get_logger().warn(f"serial read failed: {ex}; will reconnect")
            self.last_read_error_log = now
        self._close_serial()

    def _handle_write_error(self, ex: Exception) -> None:
        now = time.monotonic()
        if now - self.last_write_error_log >= 2.0:
            self.get_logger().warn(f"serial write failed: {ex}; will reconnect")
            self.last_write_error_log = now
        self._close_serial()

    def _cmd_callback(self, msg: Twist) -> None:
        self.current_vx = float(msg.linear.x)
        self.current_vy = float(msg.linear.y)
        self.current_wz = float(msg.angular.z)
        self.last_cmd_time = time.monotonic()

    def _compensate_deadzone(
        self, vx: float, vy: float, wz: float
    ) -> tuple[float, float, float]:
        if not self.deadzone_compensation_enabled:
            return vx, vy, wz

        linear_speed = math.hypot(vx, vy)
        if linear_speed > 1.0e-6:
            if linear_speed < self.linear_deadzone_speed:
                scale = self.linear_deadzone_speed / linear_speed
                vx *= scale
                vy *= scale

        if 1.0e-6 < abs(wz) < self.angular_deadzone_speed:
            wz = math.copysign(self.angular_deadzone_speed, wz)

        return vx, vy, wz

    def _effective_current_cmd(self) -> tuple[float, float, float]:
        if (
            self.cmd_vel_timeout_sec > 0.0
            and time.monotonic() - self.last_cmd_time > self.cmd_vel_timeout_sec
        ):
            return 0.0, 0.0, 0.0
        return self._compensate_deadzone(
            self.current_vx, self.current_vy, self.current_wz
        )

    @classmethod
    def _crc16(cls, data: bytes) -> int:
        crc = cls.CRC16_INIT
        for ch_data in data:
            crc = (crc >> 8) ^ cls.W_CRC_TABLE[(crc ^ ch_data) & 0x00FF]
        return crc & 0xFFFF

    def _build_cmd_frame(self, vx=None, vy=None, wz=None) -> bytes:
        # 与既有 MCU 协议保持一致的符号处理与帧格式。
        vx = self.current_vx if vx is None else float(vx)
        vy = self.current_vy if vy is None else float(vy)
        wz = self.current_wz if wz is None else float(wz)

        buffer = bytearray(15)
        buffer[0] = 0xA5
        struct.pack_into("<f", buffer, 1, vx)
        struct.pack_into("<f", buffer, 5, vy)
        struct.pack_into("<f", buffer, 9, wz)

        crc = self._crc16(buffer[:-2])
        buffer[-2] = crc & 0xFF
        buffer[-1] = (crc >> 8) & 0xFF
        return bytes(buffer)

    def _send_current_cmd(self) -> None:
        if self.ser is None:
            return
        vx, vy, wz = self._effective_current_cmd()
        frame = self._build_cmd_frame(vx, vy, wz)
        try:
            written = self.ser.write(frame)
            if written != len(frame):
                self.get_logger().warn(
                    f"incomplete serial write: {written}/{len(frame)}"
                )
            effective_cmd = Twist()
            effective_cmd.linear.x = vx
            effective_cmd.linear.y = vy
            effective_cmd.angular.z = wz
            self.effective_cmd_pub.publish(effective_cmd)
        except Exception as ex:
            self._handle_write_error(ex)

    def _send_stop_cmd(self) -> None:
        if self.ser is None:
            return

        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        frame = self._build_cmd_frame(0.0, 0.0, 0.0)
        for _ in range(max(1, self.shutdown_stop_frame_count)):
            try:
                self.ser.write(frame)
                try:
                    self.ser.flush()
                except Exception:
                    pass
            except Exception as ex:
                self._handle_write_error(ex)
                break
            time.sleep(max(0.0, self.shutdown_stop_frame_interval_sec))

    def _publish_status(self, value: int) -> None:
        msg = Int32()
        msg.data = int(value)
        self.pub.publish(msg)

    def _publish_command(self, text: str) -> None:
        command = text.strip()
        if not command:
            return
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f"serial command received: '{command}'")

    def _handle_line_text(self, text: str) -> None:
        if self.line_numeric_mode in ("command", "both"):
            self._publish_command(text)

        try:
            status = int(text, 16)
        except ValueError:
            if self.line_numeric_mode == "status":
                self._publish_command(text)
            return

        if self.line_numeric_mode in ("status", "both"):
            self._publish_status(status)

    def _read_once(self) -> None:
        if self.ser is None:
            self._try_reopen_if_needed()
            return

        try:
            pending = self.ser.in_waiting
            if pending <= 0:
                return
            data = self.ser.read(pending)
            if not data:
                return
        except Exception as ex:
            self._handle_read_error(ex)
            return

        if self.read_mode == "binary":
            self._handle_binary(data)
        else:
            self._handle_line(data)

    def _handle_line(self, data: bytes) -> None:
        self.rx_buffer.extend(data)
        while b"\n" in self.rx_buffer:
            line, _, rest = self.rx_buffer.partition(b"\n")
            self.rx_buffer = bytearray(rest)
            text = line.decode("utf-8", errors="ignore").strip()
            if not text:
                continue
            self._handle_line_text(text)

    def _handle_binary(self, data: bytes) -> None:
        self.rx_buffer.extend(data)
        while len(self.rx_buffer) >= 3:
            if self.rx_buffer[0] != 0x5A:
                del self.rx_buffer[0]
                continue

            data_high = self.rx_buffer[1]
            data_low = self.rx_buffer[2]
            status = (data_low << 8) | data_high

            del self.rx_buffer[:3]
            self._publish_status(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = McuSerialReceiver()
    try:
        rclpy.spin(node)
    finally:
        node._send_stop_cmd()
        node._close_serial()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
