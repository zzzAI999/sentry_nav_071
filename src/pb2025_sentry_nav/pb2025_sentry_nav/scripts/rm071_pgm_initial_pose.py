#!/usr/bin/env python3

import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy import ndimage, optimize, signal, spatial
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def rotate(points, yaw):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    matrix = np.array([[cosine, -sine], [sine, cosine]], dtype=np.float64)
    return points @ matrix.T


class PgmInitialPose(Node):
    def __init__(self):
        super().__init__("rm071_pgm_initial_pose")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("input_topic", "/registered_scan")
        self.declare_parameter("robot_base_frame", "gimbal_yaw")
        self.declare_parameter("required_messages", 30)
        self.declare_parameter("wait_timeout_sec", 15.0)
        self.declare_parameter("yaw_step_deg", 3.0)
        self.declare_parameter("min_vertical_cells", 180)
        self.declare_parameter("min_within_20_ratio", 0.75)
        self.declare_parameter("max_median_distance", 0.10)
        self.declare_parameter("reject_ambiguous", True)
        self.declare_parameter("ambiguity_within_ratio_margin", 0.015)
        self.declare_parameter("ambiguity_median_margin", 0.015)
        self.declare_parameter("ambiguity_translation_separation", 0.50)
        self.declare_parameter("ambiguity_yaw_separation_deg", 12.0)
        self.declare_parameter("publish_initial_pose", True)

        self.map_yaml = os.path.expanduser(
            str(self.get_parameter("map_yaml").value)
        )
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.robot_base_frame = str(
            self.get_parameter("robot_base_frame").value
        )
        self.required_messages = max(
            5, int(self.get_parameter("required_messages").value)
        )
        self.wait_timeout_sec = max(
            3.0, float(self.get_parameter("wait_timeout_sec").value)
        )
        self.yaw_step_deg = max(
            1.0, float(self.get_parameter("yaw_step_deg").value)
        )
        self.min_vertical_cells = max(
            50, int(self.get_parameter("min_vertical_cells").value)
        )
        self.min_within_20_ratio = float(
            self.get_parameter("min_within_20_ratio").value
        )
        self.max_median_distance = float(
            self.get_parameter("max_median_distance").value
        )
        self.reject_ambiguous = bool(
            self.get_parameter("reject_ambiguous").value
        )
        self.ambiguity_within_ratio_margin = max(
            0.0,
            float(
                self.get_parameter(
                    "ambiguity_within_ratio_margin"
                ).value
            ),
        )
        self.ambiguity_median_margin = max(
            0.0,
            float(self.get_parameter("ambiguity_median_margin").value),
        )
        self.ambiguity_translation_separation = max(
            0.0,
            float(
                self.get_parameter(
                    "ambiguity_translation_separation"
                ).value
            ),
        )
        self.ambiguity_yaw_separation = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        "ambiguity_yaw_separation_deg"
                    ).value
                ),
            )
        )
        self.publish_initial_pose = bool(
            self.get_parameter("publish_initial_pose").value
        )

        if not self.map_yaml or not os.path.isfile(self.map_yaml):
            raise RuntimeError(f"Map yaml does not exist: {self.map_yaml}")

        self.clouds = []
        self.scan_frame = ""
        self.solve_started = False
        self.finished = False
        self.succeeded = False
        self.started_at = time.monotonic()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.create_subscription(
            PointCloud2, self.input_topic, self._on_cloud, 10
        )
        self.create_timer(0.2, self._check_timeout)
        self.get_logger().info(
            f"Waiting for {self.required_messages} scans on {self.input_topic}"
        )

    def _on_cloud(self, msg):
        if self.solve_started:
            return
        points = point_cloud2.read_points_numpy(
            msg, field_names=["x", "y", "z"], skip_nans=True
        )
        points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        if len(points) == 0:
            return
        self.clouds.append(points)
        self.scan_frame = msg.header.frame_id
        if len(self.clouds) >= self.required_messages:
            self.solve_started = True
            threading.Thread(target=self._solve, daemon=True).start()

    def _check_timeout(self):
        if self.finished or self.solve_started:
            return
        if time.monotonic() - self.started_at <= self.wait_timeout_sec:
            return
        self.get_logger().error(
            f"Timed out waiting for point cloud on {self.input_topic}"
        )
        self.finished = True
        rclpy.shutdown()

    @staticmethod
    def _load_map(yaml_path):
        with open(yaml_path, "r", encoding="utf-8") as stream:
            metadata = yaml.safe_load(stream)
        image_path = os.path.expanduser(str(metadata["image"]))
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(yaml_path), image_path)
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise RuntimeError(f"Could not read PGM map: {image_path}")
        negate = int(metadata.get("negate", 0))
        occupied_threshold = float(metadata.get("occupied_thresh", 0.65))
        occupancy_probability = (
            image.astype(np.float32) / 255.0
            if negate
            else (255.0 - image.astype(np.float32)) / 255.0
        )
        occupied = np.flipud(
            occupancy_probability >= occupied_threshold
        )
        resolution = float(metadata["resolution"])
        if resolution <= 0.0:
            raise RuntimeError(f"Invalid map resolution: {resolution}")
        origin_values = list(metadata["origin"])
        if len(origin_values) < 3:
            raise RuntimeError("Map origin must contain x, y and yaw")
        origin = np.asarray(origin_values[:3], dtype=np.float64)
        return occupied, resolution, origin

    @staticmethod
    def _select_vertical_structure(points):
        cell_size = 0.08
        finite = points[np.isfinite(points).all(axis=1)]
        indices = np.floor(finite[:, :2] / cell_size).astype(np.int32)
        unique, inverse = np.unique(indices, axis=0, return_inverse=True)
        z_min = np.full(len(unique), np.inf, dtype=np.float32)
        z_max = np.full(len(unique), -np.inf, dtype=np.float32)
        counts = np.zeros(len(unique), dtype=np.int32)
        np.minimum.at(z_min, inverse, finite[:, 2])
        np.maximum.at(z_max, inverse, finite[:, 2])
        np.add.at(counts, inverse, 1)
        keep = ((z_max - z_min) >= 0.25) & (counts >= 3)
        return (unique[keep].astype(np.float64) + 0.5) * cell_size

    @staticmethod
    def _occupied_points(occupied, resolution, origin):
        rows, columns = np.nonzero(occupied)
        local_points = np.column_stack(
            (
                (columns.astype(np.float64) + 0.5) * resolution,
                (rows.astype(np.float64) + 0.5) * resolution,
            )
        )
        return rotate(local_points, float(origin[2])) + origin[:2]

    @staticmethod
    def _rasterize(points, resolution):
        minimum = points.min(axis=0)
        indices = np.floor((points - minimum) / resolution).astype(np.int32)
        width = int(indices[:, 0].max()) + 1
        height = int(indices[:, 1].max()) + 1
        raster = np.zeros((height, width), dtype=np.float32)
        raster[indices[:, 1], indices[:, 0]] = 1.0
        return raster, minimum

    def _coarse_candidates(self, source, occupied, resolution, origin):
        target = ndimage.binary_dilation(
            occupied, iterations=2
        ).astype(np.float32)
        candidates = []
        for yaw_deg in np.arange(-180.0, 180.0, self.yaw_step_deg):
            yaw = math.radians(float(yaw_deg))
            rotated = rotate(source, yaw)
            source_raster, source_minimum = self._rasterize(
                rotated, resolution
            )
            correlation = signal.fftconvolve(
                target, source_raster[::-1, ::-1], mode="full"
            )
            flat_indices = np.argpartition(correlation.ravel(), -3)[-3:]
            for flat_index in flat_indices:
                row, column = np.unravel_index(
                    flat_index, correlation.shape
                )
                offset = np.array(
                    [
                        column - (source_raster.shape[1] - 1),
                        row - (source_raster.shape[0] - 1),
                    ],
                    dtype=np.float64,
                )
                local_translation = (
                    offset * resolution - source_minimum
                )
                translation = (
                    rotate(
                        local_translation.reshape(1, 2),
                        float(origin[2]),
                    )[0]
                    + origin[:2]
                )
                candidates.append(
                    (
                        float(correlation[row, column]),
                        normalize_angle(yaw + float(origin[2])),
                        translation,
                    )
                )
        candidates.sort(key=lambda item: item[0], reverse=True)
        return candidates[:24]

    @staticmethod
    def _refine(source, target_tree, candidate):
        def objective(parameters):
            yaw, tx, ty = parameters
            transformed = rotate(source, yaw) + np.array([tx, ty])
            distances, _ = target_tree.query(
                transformed, k=1, workers=-1
            )
            clipped = np.minimum(distances, 0.60)
            cutoff = np.quantile(clipped, 0.80)
            trimmed = clipped[clipped <= cutoff]
            return float(np.mean(np.square(trimmed)))

        initial = np.array(
            [candidate[1], candidate[2][0], candidate[2][1]]
        )
        result = optimize.minimize(
            objective,
            initial,
            method="Powell",
            options={"maxiter": 100, "xtol": 1e-4, "ftol": 1e-6},
        )
        yaw = normalize_angle(float(result.x[0]))
        translation = np.asarray(result.x[1:3], dtype=np.float64)
        transformed = rotate(source, yaw) + translation
        distances, _ = target_tree.query(
            transformed, k=1, workers=-1
        )
        return {
            "objective": float(result.fun),
            "yaw": yaw,
            "translation": translation,
            "within_20": float(np.mean(distances <= 0.20)),
            "median": float(np.median(distances)),
        }

    def _solve_alignment(self, points):
        occupied, resolution, origin = self._load_map(self.map_yaml)
        source = self._select_vertical_structure(points)
        if len(source) < self.min_vertical_cells:
            raise RuntimeError(
                f"Only {len(source)} vertical cells; "
                f"required {self.min_vertical_cells}"
            )
        target = self._occupied_points(occupied, resolution, origin)
        target_tree = spatial.cKDTree(target)
        coarse = self._coarse_candidates(
            source, occupied, resolution, origin
        )

        refined = []
        for candidate in coarse[:12]:
            result = self._refine(source, target_tree, candidate)
            duplicate = any(
                abs(normalize_angle(result["yaw"] - previous["yaw"]))
                < math.radians(2.0)
                and np.linalg.norm(
                    result["translation"] - previous["translation"]
                )
                < 0.20
                for previous in refined
            )
            if not duplicate:
                refined.append(result)
            if len(refined) >= 6:
                break
        if not refined:
            raise RuntimeError("PGM alignment produced no candidate")
        refined.sort(
            key=lambda item: (
                -item["within_20"],
                item["median"],
                item["objective"],
            )
        )
        best = refined[0]
        if (
            best["within_20"] < self.min_within_20_ratio
            or best["median"] > self.max_median_distance
        ):
            raise RuntimeError(
                "PGM alignment confidence is too low: "
                f"within20={best['within_20']:.3f}, "
                f"median={best['median']:.3f}m"
            )

        candidate_summary = "; ".join(
            "yaw=%.1fdeg xy=(%.2f,%.2f) within20=%.3f median=%.3fm"
            % (
                math.degrees(candidate["yaw"]),
                candidate["translation"][0],
                candidate["translation"][1],
                candidate["within_20"],
                candidate["median"],
            )
            for candidate in refined[:4]
        )
        self.get_logger().info(
            f"PGM alignment candidates: {candidate_summary}"
        )

        if self.reject_ambiguous:
            for alternative in refined[1:]:
                translation_separation = np.linalg.norm(
                    alternative["translation"] - best["translation"]
                )
                yaw_separation = abs(
                    normalize_angle(alternative["yaw"] - best["yaw"])
                )
                is_distinct = (
                    translation_separation
                    >= self.ambiguity_translation_separation
                    or yaw_separation >= self.ambiguity_yaw_separation
                )
                similarly_supported = (
                    best["within_20"] - alternative["within_20"]
                    <= self.ambiguity_within_ratio_margin
                    and alternative["median"] - best["median"]
                    <= self.ambiguity_median_margin
                )
                if is_distinct and similarly_supported:
                    raise RuntimeError(
                        "PGM alignment is ambiguous: best and alternate "
                        "solutions have similar support "
                        f"(best yaw={math.degrees(best['yaw']):.1f}deg, "
                        f"alternate yaw="
                        f"{math.degrees(alternative['yaw']):.1f}deg, "
                        f"translation separation="
                        f"{translation_separation:.2f}m)"
                    )
        return best, len(source)

    def _robot_pose_from_map_to_scan(self, alignment):
        transform = self.tf_buffer.lookup_transform(
            self.scan_frame,
            self.robot_base_frame,
            Time(),
            timeout=Duration(seconds=1.0),
        )
        scan_to_base_yaw = quaternion_yaw(transform.transform.rotation)
        scan_to_base_xy = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
            ]
        )
        map_to_base_xy = (
            rotate(
                scan_to_base_xy.reshape(1, 2), alignment["yaw"]
            )[0]
            + alignment["translation"]
        )
        map_to_base_yaw = normalize_angle(
            alignment["yaw"] + scan_to_base_yaw
        )
        return map_to_base_xy, map_to_base_yaw

    def _solve(self):
        try:
            points = np.concatenate(self.clouds, axis=0)
            alignment, vertical_cells = self._solve_alignment(points)
            position, yaw = self._robot_pose_from_map_to_scan(alignment)
            self.get_logger().warn(
                "PGM initial pose solved: "
                f"x={position[0]:.3f}, y={position[1]:.3f}, "
                f"yaw={math.degrees(yaw):.2f}deg, "
                f"within20={alignment['within_20']:.3f}, "
                f"median={alignment['median']:.3f}m, "
                f"vertical_cells={vertical_cells}"
            )

            if self.publish_initial_pose:
                message = PoseWithCovarianceStamped()
                message.header.frame_id = "map"
                message.pose.pose.position.x = float(position[0])
                message.pose.pose.position.y = float(position[1])
                message.pose.pose.orientation.z = math.sin(yaw * 0.5)
                message.pose.pose.orientation.w = math.cos(yaw * 0.5)
                message.pose.covariance[0] = 0.01
                message.pose.covariance[7] = 0.01
                message.pose.covariance[35] = 0.0025
                for _ in range(3):
                    message.header.stamp = self.get_clock().now().to_msg()
                    self.initial_pose_pub.publish(message)
                    time.sleep(0.4)
            self.succeeded = True
        except Exception as ex:
            self.get_logger().error(f"PGM initial pose failed: {ex}")
        finally:
            self.finished = True
            time.sleep(0.3)
            rclpy.shutdown()


def main():
    rclpy.init()
    node = PgmInitialPose()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
    return 0 if node.succeeded else 1


if __name__ == "__main__":
    sys.exit(main())
