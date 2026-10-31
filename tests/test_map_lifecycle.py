import importlib.util
import json
import math
from pathlib import Path
import struct
import tempfile
import types
import unittest

import cv2
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


recorder = load_module("map_recorder", ROOT /
                       "src/pb2025_sentry_nav/pb2025_sentry_nav/scripts/rm071_map_recorder.py")
manifest = load_module("map_manifest", ROOT / "tools/map_manifest.py")


def grid():
    orientation = types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(0.2), w=math.cos(0.2))
    position = types.SimpleNamespace(x=-2.0, y=1.5)
    origin = types.SimpleNamespace(position=position, orientation=orientation)
    info = types.SimpleNamespace(width=8, height=8, resolution=0.05, origin=origin)
    data = [0] * 64
    data[0] = 100
    data[8] = -1
    return types.SimpleNamespace(info=info, data=data)


def pcd(path, points=1000):
    header = ("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n"
              "TYPE F F F F\nCOUNT 1 1 1 1\n"
              f"WIDTH {points}\nHEIGHT 1\nPOINTS {points}\nDATA binary\n")
    path.write_bytes(header.encode("ascii") + struct.pack("<ffff", 1.0, 2.0, 3.0, 0.0) * points)


class MapLifecycleTest(unittest.TestCase):
    def test_save_preserves_origin_and_flips_pgm_rows(self):
        with tempfile.TemporaryDirectory() as directory:
            pgm, metadata_path = recorder.save_occupancy_grid(grid(), Path(directory) / "map.yaml")
            metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
            image = cv2.imread(str(pgm), cv2.IMREAD_GRAYSCALE)
            self.assertEqual(image.shape, (8, 8))
            self.assertEqual(int(image[-1, 0]), 0)
            self.assertEqual(int(image[-2, 0]), 127)
            self.assertEqual(int(image[0, 0]), 254)
            self.assertEqual(metadata["image"], "map.pgm")
            self.assertAlmostEqual(metadata["origin"][2], 0.4)
            self.assertAlmostEqual(metadata["resolution"], 0.05)
            unknown_probability = (255 - int(image[-2, 0])) / 255.0
            self.assertGreater(unknown_probability, metadata["free_thresh"])
            self.assertLess(unknown_probability, metadata["occupied_thresh"])

    def test_invalid_grid_is_not_written(self):
        with tempfile.TemporaryDirectory() as directory:
            message = grid()
            message.info.resolution = float("nan")
            path = Path(directory) / "map.yaml"
            with self.assertRaises(ValueError):
                recorder.save_occupancy_grid(message, path)
            self.assertFalse(path.exists())

    def test_latest_map_is_updated_only_after_complete_pcd(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first = root / "first"
            first.mkdir()
            recorder.save_occupancy_grid(grid(), first / "map.yaml")
            pcd(first / "map.pcd")
            latest = root / "latest.json"
            manifest.record_map(latest, first / "map.yaml", first / "map.pcd")
            previous = latest.read_bytes()

            second = root / "second"
            second.mkdir()
            recorder.save_occupancy_grid(grid(), second / "map.yaml")
            pcd(second / "map.pcd")
            with (second / "map.pcd").open("r+b") as stream:
                stream.truncate((second / "map.pcd").stat().st_size - 4)
            with self.assertRaisesRegex(ValueError, "incomplete"):
                manifest.record_map(latest, second / "map.yaml", second / "map.pcd")
            self.assertEqual(latest.read_bytes(), previous)
            selected = manifest.select_map(latest)
            self.assertEqual(selected, ((first / "map.yaml").resolve(), (first / "map.pcd").resolve()))
            self.assertEqual(json.loads(previous)["pcd_points"], 1000)

    def test_tampered_map_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            recorder.save_occupancy_grid(grid(), root / "map.yaml")
            pcd(root / "map.pcd")
            latest = root / "latest.json"
            manifest.record_map(latest, root / "map.yaml", root / "map.pcd")
            image = cv2.imread(str(root / "map.pgm"), cv2.IMREAD_GRAYSCALE)
            image[0, 0] = 0
            self.assertTrue(cv2.imwrite(str(root / "map.pgm"), image))
            with self.assertRaisesRegex(ValueError, "changed"):
                manifest.select_map(latest)


if __name__ == "__main__":
    unittest.main()
