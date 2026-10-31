"""Exercise the actual PGM matching methods without ROS or robot hardware."""

import ast
import math
from pathlib import Path
import types
import unittest

import numpy as np
from scipy import ndimage, optimize, signal, spatial


SCRIPT = (Path(__file__).resolve().parents[1] /
          "src/pb2025_sentry_nav/pb2025_sentry_nav/scripts/rm071_pgm_initial_pose.py")


def load_matching_methods():
    tree = ast.parse(SCRIPT.read_text(encoding="utf-8"), filename=str(SCRIPT))
    functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                 and node.name in ("normalize_angle", "rotate")]
    original = next(node for node in tree.body if isinstance(node, ast.ClassDef)
                    and node.name == "PgmInitialPose")
    names = {"_select_vertical_structure", "_occupied_points", "_rasterize",
             "_coarse_candidates", "_refine", "_solve_alignment"}
    methods = [node for node in original.body if isinstance(node, ast.FunctionDef)
               and node.name in names]
    cls = ast.ClassDef(name="MatchingMethods", bases=[], keywords=[], body=methods,
                       decorator_list=[])
    module = ast.fix_missing_locations(ast.Module(body=functions + [cls], type_ignores=[]))
    namespace = {"math": math, "np": np, "ndimage": ndimage, "optimize": optimize,
                 "signal": signal, "spatial": spatial}
    exec(compile(module, str(SCRIPT), "exec"), namespace)
    return namespace


class RelocalizationMathTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.impl = load_matching_methods()

    def matcher(self):
        node = self.impl["MatchingMethods"]()
        node.yaw_step_deg = 15.0
        node.min_vertical_cells = 50
        node.min_within_20_ratio = 0.75
        node.max_median_distance = 0.10
        node.reject_ambiguous = True
        node.ambiguity_within_ratio_margin = 0.015
        node.ambiguity_median_margin = 0.015
        node.ambiguity_translation_separation = 0.50
        node.ambiguity_yaw_separation = math.radians(12.0)
        node.get_logger = lambda: types.SimpleNamespace(info=lambda message: None)
        node.map_yaml = "unused-in-synthetic-test"
        return node

    def test_vertical_structure_requires_multiple_heights(self):
        node = self.matcher()
        points = np.array([[0.02, 0.02, z] for z in (0.0, 0.2, 0.4)] +
                          [[1.02, 0.02, z] for z in (0.0, 0.01, 0.02)])
        selected = node._select_vertical_structure(points)
        self.assertEqual(selected.shape, (1, 2))
        np.testing.assert_allclose(selected[0], [0.04, 0.04])

    def test_asymmetric_map_alignment_with_large_yaw(self):
        node = self.matcher()
        rng = np.random.default_rng(31)
        occupied = np.zeros((65, 72), dtype=bool)
        selected = rng.choice(occupied.size, size=350, replace=False)
        occupied.flat[selected] = True
        resolution = 0.10
        origin = np.array([-3.2, -2.7, 0.0])
        node._load_map = lambda _: (occupied, resolution, origin)
        target = node._occupied_points(occupied, resolution, origin)
        expected_yaw = math.radians(165.0)
        expected_translation = np.array([1.1, -0.7])
        scan = self.impl["rotate"](target - expected_translation, -expected_yaw)
        points = np.vstack([np.column_stack((scan, np.full(len(scan), z)))
                            for z in (0.0, 0.2, 0.4)])
        result, count = node._solve_alignment(points)
        self.assertGreaterEqual(count, 180)
        self.assertLess(abs(self.impl["normalize_angle"](result["yaw"] - expected_yaw)),
                        math.radians(5.0))
        self.assertLess(np.linalg.norm(result["translation"] - expected_translation), 0.20)
        self.assertGreater(result["within_20"], 0.90)

    def test_ambiguous_candidates_are_rejected(self):
        node = self.matcher()
        node._load_map = lambda _: (np.ones((3, 3), dtype=bool), 1.0, np.zeros(3))
        node._select_vertical_structure = lambda _: np.zeros((200, 2))
        node._coarse_candidates = lambda *args: [0, 1]
        node._refine = lambda source, target, candidate: {
            "yaw": candidate * math.pi, "translation": np.array([candidate * 2.0, 0.0]),
            "within_20": 0.95, "median": 0.03, "objective": 0.001,
        }
        with self.assertRaisesRegex(RuntimeError, "ambiguous"):
            node._solve_alignment(np.zeros((1, 3)))


if __name__ == "__main__":
    unittest.main()
