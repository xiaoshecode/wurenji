"""Offline regressions: these tests never load ROS or send flight commands."""
import importlib.util
from pathlib import Path
import unittest

import cv2
import numpy as np

spec = importlib.util.spec_from_file_location(
    "circle_detect", Path(__file__).parents[1] / "raspicam_node/src/cv/circle_detect.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class CircleDetectTest(unittest.TestCase):
    def test_full_foreground_is_not_discarded_as_background(self):
        detector = module.circle_detector()
        detector.thresh_img = np.full((100, 100), 255, dtype=np.uint8)
        detector._patch()
        self.assertEqual(detector.num_components, 1)
        self.assertEqual(detector.stats[0, cv2.CC_STAT_AREA], 10000)

    def test_ambiguous_matches_remove_the_correct_edge(self):
        detector = module.circle_detector()
        # Patch 0 matches 1 and 2; neither lower patch matches the other.
        detector.stats = np.array([
            [0, 0, 100, 30, 2100],
            [0, 90, 100, 30, 2100],
            [0, 100, 100, 30, 2100],
        ])
        detector.num_components = 3
        detector._strong_match = lambda first, second: 0.9 if first == 1 else 0.1
        circles = detector._match_patches()
        self.assertEqual(len(circles), 1)
        np.testing.assert_array_equal(circles[0], [60, 50, 60])

    def test_empty_image_has_no_circles(self):
        detector = module.circle_detector()
        self.assertEqual(detector.detect(np.zeros((100, 100, 3), dtype=np.uint8)), [])


if __name__ == "__main__":
    unittest.main()
