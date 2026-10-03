#!/usr/bin/env python3
import importlib.util
import math
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('lio_external_pose', str(Path(__file__).resolve().parents[1] / 'scripts/lio_external_pose.py'))
node = importlib.util.module_from_spec(spec)
spec.loader.exec_module(node)


class BodyExtrinsicTest(unittest.TestCase):
    def test_level_mount(self):
        p, q = node.body_position((1, 2, 3), (0, 0, 0, 1), (0, 0, -.05))
        self.assertEqual(p, [1, 2, 2.95])

    def test_rotated_mount(self):
        s = math.sqrt(.5)
        p, q = node.body_position((1, 2, 3), (0, s, 0, s), (0, 0, -.05))
        for got, want in zip(p, (.95, 2, 3)):
            self.assertAlmostEqual(got, want)

    def test_bad_orientation(self):
        for q in ((0, 0, 0, 0), (0, 0, 0, float('nan'))):
            with self.assertRaises(ValueError):
                node.body_position((0, 0, 0), q, (0, 0, -.05))


if __name__ == '__main__':
    unittest.main()
