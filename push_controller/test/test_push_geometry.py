import importlib.util
import math
from pathlib import Path
import unittest

import numpy as np


geometry_file = (
    Path(__file__).resolve().parents[1] / "push_controller" / "geometry.py"
)
spec = importlib.util.spec_from_file_location("push_geometry", geometry_file)
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)

build_push_observation = geometry.build_push_observation
clip_push_action = geometry.clip_push_action
is_fresh = geometry.is_fresh
quat_to_yaw = geometry.quat_to_yaw


class PushGeometryTest(unittest.TestCase):

    def test_quat_to_yaw_reads_identity(self):
        self.assertAlmostEqual(quat_to_yaw(0.0, 0.0, 0.0, 1.0), 0.0)

    def test_clip_push_action_uses_training_bounds(self):
        action = clip_push_action(np.array([-1.0, 2.0, 1.0], dtype=np.float32))

        np.testing.assert_allclose(action, [-0.5, 1.0, 0.5])

    def test_build_push_observation_matches_training_shape(self):
        obs = build_push_observation(
            robot_position=np.array([0.0, 0.0, 0.0]),
            robot_yaw=0.0,
            projected_gravity=np.array([0.0, 0.0, -1.0]),
            base_ang_vel=np.array([0.1, 0.2, 0.3]),
            box_position=np.array([1.0, 0.5, 0.12]),
            box_yaw=math.pi / 2.0,
            goal_position=np.array([2.0, 0.5, 0.12]),
            goal_yaw=math.pi / 2.0,
        )

        self.assertEqual(obs.shape, (16,))
        np.testing.assert_allclose(obs[:3], [0.1, 0.2, 0.3])
        np.testing.assert_allclose(obs[3:6], [0.0, 0.0, -1.0])
        np.testing.assert_allclose(obs[6:9], [1.0, 0.5, 0.12])
        np.testing.assert_allclose(obs[9:11], [1.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(obs[11:14], [0.0, -1.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(obs[14:16], [0.0, 1.0], atol=1e-6)

    def test_is_fresh_rejects_stale_timestamps(self):
        self.assertTrue(is_fresh(now_sec=1.20, stamp_sec=1.10, timeout_sec=0.20))
        self.assertFalse(is_fresh(now_sec=1.31, stamp_sec=1.10, timeout_sec=0.20))


if __name__ == "__main__":
    unittest.main()
