import importlib.util
from pathlib import Path
import unittest

import numpy as np


bridge_file = (
    Path(__file__).resolve().parents[1]
    / "push_controller"
    / "push_box_obs_bridge_node.py"
)
spec = importlib.util.spec_from_file_location("push_box_obs_bridge_node", bridge_file)
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


class PushBoxObsBridgeTest(unittest.TestCase):

    def test_height_map_data_to_float_list_preserves_16d_obs(self):
        data = np.arange(16, dtype=np.float32)

        converted = bridge.height_map_data_to_float_list(data, expected_size=16)

        self.assertEqual(converted, [float(value) for value in data])

    def test_height_map_data_to_float_list_rejects_wrong_size(self):
        with self.assertRaises(ValueError):
            bridge.height_map_data_to_float_list([1.0, 2.0], expected_size=16)


if __name__ == "__main__":
    unittest.main()
