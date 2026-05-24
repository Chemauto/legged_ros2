import importlib.util
from pathlib import Path
import unittest


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class BringupNavLaunchTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        go2_description_dir = Path(__file__).resolve().parents[1]
        repo_dir = go2_description_dir.parents[1]
        cls.bringup_nav = load_module(
            "bringup_nav_launch",
            go2_description_dir / "launch" / "bringup_nav.launch.py",
        )
        cls.nav_controller_launch = load_module(
            "nav_controller_launch",
            repo_dir / "nav_controller" / "launch" / "nav_controller.launch.py",
        )

    def test_nav_launch_uses_nav_low_level_profile(self):
        self.assertEqual(
            self.bringup_nav.get_low_level_policy_profile(),
            "nav_low_level",
        )

    def test_nav_launch_uses_go2_nav_policy_for_high_level(self):
        self.assertEqual(
            self.bringup_nav.get_high_level_policy_path_parts(),
            ("config", "nav_policy", "policy.onnx"),
        )

    def test_nav_controller_default_model_comes_from_go2_description(self):
        self.assertEqual(
            self.nav_controller_launch.get_default_model_path_parts(),
            ("config", "nav_policy", "policy.onnx"),
        )

    def test_nav_launch_uses_isolated_cmd_vel_topic(self):
        self.assertEqual(
            self.bringup_nav.get_default_cmd_vel_topic(),
            "/nav_cmd_vel",
        )

    def test_nav_controller_standalone_default_uses_cmd_vel(self):
        self.assertEqual(
            self.nav_controller_launch.get_default_cmd_vel_topic(),
            "/cmd_vel",
        )

    def test_nav_launch_real_robot_default_uses_odometry_topic(self):
        self.assertEqual(
            self.bringup_nav.get_default_odom_topic(),
            "/Odometry",
        )

    def test_nav_controller_standalone_default_uses_odometry_topic(self):
        self.assertEqual(
            self.nav_controller_launch.get_default_odom_topic(),
            "/Odometry",
        )

    def test_nav_launch_uses_heightmap_topic(self):
        self.assertEqual(
            self.bringup_nav.get_default_heightmap_topic(),
            "/height_sampler_node/height_map",
        )

    def test_nav_controller_standalone_default_uses_heightmap_topic(self):
        self.assertEqual(
            self.nav_controller_launch.get_default_heightmap_topic(),
            "/height_sampler_node/height_map",
        )


if __name__ == "__main__":
    unittest.main()
