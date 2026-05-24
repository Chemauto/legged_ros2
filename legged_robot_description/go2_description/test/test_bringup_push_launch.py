import importlib.util
from pathlib import Path
import unittest


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class BringupPushLaunchTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        go2_description_dir = Path(__file__).resolve().parents[1]
        repo_dir = go2_description_dir.parents[1]
        cls.bringup_rl = load_module(
            "bringup_rl_launch",
            go2_description_dir / "launch" / "bringup_rl.launch.py",
        )
        cls.bringup_push = load_module(
            "bringup_push_launch",
            go2_description_dir / "launch" / "bringup_push.launch.py",
        )
        cls.push_controller_launch = load_module(
            "push_controller_launch",
            repo_dir / "push_controller" / "launch" / "push_controller.launch.py",
        )

    def test_push_low_level_profile_uses_push_policy(self):
        paths = self.bringup_rl.get_policy_profile_paths("push_low_level")

        self.assertEqual(paths.policy_dir_parts, ("push_policy", "low_level_policy"))

    def test_push_launch_uses_push_low_level_profile(self):
        self.assertEqual(
            self.bringup_push.get_low_level_policy_profile(),
            "push_low_level",
        )

    def test_push_launch_uses_go2_push_policy_for_high_level(self):
        self.assertEqual(
            self.bringup_push.get_high_level_policy_path_parts(),
            ("config", "push_policy", "policy.onnx"),
        )

    def test_push_launch_uses_isolated_cmd_vel_topic(self):
        self.assertEqual(
            self.bringup_push.get_default_cmd_vel_topic(),
            "/push_cmd_vel",
        )

    def test_push_launch_converts_raw_push_obs_to_float_topic(self):
        self.assertEqual(
            self.bringup_push.get_default_converted_push_obs_topic(),
            "/push_box_obs_float",
        )

    def test_push_controller_standalone_default_uses_cmd_vel(self):
        self.assertEqual(
            self.push_controller_launch.get_default_cmd_vel_topic(),
            "/cmd_vel",
        )

    def test_push_high_level_starts_after_controller_spawners(self):
        self.assertEqual(
            self.bringup_push.get_high_level_start_delay_sec(),
            5.0,
        )


if __name__ == "__main__":
    unittest.main()
