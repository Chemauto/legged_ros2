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
        cls.bringup_rl = load_module(
            "bringup_rl_launch",
            go2_description_dir / "launch" / "bringup_rl.launch.py",
        )
        cls.bringup_push = load_module(
            "bringup_push_launch",
            go2_description_dir / "launch" / "bringup_push.launch.py",
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


if __name__ == "__main__":
    unittest.main()
