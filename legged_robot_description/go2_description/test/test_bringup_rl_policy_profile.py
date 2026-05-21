import importlib.util
from pathlib import Path
import unittest


class BringupRLPolicyProfileTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        launch_file = (
            Path(__file__).resolve().parents[1] / "launch" / "bringup_rl.launch.py"
        )
        spec = importlib.util.spec_from_file_location("bringup_rl_launch", launch_file)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def test_default_profile_uses_rl_policy(self):
        paths = self.module.get_policy_profile_paths("rl")

        self.assertEqual(paths.policy_dir_parts, ("rl_policy",))

    def test_nav_low_level_profile_uses_nested_policy(self):
        paths = self.module.get_policy_profile_paths("nav_low_level")

        self.assertEqual(paths.policy_dir_parts, ("nav_policy", "low_level_policy"))

    def test_unknown_profile_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "Unsupported policy_profile"):
            self.module.get_policy_profile_paths("missing")


if __name__ == "__main__":
    unittest.main()
