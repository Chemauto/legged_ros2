import importlib
import sys
import threading
import unittest
from unittest.mock import patch


class LlmServiceDefaultsTest(unittest.TestCase):

    def test_robot_service_defaults_match_legged_ros2_topics(self):
        robot_service = importlib.import_module("llmservice.robot_service")

        with patch.object(sys, "argv", ["robot_service.py"]):
            args = robot_service.parse_args()

        self.assertEqual(args.odom_topic, "/odom")
        self.assertEqual(args.goal_pose_topic, "/go2/goal_pose")
        self.assertEqual(args.skill_command_topic, "/go2/skill_command")
        self.assertEqual(args.cmd_vel_topic, "/cmd_vel")

    def test_protocol_accepts_planner_skill_aliases(self):
        protocol = importlib.import_module("llmservice.protocol")

        self.assertEqual(protocol.normalize_skill("navigation"), "nav")
        self.assertEqual(protocol.normalize_skill("push_box"), "push")
        self.assertEqual(protocol.normalize_skill("walk"), "walk_skill")

    def test_ros2_state_requires_robot_odom_before_ready(self):
        ros2_state = importlib.import_module("llmservice.ros2_state")
        state = object.__new__(ros2_state.Ros2TopicState)
        state._lock = threading.Lock()
        state._robot = None
        state._skill_status = {"start": True}

        self.assertFalse(state.ready())

        state._robot = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}

        self.assertTrue(state.ready())


if __name__ == "__main__":
    unittest.main()
