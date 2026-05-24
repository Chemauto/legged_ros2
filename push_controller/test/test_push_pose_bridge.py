import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest


bridge_file = (
    Path(__file__).resolve().parents[1] / "push_controller" / "push_pose_bridge_node.py"
)
spec = importlib.util.spec_from_file_location("push_pose_bridge", bridge_file)
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)

pose_to_odometry = bridge.pose_to_odometry


def _pose_stamped(x=1.0, y=2.0, z=0.3, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="odom", stamp="stamp"),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=z),
            orientation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
        ),
    )


class PushPoseBridgeTest(unittest.TestCase):

    def test_pose_to_odometry_copies_pose_and_frames(self):
        pose_msg = _pose_stamped(x=1.2, y=-0.4, z=0.5, qz=0.3, qw=0.95)

        odom = pose_to_odometry(pose_msg, child_frame_id="base")

        self.assertEqual(odom.header.frame_id, "odom")
        self.assertEqual(odom.header.stamp, "stamp")
        self.assertEqual(odom.child_frame_id, "base")
        self.assertEqual(odom.pose.pose.position.x, 1.2)
        self.assertEqual(odom.pose.pose.position.y, -0.4)
        self.assertEqual(odom.pose.pose.position.z, 0.5)
        self.assertEqual(odom.pose.pose.orientation.z, 0.3)
        self.assertEqual(odom.pose.pose.orientation.w, 0.95)
        self.assertEqual(odom.twist.twist.angular.z, 0.0)


if __name__ == "__main__":
    unittest.main()
