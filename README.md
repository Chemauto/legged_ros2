# legged_ros2

[![Ubuntu 20.04/22.04](https://img.shields.io/badge/Ubuntu-22.04-blue.svg?logo=ubuntu)](https://ubuntu.com/) [![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg?logo=ros)](https://docs.ros.org/en/humble/index.html)

ROS 2 packages for the control, simulation, and deployment of legged robots. 

## Sim2Sim / Sim2Real Tips

* Order of joints
* `armature` of joint in mujoco model (refer to [armature in mujoco doc](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint-armature))
* IMU mounting position and orientation
* Joint limits (RL policy might exploit the joint limits, if the real robot has different joint limits, the policy might not work well)


