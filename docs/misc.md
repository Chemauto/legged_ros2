
## Sim2Sim / Sim2Real Tips

* Order of joints
* `armature` of joint in mujoco model (refer to [armature in mujoco doc](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint-armature))
* IMU mounting position and orientation
* Joint limits (RL policy might exploit the joint limits, if the real robot has different joint limits, the policy might not work well)

