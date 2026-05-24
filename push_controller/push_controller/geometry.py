import math

import numpy as np


PUSH_OBS_DIM = 16
PUSH_POLICY_OBS_DIM = 19
PUSH_ACTION_CLIP_MIN = np.array([-0.5, -1.0, -0.5], dtype=np.float32)
PUSH_ACTION_CLIP_MAX = np.array([1.0, 1.0, 0.5], dtype=np.float32)
PUSH_GOAL_IN_BOX_POS_SLICE = slice(11, 14)


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_to_rot_matrix(qx, qy, qz, qw):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def projected_gravity(qx, qy, qz, qw):
    rotation = quat_to_rot_matrix(qx, qy, qz, qw)
    return (rotation.T @ np.array([0.0, 0.0, -1.0])).astype(np.float32)


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def is_fresh(now_sec, stamp_sec, timeout_sec):
    return 0.0 <= now_sec - stamp_sec <= timeout_sec


def push_goal_distance_xy(push_obs):
    obs = np.asarray(push_obs, dtype=np.float32).flatten()
    if obs.size < PUSH_OBS_DIM:
        return math.inf
    goal_in_box_pos = obs[PUSH_GOAL_IN_BOX_POS_SLICE]
    return float(math.hypot(float(goal_in_box_pos[0]), float(goal_in_box_pos[1])))


def push_goal_reached(push_obs, tolerance_xy):
    return push_goal_distance_xy(push_obs) <= float(tolerance_xy)


def yaw_to_sin_cos(yaw):
    return np.array([math.sin(yaw), math.cos(yaw)], dtype=np.float32)


def rotate_world_to_yaw_frame(vector, yaw):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    x = cos_yaw * vector[0] + sin_yaw * vector[1]
    y = -sin_yaw * vector[0] + cos_yaw * vector[1]
    return np.array([x, y, vector[2]], dtype=np.float32)


def build_push_observation(
    robot_position,
    robot_yaw,
    projected_gravity,
    base_ang_vel,
    box_position,
    box_yaw,
    goal_position,
    goal_yaw,
):
    box_in_robot_pos = rotate_world_to_yaw_frame(box_position - robot_position, robot_yaw)
    box_in_robot_yaw = yaw_to_sin_cos(wrap_to_pi(box_yaw - robot_yaw))
    goal_in_box_pos = rotate_world_to_yaw_frame(goal_position - box_position, box_yaw)
    goal_in_box_yaw = yaw_to_sin_cos(wrap_to_pi(goal_yaw - box_yaw))

    return np.concatenate([
        np.asarray(base_ang_vel, dtype=np.float32),
        np.asarray(projected_gravity, dtype=np.float32),
        box_in_robot_pos,
        box_in_robot_yaw,
        goal_in_box_pos,
        goal_in_box_yaw,
    ]).astype(np.float32)


def clip_push_action(action):
    return np.clip(
        np.asarray(action, dtype=np.float32),
        PUSH_ACTION_CLIP_MIN,
        PUSH_ACTION_CLIP_MAX,
    ).astype(np.float32)
