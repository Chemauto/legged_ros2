/**
 * @file motor_crc_hg.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief This file is copied from unitree_ros2 repository
 * @version 0.1
 * @date 2026-02-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#ifndef _MOTOR_CRC_HG_H_
#define _MOTOR_CRC_HG_H_

#include <stdint.h>

#include <array>
#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/motor_cmd.hpp"

constexpr int kG1NumMotor = 29;

typedef struct {
  uint8_t mode;  // desired working mode
  float q;       // desired angle (unit: radian)
  float dq;      // desired velocity (unit: radian/second)
  float tau;     // desired output torque (unit: N.m)
  float Kp;      // desired position stiffness (unit: N.m/rad )
  float Kd;      // desired velocity stiffness (unit: N.m/(rad/s) )
  uint32_t reserve = 0;
} MotorCmd;  // motor control

typedef struct {
  uint8_t modePr;
  uint8_t modeMachine;
  std::array<MotorCmd, 35> motorCmd;
  std::array<uint32_t, 4> reserve;
  uint32_t crc;
} LowCmd;

uint32_t crc32_core(uint32_t *ptr, uint32_t len);
void get_crc(unitree_hg::msg::LowCmd &msg);

enum class JointIndex{
    Left_Hip_Pitch = 0,
    Left_Hip_Roll = 1,
    Left_Hip_Yaw = 2,
    Left_Knee = 3,
    Left_Ankle_Pitch = 4,
    Left_Ankle_Roll = 5,
    Right_Hip_Pitch = 6,
    Right_Hip_Roll = 7,
    Right_Hip_Yaw = 8,
    Right_Knee = 9,
    Right_Ankle_Pitch = 10,
    Right_Ankle_Roll = 11,
    Waist_Yaw = 12,
    Waist_Roll = 13,
    Waist_Pitch = 14,
    Left_Shoulder_Pitch = 15,
    Left_Shoulder_Roll = 16,
    Left_Shoulder_Yaw = 17,
    Left_Elbow = 18,
    Left_Wrist_Roll = 19,
    Left_Wrist_Pitch = 20,
    Left_Wrist_Yaw = 21,
    Right_Shoulder_Pitch = 22,
    Right_Shoulder_Roll = 23,
    Right_Shoulder_Yaw = 24,
    Right_Elbow = 25,
    Right_Wrist_Roll = 26,
    Right_Wrist_Pitch = 27,
    Right_Wrist_Yaw = 28,
};

static const std::map<std::string, JointIndex> g1_joint_index_map = {
    {"left_hip_pitch_joint", JointIndex::Left_Hip_Pitch},
    {"left_hip_roll_joint", JointIndex::Left_Hip_Roll},
    {"left_hip_yaw_joint", JointIndex::Left_Hip_Yaw},
    {"left_knee_joint", JointIndex::Left_Knee},
    {"left_ankle_pitch_joint", JointIndex::Left_Ankle_Pitch},
    {"left_ankle_roll_joint", JointIndex::Left_Ankle_Roll},
    {"right_hip_pitch_joint", JointIndex::Right_Hip_Pitch},
    {"right_hip_roll_joint", JointIndex::Right_Hip_Roll},
    {"right_hip_yaw_joint", JointIndex::Right_Hip_Yaw},
    {"right_knee_joint", JointIndex::Right_Knee},
    {"right_ankle_pitch_joint", JointIndex::Right_Ankle_Pitch},
    {"right_ankle_roll_joint", JointIndex::Right_Ankle_Roll},
    {"waist_yaw_joint", JointIndex::Waist_Yaw},
    {"waist_roll_joint", JointIndex::Waist_Roll},
    {"waist_pitch_joint", JointIndex::Waist_Pitch},
    {"left_shoulder_pitch_joint", JointIndex::Left_Shoulder_Pitch},
    {"left_shoulder_roll_joint", JointIndex::Left_Shoulder_Roll},
    {"left_shoulder_yaw_joint", JointIndex::Left_Shoulder_Yaw},
    {"left_elbow_joint", JointIndex::Left_Elbow},
    {"left_wrist_roll_joint", JointIndex::Left_Wrist_Roll},
    {"left_wrist_pitch_joint", JointIndex::Left_Wrist_Pitch},
    {"left_wrist_yaw_joint", JointIndex::Left_Wrist_Yaw},
    {"right_shoulder_pitch_joint", JointIndex::Right_Shoulder_Pitch},
    {"right_shoulder_roll_joint", JointIndex::Right_Shoulder_Roll},
    {"right_shoulder_yaw_joint", JointIndex::Right_Shoulder_Yaw},
    {"right_elbow_joint", JointIndex::Right_Elbow},
    {"right_wrist_roll_joint", JointIndex::Right_Wrist_Roll},
    {"right_wrist_pitch_joint", JointIndex::Right_Wrist_Pitch},
    {"right_wrist_yaw_joint", JointIndex::Right_Wrist_Yaw},
};

#endif
