/**
 * @file motor_crc.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief This file is copied from unitree_ros2 repository
 * @version 0.1
 * @date 2025-12-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#ifndef _MOTOR_CRC_H_
#define _MOTOR_CRC_H_

#include <stdint.h>

#include <array>
#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/bms_cmd.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"

constexpr int HIGHLEVEL = 0xee;
constexpr int LOWLEVEL = 0xff;
constexpr int TRIGERLEVEL = 0xf0;
constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

// joint index
constexpr int FR_0 = 0;
constexpr int FR_1 = 1;
constexpr int FR_2 = 2;

constexpr int FL_0 = 3;
constexpr int FL_1 = 4;
constexpr int FL_2 = 5;

constexpr int RR_0 = 6;
constexpr int RR_1 = 7;
constexpr int RR_2 = 8;

constexpr int RL_0 = 9;
constexpr int RL_1 = 10;
constexpr int RL_2 = 11;

typedef struct {
  uint8_t off;  // off 0xA5
  std::array<uint8_t, 3> reserve;
} BmsCmd;

typedef struct {
  uint8_t mode;  // desired working mode
  float q;       // desired angle (unit: radian)
  float dq;      // desired velocity (unit: radian/second)
  float tau;     // desired output torque (unit: N.m)
  float Kp;      // desired position stiffness (unit: N.m/rad )
  float Kd;      // desired velocity stiffness (unit: N.m/(rad/s) )
  std::array<uint32_t, 3> reserve;
} MotorCmd;  // motor control

typedef struct {
  std::array<uint8_t, 2> head;
  uint8_t levelFlag;
  uint8_t frameReserve;

  std::array<uint32_t, 2> SN;
  std::array<uint32_t, 2> version;
  uint16_t bandWidth;
  std::array<MotorCmd, 20> motorCmd;
  BmsCmd bms;
  std::array<uint8_t, 40> wirelessRemote;
  std::array<uint8_t, 12> led;
  std::array<uint8_t, 2> fan;
  uint8_t gpio;
  uint32_t reserve;

  uint32_t crc;
} LowCmd;

uint32_t crc32_core(uint32_t* ptr, uint32_t len);
void get_crc(unitree_go::msg::LowCmd& msg);


enum class JointIndex{
    FR_Hip = 0,
    FR_Thigh = 1,
    FR_Calf = 2,
    FL_Hip = 3,
    FL_Thigh = 4,
    FL_Calf = 5,
    RR_Hip = 6,
    RR_Thigh = 7,
    RR_Calf = 8,
    RL_Hip = 9,
    RL_Thigh = 10,
    RL_Calf = 11,
};

static const std::map<std::string, JointIndex> go2_joint_index_map = {
    {"FR_hip_joint", JointIndex::FR_Hip},
    {"FR_thigh_joint", JointIndex::FR_Thigh},
    {"FR_calf_joint", JointIndex::FR_Calf},
    {"FL_hip_joint", JointIndex::FL_Hip},
    {"FL_thigh_joint", JointIndex::FL_Thigh},
    {"FL_calf_joint", JointIndex::FL_Calf},
    {"RR_hip_joint", JointIndex::RR_Hip},
    {"RR_thigh_joint", JointIndex::RR_Thigh},
    {"RR_calf_joint", JointIndex::RR_Calf},
    {"RL_hip_joint", JointIndex::RL_Hip},
    {"RL_thigh_joint", JointIndex::RL_Thigh},
    {"RL_calf_joint", JointIndex::RL_Calf},
};


#endif