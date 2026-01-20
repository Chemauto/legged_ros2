/**
 * @file go2_system_interface.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief System interface for Unitree Go2 robot
 * @ref https://github.com/PickNikRobotics/topic_based_ros2_control
 * @version 0.1
 * @date 2025-12-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include "legged_ros2_control/robots/unitree_go2/go2_lowlevel_node.hpp"

#include "legged_ros2_control/legged_system_interface.hpp"
#include "legged_ros2_control/visibility_control.h"

namespace legged
{


class HARDWARE_INTERFACE_PUBLIC Go2SystemInterface : public LeggedSystemInterface
{

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Go2SystemInterface)
  RCLCPP_UNIQUE_PTR_DEFINITIONS(Go2SystemInterface)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  bool build_joint_data_() override;

  Go2LowLevelNode::SharedPtr unitree_ros2_node_;

  bool enable_lowlevel_write_ = false; // if true, do not write lowcmd to robot


};


}

