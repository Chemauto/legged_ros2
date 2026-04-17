/**
 * @file g1_system_interface.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief System interface for Unitree G1 robot
 * @ref https://github.com/PickNikRobotics/topic_based_ros2_control
 * @version 0.1
 * @date 2026-02-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "legged_ros2_control/robots/unitree_g1/g1_system_interface.hpp"
#include "legged_ros2_control/robots/unitree_g1/motor_crc_hg.h"
#include <algorithm>
#include <cctype>

namespace legged
{

CallbackReturn G1SystemInterface::on_init(const hardware_interface::HardwareInfo & info){
  if (LeggedSystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if(imu_data_.size() != 1){
    RCLCPP_ERROR(*logger_, "G1SystemInterface only supports one IMU sensor");
    return CallbackReturn::ERROR;
  }

  // create lowlevel node for unitree ros2 communication
  unitree_ros2_node_ = std::make_shared<G1LowLevelNode>();

  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("G1SystemInterface"));


  auto it = info.hardware_parameters.find("enable_lowlevel_write");
  if (it != info.hardware_parameters.end()) {
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    enable_lowlevel_write_ = (value == "true");
  } else {
    RCLCPP_WARN(*logger_, "Parameter 'enable_lowlevel_write' not found, default to false");
    enable_lowlevel_write_ = false;
  }

  RCLCPP_INFO(*logger_, "Enable_lowlevel_write_: %s", enable_lowlevel_write_ ? "true" : "false");
  RCLCPP_INFO(*logger_, "G1SystemInterface initialized successfully");


  return CallbackReturn::SUCCESS;
}


bool G1SystemInterface::build_joint_data_(){
  for(size_t i=0; i<joint_data_.size(); i++){
    const auto & jnt_name = joint_data_[i].name;
    auto it = g1_joint_index_map.find(jnt_name);
    if(it == g1_joint_index_map.end()){
      RCLCPP_ERROR(*logger_, "Joint name %s not found in G1 joint map", jnt_name.c_str());
      return false;
    }
    joint_data_[i].adr = static_cast<int>(it->second);
  }
  return true;
}


return_type G1SystemInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
  
  if(rclcpp::ok()){
    rclcpp::spin_some(unitree_ros2_node_);
  }

  // Read joint data
  for (size_t i=0; i < joint_data_.size(); ++i) {
    joint_data_[i].pos_ = unitree_ros2_node_->lowstate().motor_state[joint_data_[i].adr].q;
    joint_data_[i].vel_ = unitree_ros2_node_->lowstate().motor_state[joint_data_[i].adr].dq;
    joint_data_[i].tau_ = unitree_ros2_node_->lowstate().motor_state[joint_data_[i].adr].tau_est;
  }

  // Read IMU data
  // Only use one IMU here
  // Unitree HG quaternion: w x y z
  // ImuData quaternion: x y z w
  imu_data_[0].quat_[0] = unitree_ros2_node_->lowstate().imu_state.quaternion[1]; // x
  imu_data_[0].quat_[1] = unitree_ros2_node_->lowstate().imu_state.quaternion[2]; // y
  imu_data_[0].quat_[2] = unitree_ros2_node_->lowstate().imu_state.quaternion[3]; // z
  imu_data_[0].quat_[3] = unitree_ros2_node_->lowstate().imu_state.quaternion[0]; // w
  imu_data_[0].ang_vel_[0] = unitree_ros2_node_->lowstate().imu_state.gyroscope[0]; // angular velocity x
  imu_data_[0].ang_vel_[1] = unitree_ros2_node_->lowstate().imu_state.gyroscope[1]; // angular velocity y
  imu_data_[0].ang_vel_[2] = unitree_ros2_node_->lowstate().imu_state.gyroscope[2]; // angular velocity z
  imu_data_[0].lin_acc_[0] = unitree_ros2_node_->lowstate().imu_state.accelerometer[0]; // linear acceleration x
  imu_data_[0].lin_acc_[1] = unitree_ros2_node_->lowstate().imu_state.accelerometer[1]; // linear acceleration y
  imu_data_[0].lin_acc_[2] = unitree_ros2_node_->lowstate().imu_state.accelerometer[2]; // linear acceleration z

  return return_type::OK;
}


return_type G1SystemInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){

  if(!enable_lowlevel_write_){
    return return_type::OK;
  }

  for(size_t i=0; i < joint_data_.size(); ++i) {
    unitree_ros2_node_->lowcmd().motor_cmd[joint_data_[i].adr].mode = 1;
    unitree_ros2_node_->lowcmd().motor_cmd[joint_data_[i].adr].q = joint_data_[i].pos_cmd_;
    unitree_ros2_node_->lowcmd().motor_cmd[joint_data_[i].adr].dq = joint_data_[i].vel_cmd_;
    unitree_ros2_node_->lowcmd().motor_cmd[joint_data_[i].adr].tau = joint_data_[i].ff_cmd_;
    unitree_ros2_node_->lowcmd().motor_cmd[joint_data_[i].adr].kp = joint_data_[i].kp_;
    unitree_ros2_node_->lowcmd().motor_cmd[joint_data_[i].adr].kd = joint_data_[i].kd_;

    // record the desired position and velocity for reading
    joint_data_[i].pos_des_ = joint_data_[i].pos_cmd_;
    joint_data_[i].vel_des_ = joint_data_[i].vel_cmd_;
  }

  unitree_ros2_node_->publish_lowcmd();

  return return_type::OK;
}

    
} // namespace legged

#include "pluginlib/class_list_macros.hpp"
// <!-- #########jazzy########## -->
// PLUGINLIB_EXPORT_CLASS(
//   legged::G1SystemInterface, legged::LeggedSystemInterface)
// <!-- #########jazzy########## -->
// <!-- #########new########## -->
// 新代码,使用jazzy版本
PLUGINLIB_EXPORT_CLASS(
  legged::G1SystemInterface, hardware_interface::SystemInterface)
// <!-- #########new########## -->
