/**
 * @file legged_articulation.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include "semantic_components/imu_sensor.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "legged_rl_controller/isaaclab/assets/articulation/articulation.h"
#include "legged_ros2_controller/semantic_components/joint_interface.hpp"


namespace legged{

using TwistMsgSharedPtr = std::shared_ptr<geometry_msgs::msg::Twist>;
using CmdBuffer = realtime_tools::RealtimeBuffer<TwistMsgSharedPtr>;

class LeggedArticulation : public isaaclab::Articulation
{
public:
  LeggedArticulation(std::shared_ptr<semantic_components::IMUSensor> imu_interface,
                     std::shared_ptr<JointInterface> joint_interface, 
                     std::shared_ptr<CmdBuffer> cmd_vel_buffer)
    : imu_interface_(std::move(imu_interface)), joint_interface_(std::move(joint_interface)), cmd_vel_buffer_(std::move(cmd_vel_buffer))
  {}

  void update() override {
    if (!joint_interface_ || !imu_interface_ || !cmd_vel_buffer_) {
      return;
    }
    // update joint data
    auto joint_pos_double = joint_interface_->get_joint_position();
    auto joint_vel_double = joint_interface_->get_joint_velocity();
    const size_t expected_joints = data.joint_names.size();
    if (expected_joints != 0U &&
        (joint_pos_double.size() != expected_joints || joint_vel_double.size() != expected_joints)) {
      return;
    }
    if (expected_joints == 0U) {
      return;
    }

    // Map the std::vector<double> to an Eigen::Map<Eigen::VectorXd> without copying
    // Then cast the entire vector from double to float and assign it.
    data.joint_pos = Eigen::Map<const Eigen::VectorXd>(joint_pos_double.data(), joint_pos_double.size()).cast<float>();
    data.joint_vel = Eigen::Map<const Eigen::VectorXd>(joint_vel_double.data(), joint_vel_double.size()).cast<float>();

    // Update IMU data
    
    // base angular velocity
    std::array<double, 3> ang_vel = imu_interface_->get_angular_velocity();
    data.root_ang_vel_b = Eigen::Vector3f(ang_vel[0], ang_vel[1], ang_vel[2]);
    
    // quaternion orientation
    std::array<double, 4> quat = imu_interface_->get_orientation();  // (x,y,z,w)
    Eigen::Quaternionf q(quat[3], quat[0], quat[1], quat[2]); // w,x,y,z
    data.projected_gravity_b = q.conjugate() * data.GRAVITY_VEC_W;
    data.root_quat = q;

    // Update Command
    TwistMsgSharedPtr cmd_vel_msg = *cmd_vel_buffer_->readFromRT();
    if(cmd_vel_msg == nullptr){
      data.velocity_command.lin_vel_x = 0.0;
      data.velocity_command.lin_vel_y = 0.0;
      data.velocity_command.ang_vel_z = 0.0;
    } else {
      data.velocity_command.lin_vel_x = cmd_vel_msg->linear.x;
      data.velocity_command.lin_vel_y = cmd_vel_msg->linear.y;
      data.velocity_command.ang_vel_z = cmd_vel_msg->angular.z;
    }
  }

private:
  std::shared_ptr<semantic_components::IMUSensor> imu_interface_;
  std::shared_ptr<JointInterface> joint_interface_;
  std::shared_ptr<CmdBuffer> cmd_vel_buffer_;
};

}
