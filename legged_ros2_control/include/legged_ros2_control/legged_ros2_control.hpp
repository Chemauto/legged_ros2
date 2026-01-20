/**
 * @file legged_ros2_control.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief This class serves as an substitute for the `ros2_control_node`
 * @ref https://github.com/ros-controls/ros2_control/blob/humble/controller_manager/src/ros2_control_node.cpp
 *      https://github.com/moveit/mujoco_ros2_control/blob/main/mujoco_ros2_control/src/mujoco_ros2_control.cpp
 * @version 0.1
 * @date 2025-07-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <memory>
#include <string>

#include "controller_manager/controller_manager.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include "legged_ros2_control/legged_system_interface.hpp"

namespace legged {

/// A class to maintain the controller manager and resource manager of ros2 control
class LeggedRos2Control{
public:
  LeggedRos2Control(rclcpp::Node::SharedPtr node);
  ~LeggedRos2Control();

  /**
   * @brief Initialize the controller manager and resource manager
   * 
   * In the default usage of ros2 control, the controlller manager is 
   * responsible for parsing the hardware info from URDF and load the 
   * components. But here we will do it manually.
   * 
   * We do this to have more control of the initialization process.
   */
  void init();

  virtual void update(const rclcpp::Time &time, const rclcpp::Duration &period);

  int get_update_rate() const { return update_rate_; }

protected:
  std::string urdf_string_;
  std::string get_robot_description_();

  /**
   * @brief Import components into the resource manager according to the hardware info
   * 
   * @param hardware_info 
   * @param resource_manager 
   */
  virtual void import_components_(std::vector<hardware_interface::HardwareInfo> &hardware_info, 
                        std::unique_ptr<hardware_interface::ResourceManager> &resource_manager);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;

  std::shared_ptr<pluginlib::ClassLoader<LeggedSystemInterface>> system_interface_loader_;

  int update_rate_;
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  rclcpp::Executor::SharedPtr cm_executor_;
  std::thread cm_thread_;
  std::thread spin_thread_;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;

};




} // namespace legged