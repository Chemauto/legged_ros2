/**
 * @file g1_lowlevel_node.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2026-02-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#include "legged_ros2_control/robots/unitree_g1/g1_lowlevel_node.hpp"

namespace legged
{

G1LowLevelNode::G1LowLevelNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options) : Node(node_name, options)
{
    init_lowcmd_();

    lowstate_subscriber_ = this->create_subscription<unitree_hg::msg::LowState>(
        "lowstate", rclcpp::SensorDataQoS(),
        std::bind(&G1LowLevelNode::lowstate_callback, this, std::placeholders::_1));

    lowcmd_publisher_ = this->create_publisher<unitree_hg::msg::LowCmd>(
        "/lowcmd", 10);
}


void G1LowLevelNode::lowstate_callback(const unitree_hg::msg::LowState::SharedPtr msg){
    lowstate_ = *msg;
    lowcmd_.mode_machine = lowstate_.mode_machine;
}


void G1LowLevelNode::publish_lowcmd(){
    get_crc(lowcmd_);
    lowcmd_publisher_->publish(lowcmd_);
}

void G1LowLevelNode::init_lowcmd_(){
    lowcmd_.mode_pr = 0;
    lowcmd_.mode_machine = 0;
    lowcmd_.reserve.fill(0);
    lowcmd_.crc = 0;

    for (auto & motor_cmd : lowcmd_.motor_cmd) {
        motor_cmd.mode = 0;
        motor_cmd.q = 0.0f;
        motor_cmd.kp = 0.0f;
        motor_cmd.dq = 0.0f;
        motor_cmd.kd = 0.0f;
        motor_cmd.tau = 0.0f;
        motor_cmd.reserve = 0;
    }
}

} // namespace legged
