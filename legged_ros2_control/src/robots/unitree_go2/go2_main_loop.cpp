/**
 * @file go2_main_loop.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-12-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "rclcpp/rclcpp.hpp"
#include "legged_ros2_control/legged_ros2_control.hpp"
#include "legged_ros2_control/robots/unitree_go2/go2_wireless_controller.hpp"


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "go2_ros2_control_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // The controller manager and resource manager are maintained inside LeggedRos2Control
    auto legged_ros2_control = std::make_shared<legged::LeggedRos2Control>(node);
    // There is executor to spin so we don't need to create another one here
    legged_ros2_control->init();

    // Create wireless controller node, it is responsible for receiving wireless controller data and 
    //  * publishing cmd_vel topic
    //  * switching controllers
    auto go2_wireless_controller = std::make_shared<legged::Go2WirelessController>(
        "go2_wireless_controller",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Only spin the wireless controller node here, others are handled inside LeggedRos2Control
    rclcpp::spin(go2_wireless_controller);
    rclcpp::shutdown();
    
    return 0;
}
