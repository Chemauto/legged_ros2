/**
 * @file go2_wireless_controller.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-12-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */


#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

#include "unitree_go/msg/wireless_controller.hpp"

#include "legged_ros2_control/robots/unitree_joystick.hpp"


namespace legged
{

using namespace unitree::common;

/// Class for transforming wireless controller inputs to robot commands
/// including cmd_vel and controller switching
class Go2WirelessController : public rclcpp::Node {
public:
    RCLCPP_SMART_PTR_DEFINITIONS(Go2WirelessController)

    explicit Go2WirelessController(
        const std::string & node_name = "go2_wireless_controller",
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
    );

    const UnitreeJoystick& joystick() const { return joystick_; }

private:

    // -----------------------------------------------------------------------
    // For wireless controller subscription
    // -----------------------------------------------------------------------

    void wireless_controller_callback(const unitree_go::msg::WirelessController::SharedPtr msg);
    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr wireless_controller_subscriber_;

    UnitreeJoystick joystick_;

    void transfer_axes_();
    void transfer_btns_();
    void load_controller_bindings_();

    static constexpr int kSlotCount = 9;
    static constexpr int kDefaultOffSlot = 9;
    int off_slot_{kDefaultOffSlot};
    std::vector<std::string> controller_names_;
    std::vector<std::string> slot_to_controller_;

    // -----------------------------------------------------------------------
    // For command velocity
    // -----------------------------------------------------------------------

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    geometry_msgs::msg::Twist cmd_vel_msg_;
    struct CmdVelScale {
        float lin_vel_x = 1.0;
        float lin_vel_y = 1.0;
        float ang_vel_z = 1.0;
    } cmd_vel_scale_;

    // -----------------------------------------------------------------------
    // For controller switching
    // -----------------------------------------------------------------------

    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_switch_client_;
    std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> switch_controller_request_;

    void request_controller_switch_(
        const std::vector<std::string>& activate, 
        const std::vector<std::string>& deactivate, 
        const std::string& log_msg
    );
    void wait_for_service_();

};


}
