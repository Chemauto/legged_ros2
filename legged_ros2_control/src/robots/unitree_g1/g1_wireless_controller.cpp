/**
 * @file g1_wireless_controller.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2026-02-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <map>
#include <cstring>

#include "legged_ros2_control/robots/unitree_g1/g1_wireless_controller.hpp"

namespace legged
{

using namespace unitree::common;

G1WirelessController::G1WirelessController(
    const std::string & node_name,
    const rclcpp::NodeOptions & options) : Node(node_name, options)
{
    wireless_controller_subscriber_ = this->create_subscription<unitree_hg::msg::LowState>(
        "lowstate", rclcpp::SensorDataQoS(),
        std::bind(&G1WirelessController::wireless_controller_callback, this, std::placeholders::_1));

    cmd_vel_scale_.lin_vel_x = this->get_parameter_or<float>("cmd_vel.scale.lin_vel_x", 1.0);
    cmd_vel_scale_.lin_vel_y = this->get_parameter_or<float>("cmd_vel.scale.lin_vel_y", 1.0);
    cmd_vel_scale_.ang_vel_z = this->get_parameter_or<float>("cmd_vel.scale.ang_vel_z", 1.0);
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::QoS(10));
    cmd_vel_msg_ = geometry_msgs::msg::Twist();

    load_controller_bindings_();

    controller_switch_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
        "controller_manager/switch_controller");
    switch_controller_request_ = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    switch_controller_request_->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    wait_for_service_();
}

void G1WirelessController::wireless_controller_callback(
    const unitree_hg::msg::LowState::SharedPtr msg)
{
    REMOTE_DATA_RX remote{};
    std::memcpy(remote.buff, msg->wireless_remote.data(), sizeof(remote.buff));
    BtnUnion btn_union = remote.RF_RX.btn;
    
    // Update joystick state
    joystick_.back(btn_union.components.Select);
    joystick_.start(btn_union.components.Start);
    joystick_.LB(btn_union.components.L1);
    joystick_.RB(btn_union.components.R1);
    // Treat digital L2 as LS for extra slots.
    joystick_.LS(btn_union.components.L2);
    joystick_.F1(btn_union.components.f1);
    joystick_.F2(btn_union.components.f2);
    joystick_.A(btn_union.components.A);
    joystick_.B(btn_union.components.B);
    joystick_.X(btn_union.components.X);
    joystick_.Y(btn_union.components.Y);
    joystick_.up(btn_union.components.up);
    joystick_.down(btn_union.components.down);
    joystick_.left(btn_union.components.left);
    joystick_.right(btn_union.components.right);
    joystick_.LT(btn_union.components.L2);
    joystick_.RT(btn_union.components.R2);
    joystick_.lx(remote.RF_RX.lx);
    joystick_.ly(remote.RF_RX.ly);
    joystick_.rx(remote.RF_RX.rx);
    joystick_.ry(remote.RF_RX.ry);

    transfer_axes_();
    transfer_btns_();
}


void G1WirelessController::transfer_axes_()
{
    // Map joystick axes to robot commands
    cmd_vel_msg_.linear.x = cmd_vel_scale_.lin_vel_x * joystick_.ly();
    cmd_vel_msg_.linear.y = - cmd_vel_scale_.lin_vel_y * joystick_.lx();
    cmd_vel_msg_.angular.z = - cmd_vel_scale_.ang_vel_z * joystick_.rx();
    cmd_vel_publisher_->publish(cmd_vel_msg_);
}


void G1WirelessController::transfer_btns_()
{
    int triggered_slot = 0;
    if (joystick_.LB.pressed && joystick_.A.on_pressed) {
        triggered_slot = 1;
    } else if (joystick_.LB.pressed && joystick_.B.on_pressed) {
        triggered_slot = 2;
    } else if (joystick_.LB.pressed && joystick_.X.on_pressed) {
        triggered_slot = 3;
    } else if (joystick_.LB.pressed && joystick_.Y.on_pressed) {
        triggered_slot = 4;
    } else if (joystick_.LS.pressed && joystick_.A.on_pressed) {
        triggered_slot = 5;
    } else if (joystick_.LS.pressed && joystick_.B.on_pressed) {
        triggered_slot = 6;
    } else if (joystick_.LS.pressed && joystick_.X.on_pressed) {
        triggered_slot = 7;
    } else if (joystick_.LS.pressed && joystick_.Y.on_pressed) {
        triggered_slot = 8;
    } else if (joystick_.LB.pressed && joystick_.RB.on_pressed) {
        triggered_slot = 9;
    }

    if (triggered_slot == 0) {
        return;
    }

    if (controller_names_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No controller bindings configured.");
        return;
    }

    if (triggered_slot == off_slot_) {
        request_controller_switch_(
            {},
            controller_names_,
            "Switched off all controllers."
        );
        return;
    }

    if (triggered_slot < 1 || triggered_slot > kSlotCount) {
        RCLCPP_WARN(this->get_logger(), "Invalid slot %d triggered.", triggered_slot);
        return;
    }

    const std::string & active_controller = slot_to_controller_[triggered_slot];
    if (active_controller.empty()) {
        RCLCPP_WARN(this->get_logger(), "No controller bound to slot %d.", triggered_slot);
        return;
    }

    std::vector<std::string> deactivate;
    deactivate.reserve(controller_names_.size());
    for (const auto & name : controller_names_) {
        if (name != active_controller) {
            deactivate.push_back(name);
        }
    }
    request_controller_switch_(
        {active_controller},
        deactivate,
        std::string("Switched to controller: ") + active_controller
    );
}

void G1WirelessController::load_controller_bindings_()
{
    slot_to_controller_.assign(kSlotCount + 1, "");
    controller_names_.clear();
    off_slot_ = kDefaultOffSlot;

    RCLCPP_INFO(this->get_logger(), "========== controller_bindings begin ==========");
    std::unordered_map<std::string, int> bindings;
    std::map<std::string, rclcpp::Parameter> params;
    this->get_node_parameters_interface()->get_parameters_by_prefix("controller_bindings", params);
    for (const auto & entry : params) {
        if (entry.second.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
            RCLCPP_WARN(this->get_logger(), "Binding '%s' is not an integer slot id.", entry.first.c_str());
            continue;
        }
        bindings[entry.first] = static_cast<int>(entry.second.as_int());
    }
    if (bindings.empty()) {
        RCLCPP_WARN(this->get_logger(), "controller_bindings is empty.");
    } else {
        for (const auto & entry : bindings) {
            RCLCPP_INFO(this->get_logger(), "binding: %s -> %d", entry.first.c_str(), entry.second);
        }
    }
    RCLCPP_INFO(this->get_logger(), "========== controller_bindings end ==========");

    for (const auto & entry : bindings) {
        const std::string & name = entry.first;
        const int slot = entry.second;
        if (name == "off") {
            if (slot != kDefaultOffSlot) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Off binding must use slot %d (LB+RB), got %d. Ignoring.",
                    kDefaultOffSlot, slot);
                continue;
            }
            off_slot_ = slot;
            continue;
        }
        if (slot < 1 || slot > kSlotCount) {
            RCLCPP_WARN(
                this->get_logger(),
                "Controller '%s' has invalid slot %d. Valid range is 1-%d.",
                name.c_str(), slot, kSlotCount);
            continue;
        }
        if (!slot_to_controller_[slot].empty()) {
            RCLCPP_WARN(
                this->get_logger(),
                "Slot %d already bound to '%s'. Ignoring '%s'.",
                slot, slot_to_controller_[slot].c_str(), name.c_str());
            continue;
        }
        slot_to_controller_[slot] = name;
        controller_names_.push_back(name);
    }
}

void G1WirelessController::request_controller_switch_(
    const std::vector<std::string>& activate, 
    const std::vector<std::string>& deactivate, 
    const std::string& log_msg
){
    switch_controller_request_->activate_controllers = activate;
    switch_controller_request_->deactivate_controllers = deactivate;
    switch_controller_request_->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    switch_controller_request_->activate_asap = true;
    controller_switch_client_->async_send_request(switch_controller_request_);
    RCLCPP_INFO(this->get_logger(), "%s", log_msg.c_str());
}


void G1WirelessController::wait_for_service_()
{
    while (!controller_switch_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
}

} // namespace legged
