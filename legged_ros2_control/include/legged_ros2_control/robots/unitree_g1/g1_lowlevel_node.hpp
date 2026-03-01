/**
 * @file g1_lowlevel_node.hpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief This class is used for low-level communication with Unitree G1 robot
 *  and it would be integrated into g1_system_interface
 * @version 0.1
 * @date 2026-02-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#include "rclcpp/rclcpp.hpp"
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "legged_ros2_control/robots/unitree_g1/motor_crc_hg.h"

namespace legged
{

/// Class for Unitree G1 low-level communication node
class G1LowLevelNode : public rclcpp::Node
{
public:
    RCLCPP_SMART_PTR_DEFINITIONS(G1LowLevelNode)

    explicit G1LowLevelNode(
        const std::string & node_name = "g1_lowlevel_node",
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
    );

    const unitree_hg::msg::LowState& lowstate() const { return lowstate_; }
    unitree_hg::msg::LowCmd& lowcmd() { return lowcmd_; }

    void publish_lowcmd();

private:
    void lowstate_callback(const unitree_hg::msg::LowState::SharedPtr msg);

    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_subscriber_;
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_publisher_;

    unitree_hg::msg::LowState lowstate_;
    unitree_hg::msg::LowCmd lowcmd_;

    void init_lowcmd_();

};

} // namespace legged
