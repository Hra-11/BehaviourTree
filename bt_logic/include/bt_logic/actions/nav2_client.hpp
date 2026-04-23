#ifndef NAV2_CLIENT_HPP
#define NAV2_CLIENT_HPP

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

class Nav2Client : public BT::StatefulActionNode {
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    bool goal_reached_;

public:
    // 1. 构造函数声明 (包含 3 个参数)
    Nav2Client(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

    // 2. 端口声明
    static BT::PortsList providedPorts();

    // 3. 状态机函数声明 (替代传统的 tick)
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

#endif
