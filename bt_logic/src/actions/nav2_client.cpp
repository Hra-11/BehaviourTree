#include "bt_logic/actions/nav2_client.hpp"

Nav2Client::Nav2Client(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node), goal_reached_(false) 
{
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");
}

BT::PortsList Nav2Client::providedPorts() {
    return { BT::InputPort<std::string>("target_pose") };
}

BT::NodeStatus Nav2Client::onStart() {
    std::string pose_str;
    if (!getInput("target_pose", pose_str)) {
        std::cout << "[Nav2Client] 缺少目标坐标" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(3))) {
        std::cout << "[Nav2Client] 导航底层未启动" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    

goal_msg.pose.pose.position.x = 2.5; 
    goal_msg.pose.pose.position.y = 1.2;

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                this->goal_reached_ = true;
            }
        };

    action_client_->async_send_goal(goal_msg, send_goal_options);
    goal_reached_ = false;

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Nav2Client::onRunning() {
    if (goal_reached_) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void Nav2Client::onHalted() {
    std::cout << "[Nav2Client] 动作被打断！" << std::endl;
    action_client_->async_cancel_all_goals();
}
