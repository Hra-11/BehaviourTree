#pragma once
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class UpstairsClient : public BT::StatefulActionNode {
public:
    // 构造函数
    UpstairsClient(
        const std::string& name, 
        const BT::NodeConfig& config,
        rclcpp::Node::SharedPtr node
    ) : BT::StatefulActionNode(name, config), node_(node) {
        // 创建发布器和订阅器
        upstairs_pub_ = node_->create_publisher<std_msgs::msg::String>("upstairs/command", 10);
        upstairs_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "upstairs/feedback",
            10,
            std::bind(&UpstairsClient::feedbackCallback, this, std::placeholders::_1)
        );
    }
    
    // 定义端口
    static BT::PortsList providedPorts() {
        return { 
            BT::InputPort<std::string>("target_area"),  // 目标区域
            BT::OutputPort<std::string>("status")        // 输出状态
        };
    }
    
    // 节点开始执行
    BT::NodeStatus onStart() override {
        std::string target;
        if (!getInput("target_area", target)) {
            RCLCPP_ERROR(node_->get_logger(), "未指定目标区域");
            return BT::NodeStatus::FAILURE;
        }
        
        RCLCPP_INFO(node_->get_logger(), "开始上台阶任务，目标区域: %s", target.c_str());
        
        // 发布命令消息
        std_msgs::msg::String msg;
        msg.data = "START:" + target;
        upstairs_pub_->publish(msg);
        
        // 重置状态
        task_completed_ = false;
        task_success_ = false;
        
        return BT::NodeStatus::RUNNING;
    }
    
    // 节点运行中
    BT::NodeStatus onRunning() override {
        if (task_completed_) {
            if (task_success_) {
                RCLCPP_INFO(node_->get_logger(), "上台阶任务完成");
                setOutput("status", "COMPLETED");
                return BT::NodeStatus::SUCCESS;
            } else {
                RCLCPP_ERROR(node_->get_logger(), "上台阶任务失败");
                setOutput("status", "FAILED");
                return BT::NodeStatus::FAILURE;
            }
        }
        
        // 如果还没完成，继续运行
        RCLCPP_DEBUG(node_->get_logger(), "上台阶任务进行中...");
        return BT::NodeStatus::RUNNING;
    }
    
    // 必须实现：节点被中断
    void onHalted() override {
        // 发送停止命令
        std_msgs::msg::String msg;
        msg.data = "STOP";
        upstairs_pub_->publish(msg);
        
        RCLCPP_WARN(node_->get_logger(), "上台阶任务被中断");
    }
    
private:
    // 反馈消息回调
    void feedbackCallback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(node_->get_logger(), "收到反馈: %s", msg->data.c_str());
        
        if (msg->data == "SUCCESS") {
            task_success_ = true;
            task_completed_ = true;
        } else if (msg->data == "FAILURE") {
            task_success_ = false;
            task_completed_ = true;
        }
        // 其他消息（如"RUNNING"）不做处理，保持运行状态
    }
    
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr upstairs_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr upstairs_sub_;
    std::atomic<bool> task_completed_{false};
    std::atomic<bool> task_success_{false};
};