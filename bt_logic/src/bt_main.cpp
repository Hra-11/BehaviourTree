#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h" // 引入终端可视化打印
#include "bt_logic/actions/nav2_client.hpp"

class MockDetectItem : public BT::SyncActionNode {
public:
    MockDetectItem(const std::string& name, const BT::NodeConfig& config) 
        : BT::SyncActionNode(name, config) {}

    // 声明接口契约：需要输入物体名字，输出坐标到黑板
    static BT::PortsList providedPorts() {
        return { 
            BT::InputPort<std::string>("item_name"), 
            BT::OutputPort<std::string>("output_pose") 
        };
    }

    BT::NodeStatus tick() override {
        std::string item;
        if (!getInput("item_name", item)) {
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[模拟视觉] 机器狗启动雷达与相机，开始扫描目标: [" << item << "]" << std::endl;
        
        // 2. 模拟经过了一通复杂的 AI 运算，得出了一个坐标
        std::string fake_pose = "X: 2.5, Y: 1.2, Yaw: 0.78";
        
        // 3. 【核心操作】把算出来的坐标狠狠地拍在黑板上！
        setOutput("output_pose", fake_pose);
        
        std::cout << "[模拟视觉] 锁定目标！坐标已写入黑板 -> " << fake_pose << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

class MockAlignToPose : public BT::SyncActionNode {
private:
    // 保存 ROS 2 节点的指针，用来发消息
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

public:
    // 构造函数：除了常规参数，现在需要接收 ROS 2 节点的指针
    MockAlignToPose(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node) 
        : BT::SyncActionNode(name, config), node_(node) {
        
        // 创建一个向 "/cmd_vel" 话题发送消息的发布器
        cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

    static BT::PortsList providedPorts() {
        return { 
            BT::InputPort<std::string>("target_pose"), 
            BT::InputPort<std::string>("tolerance") 
        };
    }

    BT::NodeStatus tick() override {
        std::string target_coords;
        if (!getInput("target_pose", target_coords)) {
            return BT::NodeStatus::FAILURE;
        }
        
        std::cout << "[执行层] 提取坐标 [" << target_coords << "]，向底盘发送移动指令..." << std::endl;


        geometry_msgs::msg::Twist twist;
        
        // 1. 让机器狗向前走 (X轴线速度 0.3 m/s)
        twist.linear.x = 0.3;
        twist.angular.z = 0.0;
        cmd_vel_pub_->publish(twist);
        
        std::cout << "机器狗正在前进！" << std::endl;
        

        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // 2. 到达位置，发送刹车指令 (速度归 0)
        twist.linear.x = 0.0;
        cmd_vel_pub_->publish(twist);
        
        std::cout << "对齐完成，刹车停稳！" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("robocon_bt_main_node");

    {
        BT::BehaviorTreeFactory factory;

        // 1. 注册我们已经写好的真实节点
        BT::NodeBuilder builder =
   			[ros_node](const std::string& name, const BT::NodeConfig& config)
		{
    		return std::make_unique<Nav2Client>(name, config, ros_node);
		};
		factory.registerBuilder<Nav2Client>("Nav2Client", builder);

        // 2. 架构师的进阶魔法：给替身节点临时声明端口！
        // 定义一个闭包，让这些替身直接返回成功
        auto ok = [](BT::TreeNode&){ return BT::NodeStatus::SUCCESS; };

        // 没有任何参数的节点，继续用原生的 AlwaysSuccessNode
        factory.registerNodeType<BT::AlwaysSuccessNode>("CheckSystemReady");
        factory.registerNodeType<BT::AlwaysSuccessNode>("WaitMatchStart");
        factory.registerNodeType<BT::AlwaysSuccessNode>("AssembleWeapon");
        // 有参数的节点，我们用 registerSimpleAction 给它们“贴上”对应的输入输出端口标签
        factory.registerNodeType<MockDetectItem>("DetectItem");
            
        factory.registerBuilder<MockAlignToPose>("AlignToPose", 
    		[ros_node](const std::string& name, const BT::NodeConfig& config) {
        		return std::make_unique<MockAlignToPose>(name, config, ros_node);
    	});
    	
        factory.registerSimpleAction("OperateGripper", ok, 
            {BT::InputPort<std::string>("action")});
            
        factory.registerSimpleAction("SetLocomotionGait", ok, 
            {BT::InputPort<std::string>("gait_type")});
            
        factory.registerSimpleAction("ExecuteGaitMacro", ok, 
            {BT::InputPort<std::string>("macro_name")});
            
        factory.registerSimpleAction("TrackEnemy", ok, 
            {BT::OutputPort<std::string>("output_enemy_pose"), BT::OutputPort<std::string>("output_enemy_state")});
        
        factory.registerSimpleAction("IsEnemyInRange", ok, 
            {BT::InputPort<std::string>("target_pose"), BT::InputPort<std::string>("attack_radius")});
            
        factory.registerSimpleAction("ExecuteAttack", ok, 
            {BT::InputPort<std::string>("target_pose")});
            
        factory.registerSimpleAction("DynamicPursuit", ok,
        	{BT::InputPort<std::string>("target_pose"),BT::InputPort<std::string>("optimal_distance")});
        // 3. 从刚刚建好的 XML 文件中加载真正的图纸
        std::string xml_file_path = "/home/hra/robocon_dev_ws/ros2_ws/src/bt_logic/bt_xml/main_tree.xml";
        auto tree = factory.createTreeFromFile(xml_file_path);

        // 4. 添加一个日志记录器，让终端打印出酷炫的树形执行过程

        std::cout << "======================================" << std::endl;
        std::cout << " 行为树主树 挂载成功" << std::endl;
        std::cout << "======================================" << std::endl;
		
		rclcpp::WallRate loop_rate(10);
        // 执行大树
        while (rclcpp::ok()) {
        
		    // 1. 【核心机制】让 ROS 2 处理一次底层的收发信件（触发回调函数！）
		    rclcpp::spin_some(ros_node); 
		    
		    // 2. 让行为树主干往前推进一次状态
		    BT::NodeStatus status = tree.tickExactlyOnce(); 
		    
		    // 3. 如果整棵大树执行完毕（成功或彻底失败），则退出循环
		    if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
		        std::cout << "行为树执行完毕，最终状态: " << status << std::endl;
		        break;
		    }
		    // 4. 稍微休息一下
		    loop_rate.sleep();
		}

    // 👆 替换到这里结束 👆

    rclcpp::shutdown();
    return 0;
	}
}
