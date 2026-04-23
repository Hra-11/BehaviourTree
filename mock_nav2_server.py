import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav2_msgs.action import NavigateToPose
import time

class MockNav2Server(Node):
    def __init__(self):
        super().__init__('mock_nav2_server')
        # 建立一个标准的 ROS 2 动作服务端，话题名为 /navigate_to_pose
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback
        )
        self.get_logger().info('导航服务端 (Mock Server) 已启动！等待接收行为树坐标...')

    def execute_callback(self, goal_handle):
        # 1. 提取行为树发来的坐标数据
        target_pose = goal_handle.request.pose.pose
        target_x = target_pose.position.x
        target_y = target_pose.position.y
        
        self.get_logger().info(f'[收到订单] 目标坐标: X={target_x:.2f}, Y={target_y:.2f}。准备出发！')
        
        # 2. 模拟物理世界中机器狗走路的耗时 (假装走了 4 秒)
        for i in range(1, 5):
            self.get_logger().info(f'正在向梅林区移动... 耗时 {i} 秒')
            time.sleep(1.0)
            
        # 3. 到达目标点，向行为树返回成功状态
        goal_handle.succeed()
        self.get_logger().info('[订单完成] 已到达目标点！')
        
        result = NavigateToPose.Result()
        return result

def main(args=None):
    rclpy.init(args=args)
    mock_server = MockNav2Server()
    rclpy.spin(mock_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
