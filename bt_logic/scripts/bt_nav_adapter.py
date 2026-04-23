import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
import math
import time

class BtNavAdapter(Node):
    def __init__(self):
        super().__init__('bt_nav_adapter')
        
        # 1. 建立标准的 Action Server，对接行为树
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            self.execute_callback,
            cancel_callback=self.cancel_callback
        )
        # 2. 修改发布者：给队友的 A* 算法发触发信号（起跑枪）
        self.trigger_pub = self.create_publisher(String, '/task/trigger', 10)
        
        # 3. 增加订阅者：偷看 A* 算法算出来的路径，为了知道终点在哪
        self.path_sub = self.create_subscription(Path, '/planning/path', self.path_callback, 10)
        self.target_x = None
        self.target_y = None
        
        # 4. 订阅机器狗当前真实位置
        self.odom_sub = self.create_subscription(Odometry, '/odom_world', self.odom_callback, 10)
        self.current_x = 0.0
        self.current_y = 0.0
        # 打断刹车用
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.target_distance_threshold = 1.0 # 到达判定误差(米)
        self.get_logger().info('行为树导航适配器 (梅林 A* 定制版) 已启动！监听 /navigate_to_pose...')
        
        # 新增：空路径检测标志
        self.received_empty_path = False 
        self.target_distance_threshold = 0.8 
        self.get_logger().info('行为树导航适配器 (终极防死锁版) 已启动！监听 /navigate_to_pose...')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
    def path_callback(self, msg):
        """抓取队友规划出的路径的最后一个点，作为终点目标"""
        if len(msg.poses) > 0:
            last_pose = msg.poses[-1].pose.position
            self.target_x = last_pose.x
            self.target_y = last_pose.y
            self.received_empty_path = False
        else:
            # 记录 A* 算法罢工了
            self.received_empty_path = True


    def cancel_callback(self, goal_handle):
        self.get_logger().warn('收到行为树紧急打断指令 (Cancel)!')
        # 强行刹车
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        return CancelResponse.ACCEPT
    def execute_callback(self, goal_handle):
        self.get_logger().info('[收到行为树指令] 正在处理导航请求...')

        # ==========================================
        # 🛡️ 架构师核心护盾：越界免检机制
        # 你的狗走到终点时 X 已经到了 7.37。
        # 如果 X > 7.0，说明早就已经安全穿出梅林区了！
        # 此时拒绝对接梅林 A*，直接向行为树放行，让它赶紧去跑后面的节点！
        # ==========================================
        if self.current_x > 7.0:
            self.get_logger().info('机器狗已彻底穿过梅林区，不再重复规划，直接放行！')
            goal_handle.succeed()
            return NavigateToPose.Result()
        self.target_x = None
        self.target_y = None
        self.received_empty_path = False
        empty_path_duration = 0.0

        trigger_msg = String()
        trigger_msg.data = "start_planning"
        self.trigger_pub.publish(trigger_msg)
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()

            if self.received_empty_path:
                # 如果收到空路径，不锁死线程，而是累加时间
                empty_path_duration += 0.1
                self.get_logger().warn(f'A* 抛出空路径，正在缓冲等待... ({empty_path_duration:.1f}s)')
                
                if empty_path_duration >= 1.2:
                    self.get_logger().error('停止：A* 算法持续 1.2 秒未恢复，确认死锁，中断导航！')
                    goal_handle.abort()
                    return NavigateToPose.Result()
            else:
                # 一旦收到正常路径，立刻清零计时器！
                empty_path_duration = 0.0

            # 正常距离检测
            if self.target_x is not None and self.target_y is not None:
                distance = math.hypot(self.target_x - self.current_x, self.target_y - self.current_y)
                
                if distance <= self.target_distance_threshold:
                    self.get_logger().info('机器狗已抵达梅林区终点！向行为树汇报成功！')
                    goal_handle.succeed()
                    return NavigateToPose.Result()

            time.sleep(0.1)            
            
            
def main(args=None):
    rclpy.init(args=args)
    adapter = BtNavAdapter()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    rclpy.spin(adapter, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

