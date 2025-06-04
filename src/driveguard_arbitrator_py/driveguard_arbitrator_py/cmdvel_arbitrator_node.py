#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.duration import Duration
from action_msgs.msg import GoalStatusArray

class CmdVelArbitratorNode(Node):
    def __init__(self):
        super().__init__('driveguard_arbitrator_node')
        
        # 发布器
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # 订阅器
        self.model_sub = self.create_subscription(
            Twist, '/cmd_vel_model', self.model_callback, 10)
        self.safety_sub = self.create_subscription(
            Twist, '/cmd_vel_safe', self.safety_callback, 10)
        
        # 定时器 (100ms)
        self.timer = self.create_timer(0.1, self.publish_muxed_cmd)
        
        # 初始化变量
        self.latest_model = Twist()
        self.latest_safety = Twist()
        self.last_model_time = self.get_clock().now() - Duration(seconds=10.0)
        self.vehicle_return = False
        
        # 初始化目标位置
        self.target = PoseStamped()
        self.target.header.frame_id = 'map'
        self.target.pose.position.x = 0.0
        self.target.pose.position.y = 0.0
        self.target.pose.position.z = 0.0
        self.target.pose.orientation.x = 0.0
        self.target.pose.orientation.y = 0.0
        self.target.pose.orientation.z = 0.0
        self.target.pose.orientation.w = 1.0
    
    def model_callback(self, msg):
        self.latest_model = msg
        self.last_model_time = self.get_clock().now()
    
    def safety_callback(self, msg):
        self.latest_safety = msg
    
    def publish_muxed_cmd(self):
        self.target.header.stamp = self.get_clock().now().to_msg()
        timeout = Duration(seconds=0.5)
        now = self.get_clock().now()
        
        if (now - self.last_model_time) < timeout:
            self.vehicle_return = False
            self.cmd_pub.publish(self.latest_model)
        else:
            if not self.vehicle_return:
                self.pose_pub.publish(self.target)
                self.vehicle_return = True
            self.cmd_pub.publish(self.latest_safety)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelArbitratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()