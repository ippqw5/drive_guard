import rclpy
import tf2_ros
import threading
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
import numpy as np

class DriveGuardNode(Node):
    def __init__(self):
        super().__init__('drive_guard_node')
        self.latest_imu_data : Imu = None
        self.latest_raw_image : Image = None
        self.latest_odom_data : Odometry = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Initialize subscribers for IMU data, raw image, and tf tree
        self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # [Must] Spin the node in a separate thread 
        self.spin_thread = threading.Thread(target=self._spin_node)
        self.spin_thread.daemon = True
        self.spin_thread.start()

        self._verify_topics()

    def _spin_node(self):
        rclpy.spin(self)
        
    def image_callback(self, msg):
        self.latest_raw_image = msg

    def imu_callback(self, msg):
        self.latest_imu_data = msg
    
    def odom_callback(self, msg):
        self.latest_odom_data = msg
    
    def get_imu_data(self) -> Imu:
        return self.latest_imu_data

    def get_raw_image(self) -> Image:
        return self.latest_raw_image

    def get_pose(self, base_frame_id='map', target_frame_id='base_footprint') -> TransformStamped:
        try:
            # Check if transform exists first
            if not self.tf_buffer.can_transform(target_frame_id, base_frame_id, rclpy.time.Time()):
                self.get_logger().warn(f"Transform from {base_frame_id} to {target_frame_id} not available")
                return None
                
            transform = self.tf_buffer.lookup_transform(
                target_frame_id,
                base_frame_id,
                rclpy.time.Time()
            )
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Transform error: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"Unexpected error in get_pose: {e}")
            return None
        
    def get_speed(self) -> float:
        '''
        Get the current speed(标量) of the robot.
        '''
        try:
            if self.latest_odom_data is None:
                self.get_logger().warn("not receive odom_data，speed return 0.0")
                return 0.0
            linear = self.latest_odom_data.twist.twist.linear
            speed = (linear.x ** 2 + linear.y ** 2 + linear.z ** 2) ** 0.5
            return speed
        except Exception as e:
            self.get_logger().error(f"Unexpected error in get_speed: {e}")
            return None

    def get_twist_target_frame(self, target_frame_id='base_footprint') -> Twist:
        '''
        Get the current twist of the robot in the target frame.
        '''
        try:
            if not self.tf_buffer.can_transform(target_frame_id, 'odom', rclpy.time.Time()):
                self.get_logger().warn(f"Transform from 'odom' to {target_frame_id} not available")
                return None
            # 获取 odom → target_frame_id 的变换
            transform = self.tf_buffer.lookup_transform(
                target_frame_id,
                'odom',
                rclpy.time.Time()
            )
            
            # 提取旋转角度（yaw）
            q = transform.transform.rotation
            quat = [q.x, q.y, q.z, q.w]
            r = R.from_quat(quat)
            roll, pitch, yaw = r.as_euler('xyz')
            
            # 构建旋转矩阵（仅绕 Z 轴）
            R_mat = np.array([
                [np.cos(yaw), -np.sin(yaw), 0],
                [np.sin(yaw),  np.cos(yaw), 0],
                [0,            0,           1]
            ])
            
            # 转换线速度
            v_odom = np.array([self.latest_odom_data.twist.twist.linear.x, self.latest_odom_data.twist.twist.linear.y, self.latest_odom_data.twist.twist.linear.z])
            v_base_link = R_mat @ v_odom  # 矩阵乘法
            
            # 角速度在 target_frame_id 和 odom 下相同（假设 2D 平面运动）
            w_base_link = self.latest_odom_data.twist.twist.angular
            
            # 发布转换后的速度
            twist_base_link = Twist()
            twist_base_link.linear.x = v_base_link[0]
            twist_base_link.linear.y = v_base_link[1]
            twist_base_link.linear.z = v_base_link[2]
            twist_base_link.angular = w_base_link
            return twist_base_link
        
        except Exception as e:
            self.get_logger().error(f"Unexpected error in get_twist_target_frame: {e}")
            return None

    def shutdown(self):
        self.get_logger().info("Shutting down DriveGuardNode:")
        rclpy.try_shutdown()
        if self.spin_thread.is_alive():
            self.spin_thread.join()
        super().destroy_node()

    # def shutdown(self):
    #     super().destroy_node()
    #     rclpy.shutdown()

    def _verify_topics(self):
        """Verify that the topics we're subscribing to exist"""
        from rclpy.topic_endpoint_info import TopicEndpointTypeEnum
        
        # Get list of publishers
        publishers = self.get_publishers_info_by_topic('/imu')
        imu_exists = len(publishers) > 0
        
        publishers = self.get_publishers_info_by_topic('/camera/image_raw')
        camera_exists = len(publishers) > 0

        publishers = self.get_publishers_info_by_topic('/odom')
        odom_exists = len(publishers) > 0
        
        self.get_logger().info(f"IMU topic exists: {imu_exists}")
        self.get_logger().info(f"Camera topic exists: {camera_exists}")
        self.get_logger().info(f"Odom topic exists: {odom_exists}")
        
        return imu_exists, camera_exists