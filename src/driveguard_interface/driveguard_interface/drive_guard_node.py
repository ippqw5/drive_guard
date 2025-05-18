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

    def get_odom_data(self) -> Odometry:
        return self.latest_odom_data

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