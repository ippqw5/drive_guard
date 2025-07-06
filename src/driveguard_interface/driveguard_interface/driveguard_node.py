import threading
import numpy as np
import transforms3d
import time
import torch
import cv2
import cv_bridge

import rclpy
from rclpy.node import Node
from tf2_ros import *
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import Pose, Transform, Twist, Vector3, Accel


class DriveGuardNode(Node):
    """
    DriveGuardNode serves as the main interface for ROS2 communication.
    
    It subscribes to sensor data and publishes commands to the robot.

    This class should be singleton in the application, as it manages the ROS2 node lifecycle. 
    """
    def __init__(self):
        rclpy.init()
        super().__init__('drive_guard_node')

        self.imu_topic_name = '/imu'
        self.camera_topic_name = '/camera_sensor/image_raw'
        self.odom_topic_name = '/odom'
        self.cmd_vel_topic_name = '/cmd_vel'
        self.cmd_vel_ai_topic_name = '/cmd_vel_ai'

        self.latest_imu_data : Imu = None
        self.latest_raw_image : Image = None
        self.latest_odom_data : Odometry = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Initialize subscribers for IMU data, raw image, and tf tree
        self.create_subscription(
            Imu,
            self.imu_topic_name,
            self._imu_callback,
            10
        )
        self.create_subscription(
            Image,
            self.camera_topic_name,
            self._image_callback,
            10
        )
        self.create_subscription(
            Odometry,
            self.odom_topic_name,
            self._odom_callback,
            10
        )
        
        # Initialize pushlishers for cmd_vel if needed
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic_name,
            10
        )

        self.cmd_vel_ai_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_ai_topic_name,
            10
        )

        # [Must] Spin the node in a separate thread
        self.spin_thread = threading.Thread(target=self._spin_node)
        self.spin_thread.daemon = True
        self.spin_thread.start()

        self._verify_topics()
        time.sleep(1.0)
        self.get_logger().warn("DriveGuardNode initialized and ready to use")
        
    def _image_callback(self, msg):
        self.latest_raw_image = msg

    def _imu_callback(self, msg):
        self.latest_imu_data = msg
    
    def _odom_callback(self, msg):
        self.latest_odom_data = msg    
                
    def _spin_node(self):
        rclpy.spin(self)
        
    def _verify_topics(self):
        """Verify that the topics we're subscribing to exist"""
        from rclpy.topic_endpoint_info import TopicEndpointTypeEnum
        
        # Get list of publishers
        publishers = self.get_publishers_info_by_topic(self.imu_topic_name)
        imu_exists = len(publishers) > 0

        publishers = self.get_publishers_info_by_topic(self.camera_topic_name)
        camera_exists = len(publishers) > 0

        publishers = self.get_publishers_info_by_topic(self.odom_topic_name)
        odom_exists = len(publishers) > 0
        
        self.get_logger().warn(f"IMU topic exists: {imu_exists}")
        self.get_logger().warn(f"Camera topic exists: {camera_exists}")
        self.get_logger().warn(f"Odom topic exists: {odom_exists}")

        return imu_exists, camera_exists, odom_exists
    
####################################################################
####################################################################
####################################################################

    def stop(self):
        self.set_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    
    def shutdown(self):
        self.get_logger().warn("Destroying")
        self.stop()
        rclpy.try_shutdown()
        if self.spin_thread.is_alive():
            self.spin_thread.join()
        super().destroy_node()        
        self.get_logger().warn("Destroyed")

####################################################################
####################################################################
####################################################################

    def get_odom_data(self) -> Odometry:
        if self.latest_odom_data is None:
            self.get_logger().warn("No odometry data available")
            return None
        return self.latest_odom_data
    
    def get_raw_image(self) -> Image:
        if self.latest_raw_image is None:
            self.get_logger().warn("No raw image data available")
            return None
        return self.latest_raw_image

    def get_image_tensor(self, resize: tuple = (224,224), normalize: bool = True, cuda: bool = True) -> torch.Tensor:
        image = self.get_raw_image()
        if image is None:
            return None

        # Convert ROS Image message to OpenCV format
        cv_image = cv_bridge.CvBridge().imgmsg_to_cv2(image, desired_encoding='rgb8')

        if resize:
            cv_image = cv2.resize(cv_image, (resize[0], resize[1]))

        image_tensor = torch.from_numpy(cv_image).permute(2, 0, 1)  # Change to CxHxW format

        if normalize:
            image_tensor = image_tensor.float() / 255.0  # Normalize to [0, 1]

        image_tensor = image_tensor.unsqueeze(0)  # Add batch dimension
    
        if cuda:
            image_tensor = image_tensor.cuda()

        return image_tensor

    def get_imu_data(self) -> Imu:
        if self.latest_imu_data is None:
            self.get_logger().warn("No IMU data available")
            return None
        return self.latest_imu_data

    def get_transform(self, base_frame_id='map', target_frame_id='base_footprint') -> TransformStamped:
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
    
    def get_linear_acceleration(self) -> Vector3:
        """
        Get the linear acceleration from the latest IMU data.
        """        
        imu_data = self.get_imu_data()
        if imu_data is None:
            return None
        
        acceleration = Vector3()
        acceleration.x = imu_data.linear_acceleration.x
        acceleration.y = imu_data.linear_acceleration.y
        acceleration.z = imu_data.linear_acceleration.z

        return acceleration

    def get_angular_acceleration(self) -> Vector3:
        """
        Get the angular acceleration by differentiating angular velocity from IMU data.
        """
        current_imu = self.get_imu_data()
        if current_imu is None:
            return None
        
        # 需要存储上一次的角速度数据来计算角加速度
        if not hasattr(self, 'previous_imu_data') or self.previous_imu_data is None:
            self.previous_imu_data = current_imu
            return None
        
        # 计算时间差
        current_time = current_imu.header.stamp.sec + current_imu.header.stamp.nanosec * 1e-9
        previous_time = self.previous_imu_data.header.stamp.sec + self.previous_imu_data.header.stamp.nanosec * 1e-9
        dt = current_time - previous_time
        
        if dt <= 0:
            return None
        
        # 计算角加速度 = (当前角速度 - 上一次角速度) / 时间差
        angular_accel = Vector3()
        angular_accel.header = current_imu.header
        angular_accel.vector.x = (current_imu.angular_velocity.x - self.previous_imu_data.angular_velocity.x) / dt
        angular_accel.vector.y = (current_imu.angular_velocity.y - self.previous_imu_data.angular_velocity.y) / dt
        angular_accel.vector.z = (current_imu.angular_velocity.z - self.previous_imu_data.angular_velocity.z) / dt
        
        # 更新历史数据
        self.previous_imu_data = current_imu
        
        return angular_accel

    def get_pose(self) -> Pose:
        """
        Get the pose of the car in the global coordinate.
            
        Returns:
            Pose: The pose of the car in the global coordinate.
        """

        transform = self.get_transform('map', 'base_link')
        if transform is None:
            self.get_logger().warn("No pose data available")
            return None

        # Convert the pose to a Pose message
        global_pose = Pose()
        global_pose.position.x = transform.transform.translation.x
        global_pose.position.y = transform.transform.translation.y
        global_pose.position.z = transform.transform.translation.z
        return global_pose

    def get_twist(self, is_global=True) -> Twist:
        """
        Get the current twist of the robot in global frame (aka "map" frame).
        
        It uses the robot's odometry data and transforms it to the global frame.
        
        Args:
            is_global (bool): If True, returns the velocity in the global frame (map).
                              If False, returns the velocity in the local frame (base_link).
        
        Returns:
            Twist: The linear/angular velocity of the car.
        """
        try:
            # 1. Get the latest odometry data
            odom_data = self.get_odom_data()
            if odom_data is None:
                self.get_logger().warn("No odometry data available")
                return None
            
            # 2. Get the transform 
            if is_global:
                transform = self.get_transform('map', 'base_link')
                if transform is None:
                    self.get_logger().warn("Transform from 'map' to 'base_link' not available")
                    return None
            else:
                transform = self.get_transform('map', 'odom')
                if transform is None:
                    self.get_logger().warn("Transform from 'map' to 'odom' not available")
                    return None
                
            # 3. Extract quaternion and convert to rotation matrix using transforms3d
            quat = transform.transform.rotation
            # transforms3d uses (w, x, y, z) order for quaternions
            rotation_matrix = transforms3d.quaternions.quat2mat([quat.w, quat.x, quat.y, quat.z])
            
            # 4. Get linear velocity from odometry (in base_link frame)
            linear_vel_local = np.array([
                odom_data.twist.twist.linear.x,
                odom_data.twist.twist.linear.y,
                odom_data.twist.twist.linear.z
            ])
            
            # 5. Transform linear velocity to map frame
            linear_vel_global = rotation_matrix @ linear_vel_local
            
            # 6. Angular velocity remains the same (assuming it's about z-axis)
            angular_vel = odom_data.twist.twist.angular
            
            # 7. Create and return the global twist
            global_twist = Twist()
            global_twist.linear.x = float(linear_vel_global[0])
            global_twist.linear.y = float(linear_vel_global[1])
            global_twist.linear.z = float(linear_vel_global[2])
            global_twist.angular.x = angular_vel.x
            global_twist.angular.y = angular_vel.y
            global_twist.angular.z = angular_vel.z
            
            return global_twist
            
        except Exception as e:
            self.get_logger().error(f"Unexpected error in get_vel: {e}")
            return None

    def get_linear_velocity(self, is_global=True) -> Vector3:
        """
        Get the linear velocity vector (similar to CARLA's get_velocity).
        
        Args:
            is_global (bool): If True, returns velocity in global frame (map).
                            If False, returns velocity in local frame (base_link).
        
        Returns:
            Vector3: Linear velocity vector in m/s
        """
        twist = self.get_twist(is_global)
        if twist is None:
            return None
        
        # 提取线速度分量（类似 CARLA 的 Vector3D）
        velocity = Vector3()
        velocity.x = twist.linear.x  # m/s
        velocity.y = twist.linear.y  # m/s
        velocity.z = twist.linear.z  # m/s

        return velocity

    def get_angular_velocity(self, is_global=True) -> Vector3:
        """
        Get the angular velocity vector (similar to CARLA's get_angular_velocity).
        
        Args:
            is_global (bool): If True, returns angular velocity in global frame (map).
                            If False, returns angular velocity in local frame (base_link).
        
        Returns:
            Vector3: Angular velocity vector in deg/s
        """
        twist = self.get_twist(is_global)
        if twist is None:
            return None
        
        # 提取角速度分量（类似 CARLA 的 Vector3D）
        angular_velocity = Vector3()
        angular_velocity.x = twist.angular.x
        angular_velocity.y = twist.angular.y
        angular_velocity.z = twist.angular.z
        return angular_velocity

####################################################################
####################################################################
####################################################################

    def set_twist(self, twist: Twist):
        """
        Set twist under the local frame(base_link).

        Args:
            twist (Twist): The twist message containing linear and angular velocities.
        """
        if twist is None:
            self.get_logger().warn("Received None twist message, not publishing")
            return
        self.cmd_vel_publisher.publish(twist)
