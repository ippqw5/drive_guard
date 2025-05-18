
from drive_guard_node import DriveGuardNode
import rclpy
import time
import cv2
from cv_bridge import CvBridge


class DriveGuard():
    """
    DriveGuard wraps the DriveGuardNode functionality.

    This class hides the complexity of the ROS2 node and provides a simple interface
    """
    def __init__(self):
        rclpy.init()
        self.node = DriveGuardNode()
        self.cv_bridge = CvBridge()
        time.sleep(2.0)

    def get_imu_data(self):
        return self.node.get_imu_data()
        
    def get_pose(self, base_frame_id='map', target_frame_id='base_footprint'):
        return self.node.get_pose(base_frame_id, target_frame_id)
    
    def get_raw_image(self):
        return self.node.get_raw_image()
    
    def display_image(self, window_name="Camera Image"):
        """Display the latest camera image using OpenCV"""
        raw_img = self.get_raw_image()
        if raw_img is None:
            print("No image received yet")
            return False
            
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(raw_img, desired_encoding="bgr8")
            
            # Display the image
            cv2.imshow(window_name, cv_image)
            cv2.waitKey(1)  # Update the display, wait 1ms
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error displaying image: {e}")
            return False

    def shutdown(self):
        cv2.destroyAllWindows()  # Close any open OpenCV windows
        self.node.shutdown()

    def get_speed(self) -> float:
        '''
        Get the current speed(标量) of the robot.
        '''
        try:
            odom_data = self.node.get_odom_data()
            if odom_data is None:
                self.get_logger().warn("not receive odom_data，speed return 0.0")
                return 0.0
            linear = odom_data.twist.twist.linear
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
            transform = self.node.get_pose("odom", target_frame_id)
            
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

            odom_data = self.node.get_odom_data()
            
            # 转换线速度
            v_odom = np.array([odom_data.twist.twist.linear.x, odom_data.twist.twist.linear.y, odom_data.twist.twist.linear.z])
            v_base_link = R_mat @ v_odom  # 矩阵乘法
            
            # 角速度在 target_frame_id 和 odom 下相同（假设 2D 平面运动）
            w_base_link = odom_data.twist.twist.angular
            
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


if __name__ == "__main__":
    drive_guard = DriveGuard()
    
    # Get IMU data
    imu_data = drive_guard.get_imu_data()
    if imu_data:
        # print(f"Type: {type(imu_data)}")
        print("IMU Data:", imu_data)
    else:
        print("No IMU data received yet")

    # Get pose transformation
    pose = drive_guard.get_pose()
    if pose:
        print("Pose Transformation:", pose)
    else:
        print("Pose transformation not available")

    # Display the camera image
    if drive_guard.display_image():
        print("Image displayed. Press any key in the image window to continue.")
        cv2.waitKey(0)  # Wait until a key is pressed

    # Get speed and twist in target frame
    while True:
        time.sleep(2)

        twist_target_frame = drive_guard.get_twist_target_frame()
        if twist_target_frame:
            print("Twist in Target Frame:", twist_target_frame)
        
        speed = drive_guard.get_speed()
        if speed is not None:
            print("Current speed:", speed)
    
    # Shutdown the node
    drive_guard.shutdown()