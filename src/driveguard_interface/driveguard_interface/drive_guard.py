from drive_guard_node import *
import rclpy
import time
import cv2
from cv_bridge import CvBridge
import transforms3d
import numpy as np

class DriveGuard():
    """DriveGuard wraps the DriveGuardNode functionality. 
    This class hides the complexity of the ROS2 node and provides a simple interface
    """

    def __init__(self):
        """Initialize the DriveGuard interface.
        This method initializes the ROS2 node and sets up the CvBridge for image conversion.
        It also waits for a short period to ensure that the node is ready to receive data.
        It is recommended to call this method before using any other methods in the class.
        """
        rclpy.init()
        self.node = DriveGuardNode()
        self.cv_bridge = CvBridge()
        time.sleep(2.0)

    def shutdown(self):
        """Shutdown the DriveGuard node and clean up resources.
        This method ensures that all resources are released and the node is properly shut down.
        """
        cv2.destroyAllWindows()  # Close any open OpenCV windows
        self.node.shutdown()

    def get_imu_data(self) -> Imu:
        return self.node.latest_imu_data
    
    def get_odom_data(self) -> Odometry:
        return self.node.latest_odom_data
    
    def get_raw_image(self) -> Image:
        return self.node.latest_raw_image

    def get_pose(self, base_frame_id='map', target_frame_id='base_footprint') -> TransformStamped:
        """Get the pose transformation from base_frame_id to target_frame_id.
        
        Args:
            base_frame_id (str): The base frame ID, default is 'map'.
            
            target_frame_id (str): The target frame ID, default is 'base_footprint'.
            
        Returns:
            TransformStamped: The transformation from base_frame_id to target_frame_id.
        """
        return self.node.get_pose(base_frame_id, target_frame_id)

    def display_image(self, window_name="Camera Image") -> bool:
        """Display the latest camera image using OpenCV
        
        Args:
            window_name (str): The name of the OpenCV window to display the image.
            
        Returns:
        
            bool: True if the image was displayed successfully, False otherwise.
        """
        raw_img = self.get_raw_image()
        if raw_img is None:
            print("No image received yet")
            return False

        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(
                raw_img, desired_encoding="bgr8")

            # Display the image
            cv2.imshow(window_name, cv_image)
            cv2.waitKey(1)  # Update the display, wait 1ms
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error displaying image: {e}")
            return False

    def get_twist(self) -> Twist:
        """Get the current twist of the robot in global frame (aka "map" frame).
        It uses the robot's odometry data and transforms it to the global frame.
        
        Returns:
            Twist: The velocity of the robot in the global frame.
        """
        try:
            # 1. Get the latest odometry data
            odom_data = self.get_odom_data()
            if odom_data is None:
                self.node.get_logger().warn("No odometry data available")
                return None
            
            # 2. Get the transform from 'map' to 'odom' frame
            transform = self.get_pose('map', 'odom')
            if transform is None:
                self.node.get_logger().warn("Transform from 'map' to 'odom' not available")
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
            self.node.get_logger().error(f"Unexpected error in get_twist: {e}")
            return None

    def set_vel(self, linear_x=0.0, linear_y=0.0, linear_z=0.0,
                        angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """Set velocity
        
        Args:
            linear_x (float): Linear velocity in x direction.
            
            linear_y (float): Linear velocity in y direction.
            
            linear_z (float): Linear velocity in z direction.
            
            angular_x (float): Angular velocity around x axis.
            
            angular_y (float): Angular velocity around y axis.
            
            angular_z (float): Angular velocity around z axis.
        """
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.linear.y = linear_y
        twist_msg.linear.z = linear_z
        twist_msg.angular.x = angular_x
        twist_msg.angular.y = angular_y
        twist_msg.angular.z = angular_z

        self.node.cmd_vel_publisher.publish(twist_msg)

    def stop_robot(self):
        """Stop the robot by publishing zero velocities
        """
        self.set_vel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

if __name__ == "__main__":
    drive_guard = DriveGuard()

    # Get IMU data
    imu_data = drive_guard.get_imu_data()
    if imu_data:
        # print(f"Type: {type(imu_data)}")
        print("IMU Data:", imu_data)
    else:
        print("No IMU data received yet")

    try:
        # Get speed and twist in target frame
        while True:
            time.sleep(0.5) 
            twist = drive_guard.get_twist()
            if twist:
                print(f"Twist: {twist}")
            else:
                print("No twist data available")

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received")
    finally:
        # Restore terminal and shutdown
        drive_guard.restore_terminal()
        drive_guard.stop_robot()
        drive_guard.shutdown()
