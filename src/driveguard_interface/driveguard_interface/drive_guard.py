
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

    while True:
        time.sleep(2)

        twist_target_frame = drive_guard.node.get_twist_target_frame()
        if twist_target_frame:
            print("Twist in Target Frame:", twist_target_frame)
        
        speed = drive_guard.node.get_speed()
        if speed is not None:
            print("Current speed:", speed)
    
    # Shutdown the node
    drive_guard.shutdown()