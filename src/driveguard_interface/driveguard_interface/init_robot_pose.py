from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
import threading

class RobotInitialPose(PoseStamped):
    """
    RobotInitialPose is a class that extends the PoseStamped class to represent the initial pose of a robot.

    Attributes:
        header.frame_id (str): The reference frame for the pose, defaulting to 'map'.

    Methods:
        set_position(x: float, y: float, z: float = 0.0):
            Sets the position of the robot in 3D space.
            Args:
                x (float): The x-coordinate of the position.
                y (float): The y-coordinate of the position.
                z (float, optional): The z-coordinate of the position. Defaults to 0.0.
        set_orientation(x: float, y: float, z: float, w: float):
            Sets the orientation of the robot using a quaternion.
            Args:
                x (float): The x-component of the quaternion.
                y (float): The y-component of the quaternion.
                z (float): The z-component of the quaternion.
                w (float): The w-component of the quaternion.
    """
    def __init__(self):
        super().__init__()
        self.header.frame_id = 'map'
    
    def set_position(self, x, y, z=0.0):
        self.pose.position.x = x
        self.pose.position.y = y
        self.pose.position.z = z

    def set_orientation(self, x, y, z, w):
        self.pose.orientation.x = x
        self.pose.orientation.y = y
        self.pose.orientation.z = z
        self.pose.orientation.w = w 

class RobotNavigator:
    """
    RobotNavigator is a singleton class that provides an interface for initializing and managing
    a robot's navigation system using ROS 2 and the Nav2 stack.

    Attributes:
        _instance (RobotNavigator): The singleton instance of the class.
        _lock (threading.Lock): A threading lock to ensure thread-safe instantiation.
        _initialized (bool): A flag indicating whether the class has been initialized.

    Methods:
        __new__(cls):
            Ensures that only one instance of the class is created (singleton pattern).

        __init__():
            Initializes the ROS 2 context and the BasicNavigator instance if not already initialized.

        set_initial_pose(pose: RobotInitialPose):
            Sets the initial pose of the robot in the navigation system.
            Args:
                pose (RobotInitialPose): The initial pose of the robot.

        wait_until_nav2_active():
            Waits until the Nav2 navigation stack becomes active.

        shutdown():
            Shuts down the ROS 2 context and resets the initialization flag.
    """
    _instance = None
    _lock = threading.Lock()
    _initialized = False

    def __new__(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(RobotNavigator, cls).__new__(cls)
            return cls._instance

    def __init__(self):
        if RobotNavigator._initialized:
            return

        rclpy.init()
        self.navigator = BasicNavigator()
        RobotNavigator._initialized = True

    def set_initial_pose(self, pose : RobotInitialPose):
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.navigator.setInitialPose(pose)
        self.wait_until_nav2_active()

    def wait_until_nav2_active(self):
        self.navigator.waitUntilNav2Active()

    def shutdown(self):
        if not RobotNavigator._initialized:
            return
        rclpy.shutdown()
        RobotNavigator._initialized = False
        

def main():
    rclpy.init()
    
    robot_initial_pose = RobotInitialPose()
    robot_initial_pose.set_position(0.0, 0.0)
    robot_initial_pose.set_orientation(0.0, 0.0, 0.0, 1.0)

    navigator_node = BasicNavigator()
    navigator_node.setInitialPose(robot_initial_pose)
    navigator_node.waitUntilNav2Active()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
