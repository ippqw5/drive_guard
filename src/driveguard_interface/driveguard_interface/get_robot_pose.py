import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion
import threading
import time

class RobotPoseNode(Node):
    """
    RobotPoseNode is a ROS 2 node that provides functionality to retrieve the
    transform data between two coordinate frames using the tf2 library.

    Methods:
        __init__():
            Initializes the RobotPoseNode, setting up the TransformListener and
            buffer for handling coordinate frame transformations.
        get_transform_data(source_frame='map', target_frame='base_footprint'):
            Retrieves the transform data between the specified source and target
            frames. The transform data includes translation, rotation in quaternion
            format, and rotation in Euler angles (roll, pitch, yaw).
            Args:
                source_frame (str): The name of the source coordinate frame. Defaults to 'map'.
                target_frame (str): The name of the target coordinate frame. Defaults to 'base_footprint'.
            Returns:
                dict: A dictionary containing the translation, rotation in quaternion format,
                      and rotation in Euler angles if the transform is successfully retrieved.
                      Returns None if the transform cannot be retrieved.
            Raises:
                Logs a warning message if the transform cannot be retrieved, including the reason.
    """
    def __init__(self):
        super().__init__('robot_pose_node')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def get_transform_data(self, source_frame='map', target_frame='base_footprint'):
        """获取坐标变换数据"""
        try:
            tf = self.buffer.lookup_transform_full(
                target_frame=target_frame,
                target_time=rclpy.time.Time(),
                source_frame=source_frame,
                source_time=rclpy.time.Time(),
                fixed_frame='map',
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            transform = tf.transform
            rotation_euler = euler_from_quaternion([
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w
            ])
            
            return {
                'translation': {
                    'x': transform.translation.x,
                    'y': transform.translation.y,
                    'z': transform.translation.z
                },
                'rotation_quaternion': {
                    'x': transform.rotation.x,
                    'y': transform.rotation.y,
                    'z': transform.rotation.z,
                    'w': transform.rotation.w
                },
                'rotation_euler': {
                    'roll': rotation_euler[0],
                    'pitch': rotation_euler[1],
                    'yaw': rotation_euler[2]
                }
            }
        except Exception as e:
            self.get_logger().warn(f'不能够获取坐标变换，原因: {str(e)}')
            return None
        
class RobotPose:
    """
    RobotPose is a singleton class that provides an interface to interact with the robot's pose
    using ROS 2. It initializes a ROS 2 node and manages a single-threaded executor to handle
    transform data requests.
    Attributes:
        _instance (RobotPose): The singleton instance of the RobotPose class.
        _lock (threading.Lock): A threading lock to ensure thread-safe singleton initialization.
        _initialized (bool): A flag indicating whether the class has been initialized.
    Methods:
        __new__(cls):
            Ensures that only one instance of the RobotPose class is created (singleton pattern).
        __init__():
            Initializes the ROS 2 node, executor, and a background thread to spin the executor.
            Ensures that the initialization is performed only once.
        get_transform(source_frame='map', target_frame='base_footprint'):
            Retrieves the transform data between the specified source and target frames.
            Args:
                source_frame (str): The name of the source frame. Defaults to 'map'.
                target_frame (str): The name of the target frame. Defaults to 'base_footprint'.
            Returns:
                Transform data between the source and target frames.
        shutdown():
            Shuts down the ROS 2 executor, destroys the node, and cleans up resources.
            Ensures that the shutdown process is performed only if the class was initialized.
    """
    _instance = None
    _lock = threading.Lock()
    _initialized = False
    
    def __new__(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(RobotPose, cls).__new__(cls)
            return cls._instance
    
    def __init__(self):
        if RobotPose._initialized:
            return
        
        rclpy.init()
        self._node = RobotPoseNode()
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self._node)
        
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

        # 等待TF树准备就绪
        time.sleep(2.0)
        
        RobotPose._initialized = True
    
    def get_transform(self, source_frame='map', target_frame='base_footprint'):
        return self._node.get_transform_data(source_frame, target_frame)
    
    def shutdown(self):
        if not RobotPose._initialized:
            return
            
        self._executor.shutdown()
        self._node.destroy_node()
        rclpy.shutdown()
        self._thread.join()
        RobotPose._initialized = False
        
def main():
    rclpy.init()

    node = RobotPoseNode()
    transform_data = node.get_transform_data()
    if transform_data:
        print("Translation:", transform_data['translation'])
        print("Rotation (Quaternion):", transform_data['rotation_quaternion'])
        print("Rotation (Euler):", transform_data['rotation_euler'])
    else:
        print("Transform data not available.")

    rclpy.shutdown()


if __name__ == '__main__':
    main()