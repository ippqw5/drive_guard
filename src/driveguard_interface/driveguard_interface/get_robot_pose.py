import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion
import threading
import time

class RobotPose:
    """封装机器人位姿获取功能，对外提供简单接口"""
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


class RobotPoseNode(Node):
    """机器人位姿节点，负责与TF系统交互获取位姿数据"""
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


# 仅当作为独立脚本运行时执行
def main():
    rclpy.init()
    node = RobotPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()