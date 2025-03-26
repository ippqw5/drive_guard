import sys
import os
import time
# 添加父目录，从而可以导入driveguard_interface
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from driveguard_interface import RobotPose

# 获取机器人位姿
robot_pose_node = RobotPose()

# 获取机器人位姿 10次
for i in range(10):
    pose_data = robot_pose_node.get_transform()
    time.sleep(0.5)
    print(f"机器人位置: x={pose_data['translation']['x']}, y={pose_data['translation']['y']}")
    print(f"机器人朝向: {pose_data['rotation_euler']['yaw']} 弧度")

robot_pose_node.shutdown()  # 释放node资源