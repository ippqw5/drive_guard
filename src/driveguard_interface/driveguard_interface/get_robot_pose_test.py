import sys
import os
import time

from get_robot_pose import *

# 获取机器人位姿
robot_pose_node = RobotPose()

# 获取机器人位姿 10次
for i in range(10):
    transform_data = robot_pose_node.get_transform()
    print(f"机器人位置: x={transform_data['translation']['x']}, y={transform_data['translation']['y']}")
    print(f"机器人朝向: {transform_data['rotation_euler']['yaw']} 弧度")
    time.sleep(0.5)

# 释放资源
robot_pose_node.shutdown()