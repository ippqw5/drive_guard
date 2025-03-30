import sys
import os
import time

from init_robot_pose import *

# 设置初始位姿
robot_initial_pose = RobotInitialPose()
robot_initial_pose.set_position(0.0, 0.0)
robot_initial_pose.set_orientation(0.0, 0.0, 0.0, 1.0)

# 发布初始位姿
robot_navigator = RobotNavigator()
robot_navigator.set_initial_pose(robot_initial_pose)

# 释放资源
robot_navigator.shutdown()