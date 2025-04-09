#!/bin/bash

# 构建功能包
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# 设置环境
source install/setup.bash

# 在后台启动gazebo仿真
ros2 launch driveguard_description gazebo_sim.launch.py &

# 等待一段时间确保gazebo完全启动
sleep 5

# 在后台启动cartographer
ros2 launch driveguard_cartographer cartographer.launch.py &

# 等待一段时间确保cartographer初始化
sleep 5

# 启动导航
ros2 launch driveguard_navigation2 navigation2.launch.py