#!/bin/bash

 # 标记是否存在 -c 参数
build=false

# 标记是否存在 -a 参数
use_ackermann=false

# 遍历所有参数
for arg in "$@"; do
    if [ "$arg" = "-c" ]; then
        build=true
    elif [ "$arg" = "-a" ]; then
        use_ackermann=true
    fi
done

# 如果存在 -c 参数，则执行构建
if $build; then
    # 构建功能包
    colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
fi

# 设置环境
source install/setup.bash

# 在后台启动gazebo仿真，根据 -a 参数决定启动哪个仿真
if $use_ackermann; then
    # 启动 ackermann 仿真
    ros2 launch driveguard_description gazebo_sim_ackermann.launch.py &
else
    # 启动默认仿真
    ros2 launch driveguard_description gazebo_sim.launch.py &
fi

# 等待一段时间确保gazebo完全启动
sleep 5

# 在后台启动cartographer
ros2 launch driveguard_cartographer cartographer.launch.py &

# 等待一段时间确保cartographer初始化
sleep 5

# 启动导航
ros2 launch driveguard_navigation2 navigation2.launch.py
