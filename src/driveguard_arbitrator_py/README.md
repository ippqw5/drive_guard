### 构建功能包

```
colcon build
```

### 运行
```
source install/setup.bash
ros2 launch driveguard_description gazebo_sim_diff_drive.launch.py 

source install/setup.bash
ros2 launch driveguard_cartographer cartographer.launch.py
```

### 运行仲裁节点（传递参数use_arbitrator）
```
source install/setup.bash
ros2 launch driveguard_navigation2 nav2_diff_drive.launch.py use_arbitrator:=true
```

### 测试
输入
```bash
ros2 topic pub /cmd_vel_model geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --rate 10
```
中断
```bash
ctrl+c
```
后小车将自动回到起点