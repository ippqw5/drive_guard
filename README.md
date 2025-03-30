# DriveGuard

## 1.项目介绍

各功能包功能如下：
- driveguard_description 机器人模型的描述文件，包含仿真相关配置
- driveguard_navigation2 导航配置
- driveguard_application 应用功能节点
    - driveguard_interface  提供易用的对外接口，屏蔽ros细节
        - 实时获得当前位置 get_robot_pose
        - 设置初始位置 init_robot_pose
    - test

## 2.使用方法   

- 系统版本： Ubunt22.04
- ROS 版本：ROS 2 Humble

### 2.1安装

建图采用 slam-toolbox，导航采用 Navigation 2 ,仿真采用 Gazebo，运动控制采用 ros2-control 实现，构建之前请先安装依赖，指令如下：

1. 安装 SLAM 和 Navigation 2

```
sudo apt install ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-slam-toolbox
```
> 图已经建过了，保存在 src/driveguard_navigation2/maps

2. 安装仿真相关功能包

```
sudo apt install ros-$ROS_DISTRO-robot-state-publisher  ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-ros2-control gazebo ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-gazebo-ros2-control ros-$ROS_DISTRO-xacro 
```

3. 安装transforms3d库，欧拉角<-->四元数

```
sudo apt install ros-$ROS_DISTRO-tf-transformations
sudo pip3 install transforms3d
```

### 2.2运行

安装完成依赖后，可以使用 colcon 工具进行构建和运行。

构建功能包

```
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

运行仿真

```
source install/setup.bash
ros2 launch driveguard_description gazebo_sim.launch.py
```

运行cartographer(目前问题：运行导航launch在已有的fishbot项目文件夹下可行,但迁移到driveguard下编译会有问题)

```
source install/setup.bash
ros2 launch driveguard_cartographer gcartographer.launch.py
```

运行导航

```
source install/setup.bash
ros2 launch driveguard_navigation2 navigation2.launch.py
```

## 3.参考

- https://github.com/fishros/ros2bookcode
- https://www.bilibili.com/video/BV1gr4y1Q7j5/ 第6-7章