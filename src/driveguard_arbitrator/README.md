# DriveGuard Arbitrator Node

`driveguard_arbitrator_node` 是一个基于 ROS 2 的指令仲裁器（CmdVel Arbitrator），用于在自动驾驶小车系统中，在多个控制源（模型控制、安全控制、人工控制）之间实现安全、鲁棒的指令切换与状态管理。

## ✨ 特性

- ✅ 基于 Boost.SML 实现的状态机仲裁逻辑
- ✅ 支持模型、安全、人工控制的优先级动态切换
- ✅ 结合 Nav2 Costmap 实现实时局部风险检测
- ✅ 自动触发返回目标点与救援机制
- ✅ 提供节点在线检测与自动恢复机制

## 📥 输入话题

| Topic 名称 | 类型 | 说明 |
|------------|------|------|
| `/cmd_vel_model` | `geometry_msgs/msg/Twist` | 模型控制指令 |
| `/cmd_vel_safe` | `geometry_msgs/msg/Twist` | 安全控制器指令 |
| `/cmd_vel_manual` | `geometry_msgs/msg/Twist` | 手动控制器指令 |
| `/local_costmap/costmap_raw` | `nav2_msgs/msg/Costmap` | 局部代价地图 |
| `/navigate_to_pose/_action/status` | `action_msgs/msg/GoalStatusArray` | 检测导航任务状态 |
| `/safety_risk` *(可选)* | `std_msgs/msg/Bool` | 外部安全风险信号 |
| 心跳话题 *(可选)* | `std_msgs/msg/Bool` | 各模块是否在线检测 |

## 📤 输出话题

| Topic 名称 | 类型 | 说明 |
|------------|------|------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 最终仲裁输出速度指令 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 风险检测后返回原始目标点 |

## ⚙️ 配置参数（位于 `driveguard_arbitrator_config.h`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `CONTROL_LOOP_PERIOD` | 100 ms | 控制循环周期 |
| `HEARTBEAT_TIMEOUT` | 3000 ms | 心跳超时阈值 |
| `MODEL_CMD_TIMEOUT` | 120 ms | 模型指令超时判定 |
| `MANUAL_CMD_TIMEOUT` | 1000 ms | 人工指令超时判定 |
| `RESCUE_DURATION` | 10000 ms | 救援倒退持续时间 |
| `RESCUE_SPEED` | 0.05 m/s | 救援倒退速度 |
| `SAFETY_THRESHOLD` | 100 | 局部代价地图风险阈值 |

## 🧠 状态说明

| 状态码 | 状态名 | 说明 |
|--------|--------|------|
| 0 | IDLE | 空闲，无控制输入 |
| 1 | MODEL_OK | 正在执行模型控制 |
| 2 | MODEL_TIMEOUT | 模型指令超时 |
| 3 | SAFETY_OVERRIDE | 进入安全控制模式 |
| 4 | SAFETY_FAILURE | 安全控制失败，尝试救援 |
| 5 | MANUAL_OVERRIDE | 人工控制优先生效 |
| 6 | STOP | 安全控制源不可用，系统停止 |

## 🔧 启动方式

### 🔨 构建功能包

```bash
colcon build
```

### 🚀 启动仿真与建图
```
source install/setup.bash
ros2 launch driveguard_gazebo gazebo_sim_diff_drive.launch.py 

source install/setup.bash
ros2 launch driveguard_cartographer cartographer.launch.py
```

### 🧭 启动导航系统与仲裁器
#### 一次性启动并启用仲裁器
```
source install/setup.bash
ros2 launch driveguard_navigation2 nav2_diff_drive.launch.py use_arbitrator:=true
```
#### 或者：分别启动导航与仲裁器
```
source install/setup.bash
ros2 launch driveguard_navigation2 nav2_diff_drive.launch.py

source install/setup.bash
ros2 launch driveguard_arbitrator driveguard_arbitrator.launch.py
```
### 🕹️ 测试控制指令
#### 使用键盘控制工具模拟模型与人工控制指令
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_model

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_manual
```

#### 使用话题直接发布模型控制指令
```
ros2 topic pub /cmd_vel_model geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --rate 10
```