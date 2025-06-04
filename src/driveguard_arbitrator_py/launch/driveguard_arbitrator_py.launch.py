from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='driveguard_arbitrator_py',
            executable='cmdvel_arbitrator_node.py',  # 修改为脚本名
            name='driveguard_arbitrator',
            output='screen',
            remappings=[
                ('/cmd_vel', '/cmd_vel'),  # 输出话题
                ('/goal_pose', '/goal_pose'),  # 目标位置话题
                ('/cmd_vel_model', '/cmd_vel_model'),  # 模型输入话题
                ('/cmd_vel_safe', '/cmd_vel_safe')  # 安全输入话题
            ]
        )
    ])