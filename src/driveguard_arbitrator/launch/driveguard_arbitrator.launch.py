from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='driveguard_arbitrator',
            executable='driveguard_arbitrator_node',
            name='driveguard_arbitrator',
            output='screen'
        )
    ])