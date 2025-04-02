import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchContext


def generate_launch_description():

    # 获取与拼接默认路径
    driveguard_navigation2_dir = get_package_share_directory(
        'driveguard_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true')
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(driveguard_navigation2_dir, 'config', 'nav2_params.yaml'))
    
    # 如果使用[]嵌套对象需要解析位为字符串
    # my_launch_path = launch.substitutions.LaunchConfiguration(
    #     'params_file', default=os.path.join(driveguard_navigation2_dir, 'launch', 'my.launch.py'))
    # context = LaunchContext()
    # my_launch_path_str = my_launch_path.perform(context)   

    return launch.LaunchDescription([
        # 声明新的 Launch 参数
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),

        launch.actions.IncludeLaunchDescription(
            # my_launch_path = /home/driveguard/Desktop/drive_guard/install/driveguard_navigation2/share/driveguard_navigation2/launch/my.launch.py
            # 上这个不行，需要用下边这个路径
            PythonLaunchDescriptionSource(
                os.path.join(driveguard_navigation2_dir, 'launch', 'my.launch.py')),
                # 如果使用[]嵌套对象需要解析位为字符串
                # [my_launch_path_str]),

            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
