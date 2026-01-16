import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    # 获取与拼接默认路径
    mine_nav2_dir = get_package_share_directory('mine_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 创建 launch 配置变量（用于引用参数值）
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_path = LaunchConfiguration('map')
    nav2_param_path = LaunchConfiguration('params_file')
    
    # 定义默认路径
    default_map_path = os.path.join(mine_nav2_dir, 'map', 'room.yaml')
    default_params_path = os.path.join(mine_nav2_dir, 'config', 'nav2_params.yaml')
    
    return LaunchDescription([
        # 声明新的 launch 参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'map',
            default_value=default_map_path,
            description='Full path to map file to load'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_path,
            description='Full path to param file to load'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            # 使用 launch 参数替换原有参数
            launch_arguments={
                'map': map_yaml_path,
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