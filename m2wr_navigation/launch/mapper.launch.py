import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Пути к конфигурационным файлам (создайте их для вашего робота!)
    slam_params_path = PathJoinSubstitution([FindPackageShare("m2wr_navigation"), "config", "slam_params.yaml"])
    
    arg_sim_time = DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='true',
            description='Use simulation time'
        )
    
    # Запуск SLAM toolbox
    slam_toolbox = Node(
        parameters=[
            slam_params_path,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    return LaunchDescription([
        arg_sim_time,
        slam_toolbox
    ])