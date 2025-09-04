from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gazebo_ros_path = PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch"])
    world_file_path = PathJoinSubstitution([FindPackageShare("m2wr_description"), "worlds", "maze.world"])

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='true',
            description='Use simulation time'
        ),
        # Запуск Gazebo с плагинами
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([gazebo_ros_path, '/gazebo.launch.py']),
                launch_arguments={'world': world_file_path,
                                  'use_sim_time': LaunchConfiguration('use_sim_time'),
                                  'verbose': 'true'}.items()
             ),
             
    ])