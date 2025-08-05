from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gazebo_ros_path = PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch"])
    
    return LaunchDescription([
        # Запуск Gazebo с плагинами
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([gazebo_ros_path, '/gazebo.launch.py']),
             )        
    ])