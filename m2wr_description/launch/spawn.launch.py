from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
import os
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("m2wr_description"), "urdf", "robot.urdf.xacro"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='true',
            description='Use simulation time'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_description': Command(['xacro ', urdf_path]),
                    'publish_frequency': 100.0,  # Частота публикации (Гц)
                    'ignore_timestamp': True,  # Игнорировать временные метки
                    'frame_prefix': ''  # Префикс для фреймов
                }
            ]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
            'source_list': ['joint_states_source'],  # откуда брать данные
            'rate': 100  # частота публикации
        }]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'm2wr',
                '-topic', 'robot_description',
                '-x', '0.0', '-y', '0.0', '-z', '0.01'
            ],
            output='screen'
        )
    ])