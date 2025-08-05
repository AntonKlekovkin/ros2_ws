import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import xacro
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    package_name='spherorobot_cpp'

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('spherorobot_cpp'))
    urdf_file = os.path.join(pkg_path,'src', 'urdf','spherorobot_v5_2.urdf')
    robot_description_config = xacro.process_file(urdf_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                    '-entity', 'spherorobot_fixed',
                    '-z','0.2',
                    '-P','0.0'],
        output='screen',
        parameters=[{
        'use_sim_time': True  
        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        node_robot_state_publisher,
        spawn_entity,
    ])