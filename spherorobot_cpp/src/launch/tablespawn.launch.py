import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    package_name='spherorobot_cpp'

    
    spawn_table = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'rails_system',
            '-file', os.path.join(get_package_share_directory('spherorobot_cpp'),'src','urdf','vibro_table.urdf'),
            '-x', '0.5',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen',
        parameters=[{
        'use_sim_time': True
        }]
    )
    
    return LaunchDescription([
        spawn_table
    ])