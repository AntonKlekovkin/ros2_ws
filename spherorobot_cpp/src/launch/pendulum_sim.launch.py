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

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'src', 'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={
                        'extra_gazebo_args': '--verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so',
        'pause': 'false'}.items()
             )

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                    '-entity', 'spherorobot_cpp',
                    '-z','0.2',
                    '-P','0.0'],
        output='screen',
        parameters=[{
        'use_sim_time': True  
        }]
    )
    
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
        rsp,
        gazebo,
        spawn_table,
        spawn_entity,
    ])