import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    package_name='spherorobot_cpp'

    pitch = DeclareLaunchArgument(
            name='P', 
            default_value='0.0',
            description='Pitch'
        )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
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
                    '-z','0.11',
                    '-P',LaunchConfiguration('P')],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    spawn_table = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'moving_platform',
            '-file', os.path.join(get_package_share_directory('spherorobot_cpp'),'urdf','vibro_table.urdf'),
            '-x', '0.5',
            '-y', '0.0',
            '-z', '0.001'
        ],
        output='screen',
        parameters=[{
        'use_sim_time': True
        }]
    )
    
    return LaunchDescription([
        gazebo,
        pitch,
        rsp,
        spawn_entity,
        spawn_table,
    ])