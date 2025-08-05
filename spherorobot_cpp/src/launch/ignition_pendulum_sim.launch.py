import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
import xacro

def generate_launch_description():
    package_share = get_package_share_directory('spherorobot_cpp')
    urdf_path = os.path.join(package_share,'src','urdf','spherorobot_v6.urdf')
    world_path = os.path.join(package_share,'src','worlds','spherorobot.world')
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': xacro.process_file(urdf_path).toxml(),
            'use_sim_time': True
        }]
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'verbose': 'true',
            'extra_gazebo_args': '--ros-args --params-file ',
            # 'physics': 'bullet'
        }.items()
    )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'spherorobot_cpp',
            '-z', '0.1',
            '-timeout', '60'
        ],
        output='screen'
    )
   

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    return ld