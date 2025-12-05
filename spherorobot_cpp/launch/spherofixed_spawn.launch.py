import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import xacro
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pitch = DeclareLaunchArgument(
            name='P', 
            default_value='0.0',
            description='Pitch'
        )
    
    simTimeArg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true')
   
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('spherorobot_cpp'))
    urdf_file = os.path.join(pkg_path,'urdf','spherorobot_v5_2.urdf')
    robot_description_config = xacro.process_file(urdf_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': LaunchConfiguration('use_sim_time')}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    gazebo_ros_path = PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch"])
    world_file_path = PathJoinSubstitution([FindPackageShare("spherorobot_cpp"), "worlds", "myEmpty.world"])

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([gazebo_ros_path, '/gazebo.launch.py']),
                launch_arguments={'world': world_file_path,
                                  'use_sim_time': LaunchConfiguration('use_sim_time'),                                  
                                  'extra_gazebo_args': '--verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so',
                                  'pause': 'false'}.items()
             )
    
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                    '-entity', 'spherorobot_fixed',
                    '-z','0.2',
                    '-P',LaunchConfiguration('P')],
        output='screen',
        parameters=[{
        'use_sim_time': True  
        }]
    )
    
    return LaunchDescription([
        pitch,
        simTimeArg,
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])