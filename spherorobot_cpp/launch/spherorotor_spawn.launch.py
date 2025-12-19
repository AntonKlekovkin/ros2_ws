import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import xacro
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_name='spherorobot_cpp'

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('spherorobot_cpp'))
    urdf_file = os.path.join(pkg_path,'urdf','spherorobot_rotor_v1.urdf')
    robot_description_config = xacro.process_file(urdf_file)
    
    # Create a robot_state_publisher parameters
    params_rsp = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}

    pitch = DeclareLaunchArgument(
            name='P', 
            default_value='0.0',
            description='Pitch'
        )

    simTimeArg = DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use sim time if true')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params_rsp]
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'source_list': ['joint_states_source'],  # откуда брать данные
            'rate': 50  # частота публикации
        }]
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
                    '-entity', 'spherorobot_rotor',
                    '-z','0.06',
                    '-P',LaunchConfiguration('P')],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    
    return LaunchDescription([
        pitch,
        simTimeArg,
        gazebo,
        node_robot_state_publisher,
        node_joint_state_publisher,
        spawn_entity,
    ])