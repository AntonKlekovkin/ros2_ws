import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
import xacro
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name='pendulum_simple'
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("pendulum_simple"), "urdf", "pendulum.urdf.xacro"]
    )

    pitchArgument = DeclareLaunchArgument(
            name='P', 
            default_value='0.0',
            description='Pitch'
        )
    simTimeArg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true')
    
    gazebo_ros_path = PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch"])
    world_file_path = PathJoinSubstitution([FindPackageShare("pendulum_simple"), "worlds", "myEmpty.world"])

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([gazebo_ros_path, '/gazebo.launch.py']),
                launch_arguments={'world': world_file_path,
                                  'use_sim_time': LaunchConfiguration('use_sim_time'),                                  
                                  'extra_gazebo_args': '--verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so',
                                  'pause': 'false'}.items()
             )
    
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory("pendulum_simple"))
    urdf_file = os.path.join(pkg_path,'urdf','pendulum.urdf.xacro')
    robot_description_config = xacro.process_file(urdf_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': True}
    # node_robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[params]
    # )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'source_list': ['joint_states_source'],  # откуда брать данные
            'rate': 50  # частота публикации
        }]
)
    
    node_robot_state_publisher= Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'robot_description': Command(['xacro ', urdf_path]),
                    'publish_frequency': 50.0,  # Частота публикации (Гц)
                    'ignore_timestamp': False,  # Игнорировать временные метки
                    'frame_prefix': ''  # Префикс для фреймов
                }
            ]
        )

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                    '-entity', 'pendulum_simple',
                    '-z','0.0',
                    '-P',LaunchConfiguration('P')],
        output='screen',
        parameters=[{
        'use_sim_time': True  
        }]
    )
    
    
    return LaunchDescription([
        simTimeArg,
        pitchArgument,
        gazebo,
        node_joint_state_publisher,
        node_robot_state_publisher,
        spawn_entity,
    ])