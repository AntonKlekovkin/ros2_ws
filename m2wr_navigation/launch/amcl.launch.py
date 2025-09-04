import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Пути к конфигурационным файлам (создайте их для вашего робота!)
    # slam_params_path = PathJoinSubstitution([FindPackageShare("m2wr_navigation"), "config", "slam_params.yaml"])
    map_path = PathJoinSubstitution([FindPackageShare("m2wr_navigation"), "maps", "my_map2.yaml"])
    nav2_params_path = PathJoinSubstitution([FindPackageShare("m2wr_navigation"), "config", "nav2_params.yaml"])

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    arg_sim_time = DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='true',
            description='Use simulation time'
        )
    
    # Сервер карты
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time', default='true')},
            {'yaml_filename': map_path}
        ],        
    )

    # AMCL локализация
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            nav2_params_path,
            {'use_sim_time': True}
        ],
        emulate_tty=True,
        remappings=remappings,
    )
    
    # Запуск Nav2
    # nav_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             get_package_share_directory('nav2_bringup'),
    #             'launch',
    #             'navigation_launch.py'
    #         )
    #     ]),
    #     launch_arguments={
    #         'use_sim_time': 'true',
    #         'params_file': nav2_params_path
    #     }.items()
    # )

    # Lifecycle менеджер для управления нодами Nav2
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ],
        emulate_tty=True
    )

    return LaunchDescription([
        arg_sim_time,
        lifecycle_manager,
        map_server,          
        amcl
    ])