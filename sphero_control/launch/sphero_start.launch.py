from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    spherorobot_package_path = PathJoinSubstitution([FindPackageShare("spherorobot_cpp"), "launch"])

    gazeboWorldArg = DeclareLaunchArgument(
        name='world', 
        default_value='myEmpty.world',
        description='Gazebo world file'
    )

    spheroController = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([spherorobot_package_path,'/sphero_vw_controller.launch.py'])
                                  )

    spheroSpawn = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([spherorobot_package_path,'/spherorotor_spawn.launch.py']),
                        launch_arguments=
                        {
                            'world': LaunchConfiguration('world')
                        }.items()
                                  )
    
    return LaunchDescription([gazeboWorldArg, spheroSpawn, spheroController])

    # ros2 launch sphero_control sphero_start.launch.py world:=cilinder11.world