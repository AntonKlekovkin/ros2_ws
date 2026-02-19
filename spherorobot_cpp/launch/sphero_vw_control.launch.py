from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    spherorobot_package_path = PathJoinSubstitution([FindPackageShare("spherorobot_cpp"), "launch"])

    linVelController = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([spherorobot_package_path,'/sphero_lin_vel_controller.launch.py'])
                                  )

    angVelController = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([spherorobot_package_path,'/sphero_ang_vel_controller.launch.py'])
                                  )

    vwController = Node(
            package='spherorobot_cpp',
            executable='sphero_vw_control',
            name='sphero_vw_control_node',            
            output='screen'
        )
    
    return LaunchDescription([linVelController, angVelController, vwController])