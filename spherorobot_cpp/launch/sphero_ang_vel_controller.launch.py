from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    vel = DeclareLaunchArgument(
            name='vel', 
            default_value='0.0',
            description='Target yaw rate for sphero. Max = 2.4 rad/s'
        )
    
    kpArg = DeclareLaunchArgument(
            name='kp', 
            default_value='0.007',
            description='Kp'
        )
    
    kdArg = DeclareLaunchArgument(
            name='kd', 
            default_value='0.00001',
            description='Kd'
        )
    
    kiArg = DeclareLaunchArgument(
            name='ki', 
            default_value='0.01',
            description='Kd'
        )

    myNode = Node(
            package='spherorobot_cpp',
            executable='sphero_ang_vel_control',
            name='sphero_ang_vel_control_node',
            parameters=[{'kp' : LaunchConfiguration('kp')},
                        {'kd' : LaunchConfiguration('kd')},
                        {'ki' : LaunchConfiguration('ki')},
                        {'vel' : LaunchConfiguration('vel')}],
            output='screen'
        )
    
    return LaunchDescription([vel, kpArg, kdArg, kiArg, myNode])