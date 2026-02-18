from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    yaw = DeclareLaunchArgument(
            name='yaw', 
            default_value='0.0',
            description='Target yaw for robot'
        )
    
    kpArg = DeclareLaunchArgument(
            name='kp', 
            default_value='0.5',
            description='Kp'
        )
    
    kdArg = DeclareLaunchArgument(
            name='kd', 
            default_value='0.01',
            description='Kd'
        )
    
    kiArg = DeclareLaunchArgument(
            name='ki', 
            default_value='0.001',
            description='Kd'
        )

    myNode = Node(
            package='spherorobot_cpp',
            executable='sphero_yaw_control',
            name='sphero_yaw_control_node',
            parameters=[{'kp' : LaunchConfiguration('kp')},
                        {'kd' : LaunchConfiguration('kd')},
                        {'ki' : LaunchConfiguration('ki')},
                        {'yaw' : LaunchConfiguration('yaw')}],
            output='screen'
        )
    
    return LaunchDescription([yaw, kpArg, kdArg, kiArg, myNode])