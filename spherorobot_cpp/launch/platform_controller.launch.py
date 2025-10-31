from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
       
    hzArg = DeclareLaunchArgument(
            name='hz', 
            default_value='1.0',
            description='Freq of platform motion'
        )
    
    ampArg = DeclareLaunchArgument(
            name='amp', 
            default_value='1.0',
            description='Amp of platform motion'
        )
    
    platformController = Node(
            package='spherorobot_cpp',
            executable='platform_control',
            name='platform_control_node',
            parameters=[{'amp' : LaunchConfiguration('amp')},
                        {'hz' : LaunchConfiguration('hz')}],
            output='screen'
        )
    
    return LaunchDescription([hzArg, ampArg, platformController])