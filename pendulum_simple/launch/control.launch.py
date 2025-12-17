from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    ang = DeclareLaunchArgument(
            name='ang', 
            default_value='0.0',
            description='Angle for stabilization'
        )
       
    kpArg = DeclareLaunchArgument(
            name='kp', 
            default_value='1.0',
            description='Kp'
        )
    
    kdArg = DeclareLaunchArgument(
            name='kd', 
            default_value='1.0',
            description='Kd'
        )
    
    kiArg = DeclareLaunchArgument(
            name='ki', 
            default_value='0.001',
            description='Kd'
        )
    
    controller = Node(
            package='pendulum_simple',
            executable='pendulum_control',
            name='pendulum_control_node',
            parameters=[{'kp' : LaunchConfiguration('kp')},
                        {'kd' : LaunchConfiguration('kd')},
                        {'ki' : LaunchConfiguration('ki')},
                        {'ang' : LaunchConfiguration('ang')}],
            output='screen'
        )
    
    return LaunchDescription([ang, kpArg, kdArg, kiArg, controller])