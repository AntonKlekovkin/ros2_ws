from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    lin_vel = DeclareLaunchArgument(
            name='lin_vel', 
            default_value='0.0',
            description='Target linear velocity for sphero. Max = 0.2 m/s'
        )
    
    lin_vel_kpArg = DeclareLaunchArgument(
            name='lin_vel_kp', 
            default_value='6.5',
            description='Kp'
        )
    
    lin_vel_kdArg = DeclareLaunchArgument(
            name='lin_vel_kd', 
            default_value='0.3',
            description='Kd'
        )
    
    lin_vel_kiArg = DeclareLaunchArgument(
            name='lin_vel_ki', 
            default_value='6.0',
            description='Kd'
        )

    lin_vel_myNode = Node(
            package='spherorobot_cpp',
            executable='sphero_lin_vel_control',
            name='sphero_lin_vel_control_node',
            parameters=[{'lin_vel_kp' : LaunchConfiguration('lin_vel_kp')},
                        {'lin_vel_kd' : LaunchConfiguration('lin_vel_kd')},
                        {'lin_vel_ki' : LaunchConfiguration('lin_vel_ki')},
                        {'lin_vel' : LaunchConfiguration('lin_vel')}],
            output='screen'
        )
    
    return LaunchDescription([lin_vel, lin_vel_kpArg, lin_vel_kdArg, lin_vel_kiArg, lin_vel_myNode])