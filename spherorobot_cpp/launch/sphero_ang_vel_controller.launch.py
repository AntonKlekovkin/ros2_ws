from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    ang_vel = DeclareLaunchArgument(
            name='ang_vel', 
            default_value='0.0',
            description='Target yaw rate for sphero. Max = 2.4 rad/s'
        )
    
    ang_vel_kpArg = DeclareLaunchArgument(
            name='ang_vel_kp', 
            default_value='0.007',
            description='Kp'
        )
    
    ang_vel_kdArg = DeclareLaunchArgument(
            name='ang_vel_kd', 
            default_value='0.00001',
            description='Kd'
        )
    
    ang_vel_kiArg = DeclareLaunchArgument(
            name='ang_vel_ki', 
            default_value='0.01',
            description='Kd'
        )

    ang_vel_myNode = Node(
            package='spherorobot_cpp',
            executable='sphero_ang_vel_control',
            name='sphero_ang_vel_control_node',
            parameters=[{'ang_vel_kp' : LaunchConfiguration('ang_vel_kp')},
                        {'ang_vel_kd' : LaunchConfiguration('ang_vel_kd')},
                        {'ang_vel_ki' : LaunchConfiguration('ang_vel_ki')},
                        {'ang_vel' : LaunchConfiguration('ang_vel')}],
            output='screen'
        )
    
    return LaunchDescription([ang_vel, ang_vel_kpArg, ang_vel_kdArg, ang_vel_kiArg, ang_vel_myNode])