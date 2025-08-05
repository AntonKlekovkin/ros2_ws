from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test_cpp_pkg',
            executable='myPublisher',
            name='minPublisher',
            parameters=[{'startValue' : 10}],
            output='screen'
        ),
        Node(
            package='test_cpp_pkg',
            executable='mySubscriber',
            name='minSubscriber',
            output='screen'
        ),
    ])