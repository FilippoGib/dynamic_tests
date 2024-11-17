from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dynamic_tests',
            executable='dynamic_tests_node',
            name='dynamic_tests_node',
            parameters=['/home/filippo/Formula/dynamic_tests/dynamic_tests/config/params.yaml'],
            output='screen',
        ),
    ])