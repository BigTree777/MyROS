from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
            package='hello_world',
            executable='hello_world'
            )
    return LaunchDescription([
        node
    ])