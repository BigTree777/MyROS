from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node_1 = Node(
            package='viewer_pointcloud',
            executable='publisher_pointcloud'
            )
    node_2 = Node(
            package='viewer_pointcloud',
            executable='subscriber_pointcloud'
            )
    node_3 = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2"
            )
    
    return LaunchDescription([
        node_1,
        node_2,
        node_3
    ])