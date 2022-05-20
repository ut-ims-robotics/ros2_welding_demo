from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    pointcloud_publisher_node = Node(
        package="pointcloud",
        executable="publish",
    )
    pointcloud_selection_node = Node(
        package="pointcloud",
        executable="pointcloud_selection"
    )
    ld.add_action(pointcloud_publisher_node)
    ld.add_action(pointcloud_selection_node)
    return ld