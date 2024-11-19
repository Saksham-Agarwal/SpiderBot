from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = "spiderbotv1",
            namespace="spiderbot",
            executable = 'cpg_node',
            name = 'cpg'
        ),
        Node(
            package="spiderbotv1",
            namespace='spiderbot',
            executable= 'communication_node',
            name = 'communication'
        )
    ])