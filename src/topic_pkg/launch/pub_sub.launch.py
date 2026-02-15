from launch_ros.actions import Node
from launch import LaunchDescription

PACKAGE_NAME: str = "topic_pkg"

def generate_launch_description():
    publisher_node = Node(
        package=PACKAGE_NAME,
        executable='publisher_class',
        name='publisher_class'
    )

    subscriber_node = Node(
        package=PACKAGE_NAME,
        executable='subscriber_class',
        name='subscriber_class'
    )

    ld = LaunchDescription()
    ld.add_action(publisher_node)
    ld.add_action(subscriber_node)

    return ld