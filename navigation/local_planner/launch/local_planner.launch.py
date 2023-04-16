from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    convert = Node(
            package="local_planner",
            executable="convert",
            name="convert"
            )
    move = Node(
            package="local_planner",
            executable="move",
            name="move"
            )
    ld.add_action(convert)
    ld.add_action(move)

    return ld
                


