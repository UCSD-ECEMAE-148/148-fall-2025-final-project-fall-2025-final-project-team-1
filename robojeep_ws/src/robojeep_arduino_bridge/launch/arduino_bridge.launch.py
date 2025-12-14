from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robojeep_arduino_bridge",
            executable="bridge_node",
            output="screen",
            parameters=[
                {"front_port": "/dev/ttyACM0"},
                {"rear_port": "/dev/ttyACM1"}
            ]
        )
    ])

