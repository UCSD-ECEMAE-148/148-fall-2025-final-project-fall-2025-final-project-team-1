#Executes both "radio_bridge" and "bridge_node"

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription ( [
		Node(
			package = "robojeep_arduino_bridge",
			executable = "bridge_node",
			name = "bridge"
		).
		Node(
			package = "robojeep_arduino_bridge",
			executable = "radio_bridge",
			name = "radio"
		)
	] )

