#Testing for driving jeep forward

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DriveForward(Node):
	def__init__(self):
		super().__init__("drive_forward")
