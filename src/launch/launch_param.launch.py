'''
'''

from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
	node1 = Node(package="mypkg",
			  executable="first_param",
			  parameters= [{'wheel_radius': 25, "character": 'diffdrive'}],
			  output='screen',
			  emulate_tty="True")
	# node2 = Node(package="mypkg", executable="first_subscription")

	# ld หมายถึง launch description
	ld = LaunchDescription()
	ld.add_action(node1)
	# ld.add_action(node2)

	return ld
