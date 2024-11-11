'''
'''

import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

	param_file = os.path.join(
		get_package_share_directory('mypkg'),
		'config',
		'param_test.yaml'
	)

	node1 = Node(package="mypkg",
			  executable="first_param",
			  output='screen',
			  emulate_tty=True,
			  parameters=[param_file],
			  )
	# node2 = Node(package="mypkg", executable="first_subscription")

	# ld หมายถึง launch description
	ld = LaunchDescription()
	ld.add_action(node1)
	# ld.add_action(node2)

	return ld
