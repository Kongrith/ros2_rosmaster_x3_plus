from launch import LaunchDescription
from launch_ros.actions import Node

# launch description
def generate_launch_description():
	return LaunchDescription([
		# Node(
		# 	package="micro_ros_agent",
		# 	executable="micro_ros_agent",
		# 	name="micro_ros_agent_teensy",
		# 	arguments=['serial', '--dev', '/dev/ttyACM0'],
		# 	output='screen'
		# ),
		Node(
			package="mypkg",
			executable="robot_core",
			name="robot_core",
			# arguments=['serial', '--dev', '/dev/ttyACM0'],
			output='screen',
		),
		Node(
			package="mypkg",
			executable="robot_core",
			name="robot_core",
			# arguments=['serial', '--dev', '/dev/ttyACM0'],
			output='screen',
		),
	])


