from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription		# inlcude launch file ของตัวอื่นๆ
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

# launch description
def generate_launch_description():

	# เอาไว้ต่อ path
	lidar_launch_path = PathJoinSubstitution([FindPackageShare('ydlidar_ros2_driver'),
										   'launch', 'ydlidar_launch.py'])

	return LaunchDescription([
		# Node(
		# 	package="micro_ros_agent",
		# 	executable="micro_ros_agent",
		# 	name="micro_ros_agent_teensy",
		# 	arguments=['serial', '--dev', '/dev/ttyACM0'],
		# 	output='screen'
		# ),
		# Node(
		# 	package="mypkg",
		# 	executable="robot_core",
		# 	name="robot_core",
		# 	# arguments=['serial', '--dev', '/dev/ttyACM0'],
		# 	output='screen',
		# ),

		Node(
			package="mypkg",
			executable="robot_run",
			name="robot_run",
			# arguments=['serial', '--dev', '/dev/ttyACM0'],
			output='screen',
		),

		# เพิ่ม launch file ตัวอื่น
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(lidar_launch_path)
		)
	])


