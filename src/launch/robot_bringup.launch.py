from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription		# inlcude launch file ของตัวอื่นๆ
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

# launch description
def generate_launch_description():

	# สำหรับ EKF
	ekf_config_path = PathJoinSubstitution([FindPackageShare('mypkg'),
										   'config', 'ekf.yaml'])

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

		Node(
			package="mypkg",
			executable="imu_publisher",
			name="imu_converter_node",
			output='screen',
		),

		Node(
			package="tf2_ros",
			executable="static_transform_publisher",
			name="imu_tf_publisher",
			# x y z roll, pitch, yaw, parent_frame, child_frame
			arguments=['0.1', '0.0', '0.05', '0.0', '0.0', '0.0', '1.0', 'base_link', 'imu_link'],
			output='screen',
		),

		Node(
			package="robot_localization",
			executable="ekf_node",
			name="ekf_filter_node",
			arguments=[ ekf_config_path ],
			output='screen',
			remappings = [("odometry/filtered", "odom")],
		),

		# เพิ่ม launch file ตัวอื่นจาก Lidar
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(lidar_launch_path)
		)
	])


