from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mypkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

		# เพิ่มตรงนี้ สำหรับการเพิ่ม launch file
		(os.path.join('share', package_name, "mypkg"), glob("../launch/*.launch.py")),
		(os.path.join('share', package_name, "launch"), glob("launch/*.launch.py")),
		(os.path.join('share', package_name, "config"), glob("../config/*.yaml")),
		(os.path.join('share', package_name, "config"), glob("../config/*.lua")),
		(os.path.join('share', package_name, "map"), glob("../map/*.yaml")),
		(os.path.join('share', package_name, "map"), glob("../map/*.pgm")),

		# Include all launch files.
		# (os.path.join('share', package_name, 'launch'), glob(os.path.join(os.getcwd(), '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cartoon',
    maintainer_email='cartoon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			# "first_publisher = mypkg.first_node:main",
			# "twist_publisher = mypkg.twist_pub:main",
			# "first_subscription = mypkg.first_sub:main",
			# "twist_subscription = mypkg.twist_sub:main",
			# "turtle_avoidance = mypkg.turtle_control:main",
			# "first_param = mypkg.first_param:main",
			# "turtle_service_server = mypkg.turtle_service_server:main",
			# "turtle_service_client = mypkg.turtle_service_client:main",
			# "robot_core = mypkg.robot_core:main",
			"robot_run = mypkg.robot_mecanum:main",
			"nav2_cmd = mypkg.navigation2_command:main",
			"imu_publisher = mypkg.imu_publisher:main",
			"img_publisher = mypkg.image_publisher:main",
			"img_subscriber = mypkg.image_subscriber:main",
            "run2m = mypkg.run2m:main",
            "turn90 = mypkg.turn90:main",
        ],
    },
)
