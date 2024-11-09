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
		(os.path.join('share', package_name), glob("launch/*.launch.py")),
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
			"first_publisher = mypkg.first_node:main",
			"twist_publisher = mypkg.twist_pub:main",
			"first_subscription = mypkg.first_sub:main",
			"twist_subscription = mypkg.twist_sub:main",
			"turtle_avoidance = mypkg.turtle_control:main",
        ],
    },
)
