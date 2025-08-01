from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        #SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        Node(
            package='cartographer_ros', executable='cartographer_node', output='screen',
            arguments=[
            	'-configuration_directory', get_package_share_directory('mypkg')+'/config',
            	# '-configuration_basename', 'cartographer.lua'
                '-configuration_basename', 'lds_2d.lua'
            ],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', '0.02', '-publish_period_sec', '1.0'],
        ),
    ])
