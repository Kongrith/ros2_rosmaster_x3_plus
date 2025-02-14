#!/bin/sh
# title1="tab 1"
# title2="tab 2"
# title3="tab 3"

# cmd1="ros2 launch mypkg robot_bringup.launch.py"
# cmd2="ros2 launch mypkg nav2.launch.py"
# cmd3="ros2 launch robot_nav_tool nav_gui_bridge.launch.py"

# xterm -e "ros2 launch mypkg robot_bringup.launch.py"
# xterm -e "ros2 launch mypkg nav2.launch.py"
# xterm -e "ros2 launch robot_nav_tool nav_gui_bridge.launch.py"
# ros2 launch mypkg robot_bringup.launch.py
# ros2 launch mypkg nav2.launch.py
# ros2 launch robot_nav_tool nav_gui_bridge.launch.py

# gnome-terminal --tab --title="$title1" --command="bash -c '$cmd1; $SHELL'" \
#                --tab --title="$title2" --command="bash -c '$cmd2; $SHELL'" \
#                --tab --title="$title3" --command="bash -c '$cmd3; $SHELL'"


ros2 launch mypkg robot_bringup.launch.py && ros2 launch mypkg nav2.launch.py && ros2 launch robot_nav_tool nav_gui_bridge.launch.py
