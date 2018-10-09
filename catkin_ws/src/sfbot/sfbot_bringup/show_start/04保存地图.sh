#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source /home/sfbot/ros_voice_system/devel/setup.bash
gnome-terminal -t "保存建图" -x bash -c "roslaunch sfbot_nav map_save.launch ;exec bash;"


