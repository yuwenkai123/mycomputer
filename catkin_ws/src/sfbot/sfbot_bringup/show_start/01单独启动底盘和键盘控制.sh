#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source /home/sfbot/ros_voice_system/devel/setup.bash
gnome-terminal -t "启动底盘" -x bash -c "roslaunch sfbot_bringup mini_start.launch ;exec bash;"

