#!/bin/bash
killall gnome-terminal-server 
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source /home/sfbot/ros_voice_system/devel/setup.bash
gnome-terminal -t "播报服务" -x bash -c "roslaunch sfbot_bringup soundplay.launch ;exec bash;"
echo navgation Mode|rosrun sound_play say.py
gnome-terminal -t "启动底盘和雷达" -x bash -c "roslaunch sfbot_bringup lidar_start.launch ;exec bash;"
sleep 3
gnome-terminal -t "启动amcl导航" -x bash -c "roslaunch sfbot_nav amcl_demo.launch ;exec bash;"
sleep 1
gnome-terminal -t "启动建图RVIZ" -x bash -c "roslaunch sfbot_rviz amcl_view.launch ;exec bash;"

