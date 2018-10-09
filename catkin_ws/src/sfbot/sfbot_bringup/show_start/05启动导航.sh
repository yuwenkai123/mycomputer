#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source /home/sfbot/ros_voice_system/devel/setup.bash
gnome-terminal -t "启动amcl导航" -x bash -c "roslaunch sfbot_nav amcl_demo.launch ;exec bash;"
sleep 1
gnome-terminal -t "启动导航rviz" -x bash -c "roslaunch sfbot_rviz amcl_view.launch ;exec bash;"
