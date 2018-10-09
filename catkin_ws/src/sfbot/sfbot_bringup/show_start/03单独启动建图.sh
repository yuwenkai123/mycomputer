#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source /home/sfbot/ros_voice_system/devel/setup.bash
gnome-terminal -t "启动Gmapping建图" -x bash -c "roslaunch sfbot_nav gmapping_demo.launch ;exec bash;"
sleep 1
gnome-terminal -t "启动建图rviz" -x bash -c "roslaunch sfbot_rviz gmapping_view.launch ;exec bash;"
