#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source /home/sfbot/ros_voice_system/devel/setup.bash
gnome-terminal -t "启动微信控制" -x bash -c "roslaunch voice_bringup voice_bringup.launch ;exec bash;"


