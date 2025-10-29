#!/bin/bash
# In a script like `wait_and_launch_navigation.sh`

sleep 10
# roslaunch jupiterobot2_navigation jupiterobot2_navigation.launch map_file:=/home/mustar/catkin_ws/maps/test1.yaml
roslaunch jupiterobot2_bringup jupiterobot2_bringup.launch 