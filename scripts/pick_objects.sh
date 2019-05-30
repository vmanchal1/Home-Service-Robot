#!/bin/sh
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/map/my_home.world " &
sleep 3
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 3
xterm -e " source /home/workspace/catkin_ws/devel/setup.bash; rosrun pick_objects pick_objects" &
sleep 3
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch "