# Home-Service-Robot

This project attempts to build a home service robotic simulation, that picks an object from a designated start location and places it at a goal location. This project uses SLAM packages available ROS.

To clone and run this project, follow the below steps-

To create an active ROS workspace-
```sh
mkdir -p catkin_ws/src #catkin_ws is the active ROS workspace
cd catkin_ws
catkin_make
source devel/setup.bash
```
To clone this repository and install all the dependencies-
```sh
cd catkin_ws/src
git clone https://github.com/vmanchal1/Home-Service-Robot.git
rosdep -i install gmapping -y
rosdep -i install turtlebot_teleop -y
rosdep -i install turtlebot_rviz_launchers -y
rosdep -i install turtlebot_gazebo -y
cd ..
catkin_make
```
To run the project-
```
cd catkin_ws
source devel/setup.bash
./src/scripts/home_service.sh
```
