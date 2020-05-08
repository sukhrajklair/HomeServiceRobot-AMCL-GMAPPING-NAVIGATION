#!/bin/sh
export TURTLEBOT_GAZEBO_WORLD_FILE="$(dirname $PWD)/map/sukhraj.world"
export TURTLEBOT_GAZEBO_MAP_FILE="$(dirname $PWD)/map/map.yaml"
xterm -e "source /opt/ros/kinetic/setup.bash; source ../../devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "source ../../devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "source ../../devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10
xterm -e "source ../../devel/setup.bash; rosrun add_markers add_markers"
