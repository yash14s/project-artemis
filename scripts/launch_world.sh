#!/bin/bash

#Ensure setup.bash is sourced
source ~/lidar_drone/catkin_ws/devel/setup.bash

#Launch World. This will launch the world, start RGB camera and obstacle avoidance
roslaunch project_artemis forest.launch