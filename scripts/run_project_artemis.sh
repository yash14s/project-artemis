#!/bin/bash

#This shell script runs the entire framework. Developed for a Docker container running Ubuntu 18.04 LTS.

#Ensure setup.bash is sourced
source ~/lidar_drone/catkin_ws/devel/setup.bash

#Launch World. This will launch the world, start RGB camera and obstacle avoidance
roslaunch project_artemis forest.launch

#Launch SITL
./startsitl.sh

#Continue execution only after GPS lock
read -p "Press Enter once you see the MAV Console displays the message - AP: EKF3 IMU0 is using GPS
AP: EKF3 IMU1 is using GPS"

#Start Best Sector Detection (Stereo Vision based collision avoidance) and Poacher Detection
conda activate paenv
rosrun project_artemis best_sector_detection.py
rosrun project_artemis poacher_detection.py

#Start Navigation
rosrun project_artemis navigation.py

