#!/bin/bash

#Ensure setup.bash is sourced
source ~/lidar_drone/catkin_ws/devel/setup.bash

#Start Navigation
source ~/anaconda3/etc/profile.d/conda.sh
conda activate ardupilotenv
rosrun project_artemis navigation.py