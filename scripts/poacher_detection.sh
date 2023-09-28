#!/bin/bash

#Ensure setup.bash is sourced
source ~/lidar_drone/catkin_ws/devel/setup.bash

#Start Poacher Detection
source ~/anaconda3/etc/profile.d/conda.sh
conda activate paenv
rosrun project_artemis poacher_detection.py