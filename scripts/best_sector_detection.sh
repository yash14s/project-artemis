#!/bin/bash

#Ensure setup.bash is sourced
source ~/lidar_drone/catkin_ws/devel/setup.bash

#Start Best Sector Detection (Stereo Vision based collision avoidance)
source ~/anaconda3/etc/profile.d/conda.sh
conda activate paenv
rosrun project_artemis best_sector_detection.py