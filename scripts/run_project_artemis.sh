#!/bin/bash

#TODO->This shell script runs the entire framework. Developed for a Docker container running Ubuntu 18.04 LTS.

#Launch
xterm -e "./launch_world.sh" &

#SITL
xterm -e "./start_sitl.sh" &

#Continue execution only after GPS lock
xterm -e "read -p 'Press Enter once you see the MAV Console displays the message - AP: EKF3 IMU0 is using GPS AP: EKF3 IMU1 is using GPS'"

#Best Sector Detection (Stereo Vision-based collision avoidance) and Poacher Detection
xterm -e "./best_sector_detection.sh" &

#Poacher Detection
xterm -e "./poacher_detection.sh" &

#Navigation
xterm -e "./navigation.sh"