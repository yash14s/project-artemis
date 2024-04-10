#!/bin/bash

#Shell script to launch Ardupilot-SITL and connect it to Gazebo

cd ~/lidar_drone/ardupilot/ArduCopter/
source ~/anaconda3/etc/profile.d/conda.sh
conda activate ardupilotenv
sim_vehicle.py -v ArduCopter -f gazebo-iris --console