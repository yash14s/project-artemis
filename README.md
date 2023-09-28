# Project-Artemis

Demo: https://youtu.be/A8umhY0R0qM

project_artemis is a ROS package.

Pre-reqs:
Ubuntu 18.04 LTS with Ardupilot SITL, Dronekit-Python, Ardupilot-Gazebo plugin setup (https://github.com/yash14s/Drone/tree/main/installation%20and%20setup)

Run create_env.sh

TODO (Maybe) -> Docker Container for the setup.

Instructions to run framework:

TODO -> Add instructions to run using shell scripts.

Usage(obsolete):
1. $ roslaunch project_artemis forest.launch
	This will launch the world, start RGB camera and obstacle avoidance

2. $ conda activate py3env
   $ cd
   $ ./startsitl.sh

3. $ conda activate py2env
   $ rosrun project_artemis best_sector_detection.py

4. $ conda activate py2env
   $ rosrun project_artemis poacher_detection.py

5. $ conda activate py2env
   $ rosrun project_artemis navigation.py


Camera:
1. View rgb camera stream: $ rosrun image_view image_view image:=/rgb_cam/image_raw

2. Stereo camera:
	i. Launch an instance of stereo_image_proc - 
		$ ROS_NAMESPACE=multisense_sl/camera rosrun stereo_image_proc stereo_image_proc
	ii. View stereo image-
		$ rosrun image_view stereo_view stereo:=/multisense_sl/camera image:=image_rect _approximate_sync:=True _queue_size:=10
	iii. For GUI - 
		$ rosrun rqt_reconfigure rqt_reconfigure




[1] http://wiki.ros.org/stereo_image_proc
	

