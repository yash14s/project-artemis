1. View rgb camera stream: $ rosrun image_view image_view image:=/rgb_cam/image_raw

2. Stereo camera:
	i. Launch an instance of stereo_image_proc - 
		$ ROS_NAMESPACE=multisense_sl/camera rosrun stereo_image_proc stereo_image_proc
	ii. View stereo image-
		$ rosrun image_view stereo_view stereo:=/multisense_sl/camera image:=image_rect _approximate_sync:=True _queue_size:=10
	iii. For GUI - 
		$ rosrun rqt_reconfigure rqt_reconfigure


[1] http://wiki.ros.org/stereo_image_proc
