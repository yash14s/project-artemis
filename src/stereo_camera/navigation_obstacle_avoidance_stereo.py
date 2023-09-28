#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import LaserScan
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import math
import time

def connect_to_python():
	global vehicle
	vehicle = connect('tcp:192.168.29.189:5762', wait_ready=True)
	print("Connected")

def disconnect_to_python():
	vehicle.close()
	print("Disconnected")

def arm():
	connect_to_python()
	while vehicle.is_armable!=True:
		print("Undergoing pre-arm checks")
		time.sleep(1)
	print('Drone is now armable')
	vehicle.mode=VehicleMode('GUIDED')
	while vehicle.mode!='GUIDED':
		print("Waiting for drone to enter GUIDED flight mode")
		time.sleep(1)
	print("Drone is now in GUIDED MODE.")
	vehicle.armed=True
	while vehicle.armed==False:
		print("Waiting for drone to become armed")
		time.sleep(1)
	print("Caution! Drone is ARMED!")

def disarm():
	print("Proceeding to disarm")
	vehicle.armed = False
	while(vehicle.armed==True):
		time.sleep(1)
	print("Drone is now DISARMED")

def lidar_callback(lidar_msg):
	global altitude
	altitude = lidar_msg.ranges[0]
	if altitude == float('inf'):
		altitude = vehicle.location.global_relative_frame.alt

def takeoff(targetHeight):
	arm()
	vehicle.simple_takeoff(targetHeight)#meters
	while True:
		print("Current Altitude: %f"%altitude)
		if altitude>=.91*targetHeight:
			break
		time.sleep(1)
	print("Target altitude reached")

def land():
	vehicle.mode=VehicleMode("LAND")
	while vehicle.mode != 'LAND':
		print("Waiting for drone to enter LAND mode")
		time.sleep(1)
	print("Vehicle in LAND mode")
	disarm()
	disconnect_to_python()

def get_distance_meters(waypoint,currentLocation):
	dLat = waypoint.lat - currentLocation.lat
	dLon = waypoint.lon - currentLocation.lon
	dAlt = waypoint.alt - altitude
	distance = math.sqrt((dLon*dLon)+(dLat*dLat)+(dAlt*dAlt))
	print("Distance to wp : %f"%distance)
	return distance

def condition_yaw(degrees, relative, clk):
	if relative:
		is_relative=1
	else:
		is_relative=0
	if clk:
		is_clk=1
	else:
		is_clk=-1
	msg=vehicle.message_factory.command_long_encode(0,0,mavutil.mavlink.MAV_CMD_CONDITION_YAW,0,degrees,0,is_clk,is_relative,0,0,0)
	vehicle.send_mavlink(msg)
	vehicle.flush()
	print("Yawing")
	time.sleep(6)
	print("Yaw complete")

def yaw_initializer():
	print("Initialising yaw")
	lat=vehicle.location.global_relative_frame.lat
	lon=vehicle.location.global_relative_frame.lon
	alt=vehicle.location.global_relative_frame.alt
	aLocation=LocationGlobalRelative(lat,lon,alt)
	msg=vehicle.message_factory.set_position_target_global_int_encode(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,0b0000111111111000,aLocation.lat*1e7,aLocation.lon*1e7,aLocation.alt,0,0,0,0,0,0,0,0)
	vehicle.send_mavlink(msg)
	vehicle.flush()
	time.sleep(2)

def send_local_ned_velocity(vx,vy,vz):
	msg=vehicle.message_factory.set_position_target_local_ned_encode(0,0,0,mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,0b0000111111000111,0,0,0,vx,vy,vz,0,0,0,0,0)
	vehicle.send_mavlink(msg)
	vehicle.flush()

def send_global_ned_velocity(vx,vy,vz):
	msg=vehicle.message_factory.set_position_target_local_ned_encode(0,0,0,mavutil.mavlink.MAV_FRAME_LOCAL_NED,0b0000111111000111,0,0,0,vx,vy,vz,0,0,0,0,0)
	vehicle.send_mavlink(msg)
	vehicle.flush()

def stop():
	print("Stop")
	send_local_ned_velocity(0,0,0)

def nav_velocity(waypoint,speed):
	x_goal = waypoint.lat
	y_goal = waypoint.lon
	z_goal = waypoint.alt
	x_init = vehicle.location.global_relative_frame.lat
	y_init = vehicle.location.global_relative_frame.lon
	z_init = altitude
	theta = math.atan2(y_goal-y_init,x_goal-x_init)
	vx = speed * math.cos(theta)
	vy = speed * math.sin(theta)
	vz = 0.5 * (z_init-z_goal)
	if theta > 0:
		theta_deg = theta*180.0/math.pi
	else:
		theta_deg = 360 + theta*180.0/math.pi
	return vx,vy,vz,theta_deg

#Getting best sector
def best_sector_callback(best_sector_msg):
	global sector
	sector = best_sector_msg.data

def generate_avoidance_velocity():
	#frame dimensions W x H
	w,h = [640,480]
	speed = 1
	k_y = 0.5
	k_z = 0.5
	y_init, z_init = [w//2, h//2]

	if sector == 1:
		y_goal, z_goal = [w//6, h//6]
	elif sector == 2:
		y_goal, z_goal = [w//2, h//6]
	elif sector == 3:
		y_goal, z_goal = [5*w//6, h//6]
	elif sector == 4:
		y_goal, z_goal = [w//6, h//2]
	elif sector == 6:
		y_goal, z_goal = [5*w//6, h//2]
	elif sector == 7:
		y_goal, z_goal = [w//6, 5*h//6]
	elif sector == 8:
		y_goal, z_goal = [w//2, 5*h//6]
	elif sector == 9:
		y_goal, z_goal = [5*w//6, 5*h//6]

	theta = math.atan2(y_goal-y_init,z_goal-z_init)
	vy = k_y * speed * math.sin(theta)
	vz = k_z * speed * math.cos(theta)
	return vy,vz

rospy.init_node('stereo_based_avoidance_node', anonymous=False)
rospy.Subscriber('/best_sector_topic', Int8, best_sector_callback)
rospy.Subscriber('spur/laser/scan', LaserScan, lidar_callback)
rospy.on_shutdown(land)

def travel(waypoint,speed):
	print("Travelling to waypoint")
	distance_to_waypoint = get_distance_meters(waypoint,vehicle.location.global_relative_frame)
	vx,vy,vz,theta_deg = nav_velocity(waypoint,speed)
	condition_yaw(theta_deg,0,1)
	while True:
		vx,vy,vz,theta_deg = nav_velocity(waypoint,speed)
		current_distance = get_distance_meters(waypoint,vehicle.location.global_relative_frame)
		send_global_ned_velocity(vx,vy,vz)
		if (current_distance<0.01*distance_to_waypoint):
			break

		if sector != 5:
			avoid_vy, avoid_vz = generate_avoidance_velocity()
			send_local_ned_velocity(0, avoid_vy, avoid_vz)
			rospy.loginfo("avoid_vy = %f , avoid_vz = %f",avoid_vy, avoid_vz)

	print("******Reached******")
	stop()

wp1=LocationGlobalRelative(-35.3642059,149.1699815,5)
wp2=LocationGlobalRelative(-35.3636285,149.1650891,5)
wp3=LocationGlobalRelative(-35.3632183,149.1652367,5)
wp4=LocationGlobalRelative(-35.3611261,149.1647887,5)
wp5=LocationGlobalRelative(-35.3635235,149.1680288,5)

takeoff(5)
yaw_initializer()
#travel(wp1,1)
#travel(wp2,1)
#travel(wp3,1)
#travel(wp4,2)
travel(wp5,1)
land()
