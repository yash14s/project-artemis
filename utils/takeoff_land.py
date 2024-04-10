# -*- coding: utf-8 -*-
"""
Created on Sun Dec  5 14:05:21 2021

@author: Yash
"""

from dronekit import connect, VehicleMode
import time

def connect_to_python():
    global vehicle
    vehicle = connect('tcp:192.168.29.231:5763', wait_ready=True)
    print("Connected")
    
def disconnect_to_python():
    vehicle.close()
    print("Disconnected")

def arm():
    connect_to_python()
    while vehicle.is_armable!=True:
        print("Waiting for the vehicle to become armable")
        time.sleep(1)

    print('Vehicle is now armable')
    vehicle.mode=VehicleMode('GUIDED')
    while vehicle.mode!='GUIDED':
        print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)

    print("Vehicle now in GUIDED MODE.")

    vehicle.armed=True
    while vehicle.armed==False:
        print("Waiting for vehicle to become armed")
        time.sleep(1)

    print("Caution! Drone is ARMED!")

def disarm():    
    print("Proceeding to disarm")
    vehicle.armed = False
    while(vehicle.armed==True):
        time.sleep(1)
    print("Drone is now DISARMED")
    
def takeoff(targetHeight):
    arm()
    vehicle.simple_takeoff(targetHeight)#meters
    while True:
        print("Current Altitude: %f"%vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=.95*targetHeight:
            break
        time.sleep(1)
    print("Target altitude reached!!")

def land():
    vehicle.mode=VehicleMode("LAND")
    while vehicle.mode != 'LAND':
        print("Waiting for drone to enter LAND mode")
        time.sleep(1)
    print("Vehicle in LAND mode")
    disarm()
    
def takeoff_land(targetHeight):
    takeoff(targetHeight)
    land()
    disconnect_to_python()
    
takeoff_land(3)
