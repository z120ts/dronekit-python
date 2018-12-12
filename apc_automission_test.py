#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil


#connection_string = "127.0.0.1:14550"
connection_string = "tcp:127.0.0.1:5763"
sitl = None


#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def get_location_metres(original_location, dNorth, dEast):
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)


def adds_survey_mission(aLocation, length, interval):

    cmds = vehicle.commands

    print(" Clear ommands")
    cmds.clear() 
    
    print(" add commands.")
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 30))

    #Define the MAV_CMD_NAV_WAYPOINT locations and add the commands
    point1 = get_location_metres(aLocation, length, 0)
    point2 = get_location_metres(aLocation, length, interval)
    point3 = get_location_metres(aLocation, 0, interval)
    point4 = get_location_metres(aLocation, 0, interval*2)
    point5 = get_location_metres(aLocation, length, interval*2)
    point6 = get_location_metres(aLocation, length, interval*3)
    point7 = get_location_metres(aLocation, 0, interval*3)

    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_SET_CAM_TRIGG_DIST, 0, 0, 10, 0, 0, 0, point1.lat, point1.lon, 30))

    for i in range(1,8):
        point = eval('point' + str(i))
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point.lat, point.lon, 30))

    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_SET_CAM_TRIGG_DIST, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 30))

    #add dummy waypoint "10" at point 7
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point7.lat, point7.lon, 30))    

    print(" Upload new commands")
    cmds.upload()


def arm_and_takeoff(aTargetAltitude):

    print("pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

        
print('Create a survey mission around here')
adds_survey_mission(vehicle.location.global_frame,60,20)

arm_and_takeoff(30)

print("Starting mission")
vehicle.commands.next=0

vehicle.mode = VehicleMode("AUTO")


# Monitor mission. 
while True:
    nextwaypoint=vehicle.commands.next
    print('to waypoint (%s)...' % (nextwaypoint))

    if nextwaypoint==10: #Dummy waypoint
        print("Exit mission")
        break;
    time.sleep(1)

print('Return to launch')
vehicle.mode = VehicleMode("RTL")

print("Close object")
vehicle.close()

if sitl is not None:
    sitl.stop()
