#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import math


# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

#connection_string = args.connect
connection_string = "tcp:127.0.0.1:5763"
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def arm_and_takeoff(aTargetAltitude):

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    #print("\n Vehicle is armed: %s)" % vehicle.armed) 
    vehicle.armed = True

	#print("\n Vehicle mode: %s)" % vehicle.mode.name) 
    vehicle.mode = VehicleMode("GUIDED")

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed and not vehicle.mode.name == 'GUIDED':
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

vehicle.mode = VehicleMode("STABILIZE")

arm_and_takeoff(50)

print("Cicle Mode")
vehicle.mode = VehicleMode("CIRCLE")

# sleep so we can see the change in map
time.sleep(10)

print(" Armed: %s" % vehicle.armed) 
print(" Mode: %s" % vehicle.mode.name) 

print(" Autopilot Firmware version: %s" % vehicle.version)
print(" GPS: %s" % vehicle.gps_0)

print(" Battery: %s" % vehicle.battery)
print(" Attitude: %s" % vehicle.attitude)
print(" Gimbal status: %s" % vehicle.gimbal)
print(" Rangefinder: %s" % vehicle.rangefinder)

vehicle.mode = VehicleMode("RTL")

while not vehicle.mode.name == 'RTL':
    print("Returning to Launch...")
    time.sleep(1)

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()
