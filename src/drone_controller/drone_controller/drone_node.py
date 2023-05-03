#!/usr/bin/env python3
from __future__ import print_function
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

#========================================================
# by Lucas Paiva for the Robocin 2023 selective process
#========================================================

# defining the color spectre for each color
green_lower = np.array([50, 100, 100])
green_upper = np.array([70, 255, 255])
blue_lower = np.array([110, 100, 100])
blue_upper = np.array([130, 255, 255])
pink_lower = np.array([140, 100, 100])
pink_upper = np.array([170, 255, 255])
red_lower = np.array([170, 120, 70])
red_upper = np.array([180, 255, 255])
# Creating a message to work with the speed control (code took from the dronekit documentation with a few modifications)
def send_ned_velocity(vx, vy, vz):
    vel = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        vx, # X velocity in NED frame in m/s
        vy, # Y velocity in NED frame in m/s
        vz, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    vehicle.send_mavlink(vel)
    
    
# Drone connection 
connection_string = "127.0.0.1:14550"
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
#this variable will be used to verify if the drone will be landing after first cycle
landing = 0

class CameraSubscriberNode(Node):
    # creating a subscriber to the camera feed
    def __init__(self):
        super().__init__("CameraSubscriber") 
        self.Camera_Subscriber = self.create_subscription(Image, "/camera/image_raw", self.camera_callback, 10) 
        
    def camera_callback(self, msg: Image): 
        global landing
        #using cvbridge to convert the image to a format that can be used by opencv
        frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #filtering the image using the color spectre
        #create white masks for the colors
        green_mask = cv2.inRange(hsv, green_lower, green_upper)  
        blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
        pink_mask = cv2.inRange(hsv, pink_lower, pink_upper)
        red_mask = cv2.inRange(hsv, red_lower, red_upper)
        
        #printing the current values
        print("Green")
        print(green_mask[240][320])
        print("Blue")
        print(blue_mask[240][320])
        print("Pink")
        print(pink_mask[240][320])
        print("Red")
        print(red_mask[240][320])
        self.get_logger().info("Feed Received")
        
        # verify if the point right above the drone is green, blue, pink or red and move the drone accordingly
        if green_mask[240][320] == 255: 
            send_ned_velocity(-0.6, 0, 0)
            if landing == 1:
                vehicle.mode = VehicleMode("LAND")
        elif blue_mask[240][320] == 255:
            send_ned_velocity(0, -0.6, 0)
        elif pink_mask[240][320] == 255:
            landing = 1
            send_ned_velocity(0.6, 0, 0)
        elif red_mask[240][320] == 255:
            send_ned_velocity(0, 0.6, 0)

    
def main(args=None):
    rclpy.init(args=args)
    
    # Pre arm the drone
    print("Pre arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(2)
    # Arming the drone
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    # Arm verification
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(2)
    print("Take off")
    vehicle.simple_takeoff(1)
    time.sleep(5)
    
    #calling the node
    node = CameraSubscriberNode()
    #spining the node
    rclpy.spin(node) 
    rclpy.shutdown()


if __name__ == '__main__':
    main()