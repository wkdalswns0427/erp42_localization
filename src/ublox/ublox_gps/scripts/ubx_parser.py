#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys,os
import rospy
import rospkg
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Path, Odometry
import tf2_ros
from tf.transformations import quaternion_from_euler
from serial import Serial
from serial.serialutil import SerialException
from pyubx2 import UBXReader
import pyubx2.exceptions as ube
from ublox_msgs.msg import NavPOSLLH, NavVELNED, NavPVT, NavCustom
from math import pi 




class ubxParser:
    def __init__(self):
        self.getMsg = False
        rospy.init_node('ubx_parser', anonymous=True)
        self.stream = Serial('/dev/ttyGPS', 38400, timeout=0.1)
        self.ubr = UBXReader(self.stream)
        self.interested_header = ['NAV-VELNED','NAV-PVT','NAV-POSLLH']
        self.custom_nav_pub = rospy.Publisher('/ublox/navCustom', NavCustom, queue_size=1) 
        self.odom_pub = rospy.Publisher('/ublox/odom', Odometry, queue_size=1) 
        self.test_mode = False
        self.initNavMsg()
        self.status = False
        self.status_last = False
        self.count = 0
        self.update()

    def update(self):
        while not rospy.is_shutdown():
            self.ubxread()
            # if(self.test_mode):
            #     self.test()

    def initNavMsg(self):
        self.msg = NavCustom()
        self.msg.lat = 0.0
        self.msg.lon = 0.0
        self.msg.gSpeed = 0
        self.msg.heading = 0.0
        self.msg.carrSoln = 0


    def ubxread(self):
        
        try:
            (raw_data, parsed_data) = self.ubr.read()
            self.count = self.count + 1 
            while parsed_data.identity in self.interested_header:
                self.status_last = self.status
                if parsed_data.identity == 'NAV-VELNED':
                    self.gSpeed = parsed_data.gSpeed 
                    self.heading = parsed_data.heading * 1e-5
                    self.heading -= 90
                elif parsed_data.identity == 'NAV-POSLLH':
                    self.lat = parsed_data.lat * 1e-7
                    self.lon = parsed_data.lon * 1e-7
                    # store init x,y at the first time 
                    # if not self.getMsg :
                    #     self.init_x , self.init_y = self.ll2tum.LLtoUTM(self.lat,self.lon)
                    self.getMsg = True
                elif parsed_data.identity == 'NAV-PVT':
                    self.carrSoln = parsed_data.carrSoln
                if self.count >= 3:
                    self.count = 0
                    self.rosMsgPublish()
                    break

                (raw_data, parsed_data) = self.ubr.read()
                self.count = self.count + 1
                self.status = True

        except (
            ube.UBXStreamError,
            ube.UBXMessageError,
            ube.UBXTypeError,
            ube.UBXParseError,
            SerialException,
            AttributeError,

        ) as err:
            #print(f"Something went wrong {err}")
            self.count = 0
            pass 


    def rosMsgPublish(self):
        if self.getMsg:
            self.msg.lat = self.lat
            self.msg.lon = self.lon
            self.msg.gSpeed = self.gSpeed
            self.msg.heading = self.heading*180/pi
            self.msg.carrSoln = self.carrSoln
            self.custom_nav_pub.publish(self.msg)

    # def test(self):
    #     x, y = self.ll2tum.LLtoUTM(self.lat,self.lon)
    #     msg = Odometry()
    #     msg.header.stamp = rospy.Time.now()
    #     msg.header.frame_id = "odom"
    #     msg.child_frame_id = "base_footprint"
    #     msg.pose.pose.position.x = x-self.init_x
    #     msg.pose.pose.position.y = x-self.init_y
    #     msg.pose.pose.orientation = quaternion_from_euler(0,0,self.heading*pi/180)
    #     self.odom_pub(msg)

    #     t = geometry_msgs.TransformStamped()
    #     t.header.stamp = rospy.Time.now()
    #     t.header.frame_id = "odom"
    #     t.child_frame_id = "base_footprint"
    #     t.transfrom.translation.x = x-self.init_x
    #     t.transfrom.translation.y = y-self.init_y
    #     t.transfrom.translation.z = 0
    #     q = quaternion_from_euler(0,0,self.heading*pi/180)
    #     t.transfrom.rotation.x = q[0]
    #     t.transfrom.rotation.x = q[1]
    #     t.transfrom.rotation.x = q[2]
    #     t.transfrom.rotation.x = q[3]
    #     self.tfBroadcaster.sendTransform(t)

if __name__ == '__main__':
    ubxParser()
        
        



