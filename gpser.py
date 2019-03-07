#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This is all for an IMU that we aren't using right now (2/10/2018)

"""partial based on http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber"""
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point32
import logging
import time
import serial
import pynmea2

class ublox(object):

    def __init__(self):
        self.ser = serial.Serial("/dev/ttyACM0", baudrate=9600, timeout=0.5)

        data = self.ser.readline()

        if data[0:6] == '$GNGGA':

            msg = pynmea2.parse(data)

            self.latval = msg.lat

            self.longval = msg.long

    def publish_position(self):
    	pub = rospy.Publisher('/boat/position',Point32, queue_size=0)
    	rospy.init_node('position', anonymous = True)
    	rate = rospy.Rate(10) # 10hz
    	while not rospy.is_shutdown():
      		# Read the Euler angles for heading, roll, pitch (all in degrees).
    		while 1:
                	try:
                    		data = self.ser.readline()
                	except:

                		if data[0:6] == '$GPGGA':

                    			msg = pynmea2.parse(data)

                    			if msg.lat[0]=="0":
                        			msg.lat[0]="-"
                        			self.latval=float32(msg.lat[0:7])
            
                    			else:
                        			self.latval = float32(msg.lat[0:6])
                    	

                    			if msg.lon[0]=="0":
                        			msg.lon[0]="-"
                        			self.longval=float32(msg.lon[0:7])
                    	
                    			else:
                        			self.longval = float32(msg.lat[0:6])
                    	


                    			rospy.loginfo(Point32({x: self.longval,y: self.latval,z:0.0}))

        				pub.publish(Point32({x: self.longval,y: self.latval,z: 0.0}))

        			rate.sleep()

if __name__ == '__main__':
	potemnkin=ublox()
	try:
		potemnkin.publish_position()
	except rospy.ROSInterruptException:
		pass
