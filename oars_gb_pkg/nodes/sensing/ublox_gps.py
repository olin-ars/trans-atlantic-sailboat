#!/usr/bin/env python
# -*- coding: utf-8 -*-

#This code uses teh ublox usb  gps and

"""partial based on http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber"""
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point32
import logging
import time
import serial
import pynmea2

class ublox(object):
    #init function just gets an intial value so that is ros not surprised later
    def __init__(self):
        #Is the place that tehegps data is read from
        self.ser = serial.Serial("/dev/ttyACM0", baudrate=9600, timeout=0.5)
        #reads a line from the serial terminal
        data = self.ser.readline()

        #GNGGA is the code for the data that holds the longitude and latitude data
        if data[0:6] == '$GNGGA':

            #pynmea2 is able to take in the line of data and put out just the longitude and latitude numbers
            msg = pynmea2.parse(data)

            #saves the latitude and longitude data
            self.latval = msg.lat

            self.longval = msg.long

    def publish_position(self):
        #Uses a Point32 type to save the gps position, x is longitude, y is latitude, z is always set to zero
    	pub = rospy.Publisher('position',Point32, queue_size=0)
    	rospy.init_node('position', anonymous = True)
    	rate = rospy.Rate(10) # 10hz
    	while not rospy.is_shutdown():
            #repeats the process in the init function over and over again
    		while 1:
                	try:
                    		data = self.ser.readline()
                	except:

                		if data[0:6] == '$GPGGA':

                    			msg = pynmea2.parse(data)

                                #if the first number of fthe latitude or longitude is 0 then the number is negative
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
