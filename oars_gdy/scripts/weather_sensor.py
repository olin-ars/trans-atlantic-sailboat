#!/usr/bin/env python

"""
This node is responsible for reading weather equipment (wind speed, direction, etc)
and publishing the (basically raw) information to the appropriate topics.
"""

import rospy
from geometry_msgs.msg import Int8MultiArray


rospy.init_node('weather_station')

r = rospy.Rate(5)

wind_pub = rospy.Publisher('/rel_wind_vel', Int8MultiArray, queue_size=0)

while not rospy.is_shutdown():

    # TODO Get actual wind speed and direction from sensor
    wind_vel_msg = Int8MultiArray()
    wind_vel_msg.data[0] = 5
    wind_vel_msg.data[1] = 80

    # Publish the message
    wind_pub.publish(wind_vel_msg)

    # Do nothing for a bit
    r.sleep()
