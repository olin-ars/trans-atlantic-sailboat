#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""partial based on http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber"""
import rospy
from geometry_msgs.msg import Vector3
import logging
from Adafruit_BNO055 import BNO055

# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=1)

# Enable verbose debug logging if -v is passed as a parameter.
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))

# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

def publish_orientation():
	pub = rospy.Publisher('/orientation',Vector3, queue_size=0)
	rospy.init_node('orientation', anonymous = True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
  		# Read the Euler angles for heading, roll, pitch (all in degrees).
		heading, roll, pitch = bno.read_euler()
		rospy.loginfo(Vector3(heading,roll,pitch))
		pub.publish(Vector3(heading,roll,pitch))
		rate.sleep()

if __name__ == '__main__':
	try:
		publish_orientation()
	except rospy.ROSInterruptException:
		pass