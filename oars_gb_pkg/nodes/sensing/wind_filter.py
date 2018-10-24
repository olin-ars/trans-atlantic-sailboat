#!/usr/bin/env python
import rospy
import numpy as np
from collections import deque
from std_msgs.msg import UInt8
from geometry_msgs.msg import Pose2D

"""
This ROS node subscribes to the output node of the Airmar sensor, on /sensors/airmar/wind/rel and 
/sensors/airmar/wind/true, and stores on a rolling basis the most recent 10 values into a deque (a 
rolling queue). An average function is performed on that deque, and the resulting values are published
to /weather/wind/rel and /weather/wind/true
"""

class WindFilter:
    def __init__(self):
        
        # Initialize the ROS node
        rospy.init_node("wind_filter", anonymous=True)
        rospy.loginfo("Starting wind filter node ...")
        rospy.loginfo("--------------")
        
        # Initialize publishers
        # Filtered relative wind speed (knots) and direction
        self.pub_rel = rospy.Publisher('/weather/wind/rel', Pose2D, queue_size=10)
        # Filtered true wind speed and direction
        self.pub_true = rospy.Publisher('/weather/wind/true', Pose2D, queue_size=10)
        
        # Create deques to store the last 10 values
        self.wind_rel_deque = {'x': deque(maxlen=10), 'theta': deque(maxlen=10)}
        self.wind_true_deque = {'x': deque(maxlen=10), 'theta': deque(maxlen=10)}
                
    def run(self):
        """
        This function loops until the node is shut down, reading messages sent from the Airmar
        and publishing their contents to the appropriate ROS topics.
        """
        while not rospy.is_shutdown():
            # Subscribe to airmar node's topics
            rospy.Subscriber("/sensors/airmar/wind/rel", Pose2D, self.received_rel_wind_msg)
            rospy.Subscriber("/sensors/airmar/wind/true", Pose2D, self.received_true_filter_msg)
            # Subscribe to the filter's window size controller topic
            rospy.Subscriber("/control/wind_filter/window_size", UInt8, self.change_window_size)
            
            # Run callbacks
            rospy.spin()


    def change_window_size(self, new_size):
        """
            Resizes the filter window size based on value (new_size) specified on controller channel
        """
        new_size = new_size.data

        # Check if window resizing is required
        if self.wind_rel_deque['x'].maxlen != new_size:
            # Resize windows for the relative wind speed and heading window
            self.wind_rel_deque['x'] = deque(self.wind_rel_deque['x'], maxlen=new_size)
            self.wind_rel_deque['theta'] = deque(self.wind_rel_deque['theta'], maxlen=new_size)

            # Resize windows for the true wind speed and heading window
            self.wind_true_deque['x'] = deque(self.wind_true_deque['x'], maxlen=new_size)
            self.wind_true_deque['theta'] = deque(self.wind_true_deque['theta'], maxlen=new_size)

    def received_rel_wind_msg(self, rel_wind):
        """
            Takes Pose2D message of raw relative wind speed and heading data, and uses a rolling average filter.
            Publishes filtered relative wind and heading data as a Pose2D message to ROS.
        """
        # Append new value to deque
        self.wind_rel_deque['x'].append(rel_wind.x)
        self.wind_rel_deque['theta'].append(rel_wind.theta)

        # Check number of stored values in window
        len_check = self.wind_rel_deque['x'].maxlen

        # Only run filter if the deque has already stored 10 values (passed initialization period)
        if (len(self.wind_rel_deque['x']) == len_check and len(self.wind_rel_deque['theta']) == len_check):
            filtered_rel_wind = Pose2D() # Create Pose2D message
    
            # Take means of the deque by converting to a numpy array
            # Stores mean values into outgoing message
            filtered_rel_wind.x = np.array(self.wind_rel_deque['x']).mean()
            filtered_rel_wind.theta = np.array(self.wind_rel_deque['theta']).mean()
    
            # Publish averaged values of wind data
            logmsg = 'Relative Wind Speed\nx      ' + str(filtered_rel_wind.x) + 'Theta  ' + str(filtered_rel_wind.theta)
            rospy.loginfo('\n' + logmsg + '\n')
            self.pub_rel.publish(filtered_rel_wind)

    def received_true_filter_msg(self, true_wind):
        """
            Takes Pose2D message of raw relative wind speed and heading data, and uses a rolling average filter.
            Publishes filtered relative wind and heading data as a Pose2D message to ROS.
        """
        # Append new value to deque
        self.wind_true_deque['x'].append(true_wind.x)
        self.wind_true_deque['theta'].append(true_wind.theta)

        # Check number of stored values in window
        len_check = self.wind_rel_deque['x'].maxlen

        # Only run filter if the deque has already stored 10 values (passed initialization period)
        if (len(self.wind_true_deque['x']) == len_check and len(self.wind_true_deque['theta']) == len_check):
            filtered_true_wind = Pose2D() # Create Pose2D message
    
            # Take means of the deque by converting to a numpy array
            # Stores mean values into outgoing message
            filtered_true_wind.x = np.array(self.wind_true_deque['x']).mean()
            filtered_true_wind.theta = np.array(self.wind_true_deque['theta']).mean()
    
            # Publish averaged values of wind data
            logmsg = 'True Wind Speed\nx      ' + str(filtered_true_wind.x) + '\nTheta  ' + str(filtered_true_wind.theta)
            rospy.loginfo('\n' + logmsg + '\n')
            self.pub_true.publish(filtered_true_wind)

if __name__ == '__main__':
    crash_count = 0
    while crash_count < 5:
        try:
            core = WindFilter()
            core.run()
        except rospy.ROSInterruptException:
            crash_count += 1
            print('Wind filter node was interrupted.')
    print('Wind filter node crashed.')
