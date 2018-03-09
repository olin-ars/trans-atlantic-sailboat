#!/usr/bin/env python

import math
try:
    import rospy
    from std_msgs.msg import Float32
    from geometry_msgs.msg import Pose2D
except ImportError:
    print('WARNING: Python ROS packages not found. Running in unit testing mode...')


class WaypointNavigator:
    """
    A ROS node for keeping track of which waypoints we are navigating using and for publishing
    the heading the boat should be on to reach the next waypoint.
    """

    def __init__(self, waypoint_reached_radius=5, use_ros=True):
        """
        Initializes a new WaypointNavigator. If running in a ROS environment, a node will be
        started. Otherwise ROS functionality will be disabled to allow for Python unit testing.
        :param waypoint_reached_radius: the proximity to a waypoint that must be achieved
        before a waypoint is considered "reached"
        :type waypoint_reached_radius: float
        :param use_ros: If True, will attempt initialization of ROS node and registration
        of publishers and subscribers. If False, will run in ROS-less unit testing mode.
        """
        self.using_ros = True if rospy is not None and use_ros else False
        self.longitude = None
        self.latitude = None
        self.waypoint_reached_radius = waypoint_reached_radius
        self.wp_list = [(6, 7), (5, 12), (7, 24)]
        self.next_wp = (6, 7)
        if self.using_ros:  # False if we're doing ROS-less unit testing
            rospy.init_node('waypoint_navigator', anonymous=True)
            self.heading_pub = rospy.Publisher('/control/desired_heading', Float32, queue_size=0)
            rospy.Subscriber('/boat/position', Pose2D, self.update_location, queue_size=1)
        print('Waypoint navigator initialized')
        if self.using_ros:
            rospy.spin()  # Keep the code running until we receive a kill signal

    def update_location(self, msg):
        """
        Updates the location of the boat.
        :param msg: a ROS Pose2D message representing the GPS coordinates of the boat.
        (x is the longitude and y is the latitude component of the coordinates. theta is unused.)
        :type msg: Pose2D
        """
        self.longitude = msg.x
        self.latitude = msg.y
        # Check if boat GPS coordinates are within proximity radius to consider waypoint reached
        if self.have_reached_wp():
            self.wp_list = self.wp_list[1:]
            self.update_wp_list(self.wp_list)
            print('Reached Waypoint')
        if self.using_ros:
            self.publish_desired_heading(self.calculate_desired_heading())

    def have_reached_wp(self):
        """
        Checks if the boat has reached the next waypoint.
        :return: True if the boat is within the waypoint-reached radius of the next waypoint,
        False otherwise.
        """
        if self.next_wp is None:
            return False
        dist_to_wp = math.sqrt(((self.next_wp[0] - self.longitude)**2) + ((self.next_wp[1] - self.latitude)**2))
        return dist_to_wp <= self.waypoint_reached_radius

    def update_wp_list(self, wp_list):
        """
        Updates the list of waypoints which the boat should be using for navigation. If running in
        a ROS environment, it will also trigger a desired heading recalculation and republish.
        :param wp_list: a list of tuples specifying GPS coordinates in the form (long, lat)
        :type wp_list: # TODO
        """
        self.wp_list = wp_list
        self.next_wp = wp_list[0]
        if self.using_ros:
            self.publish_desired_heading(self.calculate_desired_heading())

    def calculate_desired_heading(self):
        """
        Calculates the heading the boat should be on given its current GPS coordinates
        and the coordinates of the next waypoint.
        :return: the heading in the range [0, 360), where 0 is north and 90 is east
        :rtype: float
        """
        if self.longitude is None:
            return
        delta_x = self.next_wp[0] - self.longitude
        delta_y = self.next_wp[1] - self.latitude
        return math.degrees(math.atan2(delta_y, delta_x))

    def publish_desired_heading(self, theta):
        """
        Attempts to publish the desired heading over ROS.
        :param theta: the heading in the range of [0, 360), where 0 is north and 90 is east
        :type theta: float
        """
        self.heading_pub.publish(theta)


if __name__ == '__main__':
    WaypointNavigator()
