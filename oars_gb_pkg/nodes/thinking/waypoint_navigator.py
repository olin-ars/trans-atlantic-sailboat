#!/usr/bin/env python

import math
from collections import deque
from oars_gb_pkg.helpers.geo import dist_between_points
try:
    import rospy
    from std_msgs.msg import Float32, UInt16, Float32MultiArray
    from geometry_msgs.msg import Pose2D
    from oars_gb.msg import WaypointList
except ImportError:
    print('WARNING: Python ROS packages not found. Running in unit testing mode...')


class WaypointNavigator:
    """
    A ROS node for keeping track of which waypoints we are navigating using and for publishing
    the heading the boat should be on to reach the next waypoint.
    """

    def __init__(self, waypoint_radius=5, use_ros=True):
        """
        Initializes a new WaypointNavigator. If running in a ROS environment, a node will be
        started. Otherwise ROS functionality will be disabled to allow for Python unit testing.
        :param waypoint_radius: the proximity threshold (in meters) determining when a waypoint
         is considered "reached"
        :type waypoint_radius: float
        :param use_ros: If True, will attempt initialization of ROS node and registration
        of publishers and subscribers. If False, will run in ROS-less unit testing mode.
        """
        self.using_ros = True if rospy is not None and use_ros else False
        self.longitude = None
        self.latitude = None
        self.wp_list = None
        self.next_wp = None
        self.waypoint_radius = waypoint_radius

        if self.using_ros:  # False if we're doing ROS-less unit testing
            rospy.init_node('waypoint_navigator', anonymous=True)
            self.heading_pub = rospy.Publisher('/control/desired_heading', Float32, queue_size=0)
            rospy.Subscriber('/boat/position', Pose2D, self.update_location, queue_size=1)
            rospy.Subscriber('/planning/waypoint_radius', UInt16, self.set_waypoint_radius, queue_size=1)
            rospy.Subscriber('/planning/waypoints', WaypointList, self.update_wp_list, queue_size=1)
            self.waypoint_pub = rospy.Publisher('/planning/waypoints', WaypointList, queue_size=0)
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
            if self.using_ros:
                self.waypoint_pub.publish(self._generate_waypoint_list_msg())
            self.next_wp = self.wp_list.popleft()  # Progress to the next waypoint
        if self.using_ros and self.next_wp:  # Don't do if we're not using ROS or if we don't know the next waypoint yet
            self.heading_pub.publish(self.calculate_desired_heading())

    def have_reached_wp(self):
        """
        Checks if the boat has reached the next waypoint.
        :return: True if the boat is within the waypoint-reached radius of the next waypoint,
        False otherwise.
        """
        if self.next_wp is None or self.longitude is None:
            return False
        current_pos = (self.longitude, self.latitude)
        return dist_between_points(self.next_wp, current_pos) <= self.waypoint_radius

    def _generate_waypoint_list_msg(self):
        lats = []
        longs = []
        for coord in list(self.wp_list):
            longs.append(coord[0])
            lats.append(coord[1])
        return WaypointList(latitudes=Float32MultiArray(data=lats), longitudes=Float32MultiArray(data=longs))

    def update_wp_list(self, wp_list_msg):
        """
        Updates the list of waypoints which the boat should be using for navigation. If running in
        a ROS environment, it will also trigger a desired heading recalculation and republish.
        :param wp_list_msg: a list of tuples specifying GPS coordinates in the form (long, lat)
        :type wp_list_msg: WaypointList
        """
        # Use queue of tuples for waypoints
        self.wp_list = deque(zip(wp_list_msg.longitudes.data, wp_list_msg.latitudes.data))
        self.next_wp = self.wp_list.popleft()  # Get the next waypoint
        if self.using_ros and self.latitude:  # Don't do if not using ROS or if we don't know our location yet
            self.heading_pub.publish(self.calculate_desired_heading())

    def calculate_desired_heading(self):
        """
        Calculates the heading the boat should be on given its current GPS coordinates
        and the coordinates of the next waypoint.
        :return: the degree heading in the range [0, 360), where 0 is north and 90 is east
        :rtype: float
        """
        if self.longitude is None:
            return
        delta_x = self.next_wp[0] - self.longitude
        delta_y = self.next_wp[1] - self.latitude
        return math.degrees(math.atan2(delta_y, delta_x))

    def set_waypoint_radius(self, msg):
        """
        Updates the waypoint radius (proximity threshold) in response to a message from an observer.
        :param msg: the ROS message giving the new radius
        :type msg: UInt8
        """
        self.waypoint_radius = msg.data


if __name__ == '__main__':
    WaypointNavigator()
