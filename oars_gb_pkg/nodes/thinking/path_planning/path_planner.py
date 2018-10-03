#!/usr/bin/env python
import rospy
from array import array
from geometry_msgs.msg import Pose2D
from oars_gb.msg import GridMap, WaypointList
from oars_gb_pkg.helpers.path_planning import AStarPlanner, Grid
from oars_gb_pkg.helpers.path_planning.waypoint_generator import gps_coords_to_cell, cell_to_gps_coords, make_waypoints


class PathPlanner:

    def __init__(self, using_ros=True):
        self.current_pos = None
        self.goal_pos = None
        self.wind_angle = None
        self.wind_speed = None
        self.grid_map = None
        self.grid_lower_left_coord = None
        self.grid_upper_right_coord = None
        self.using_ros = using_ros

        if self.using_ros:
            # Register the node
            rospy.init_node('path_planner', anonymous=True)

            # Register the publishers and subscribers
            rospy.Subscriber('/planning/map', GridMap, self.received_grid_map_msg, queue_size=1)
            rospy.Subscriber('/boat/position', Pose2D, self.received_boat_pos_msg, queue_size=1)
            rospy.Subscriber('/planning/goal_pos', Pose2D, self.received_desired_pos_msg, queue_size=1)
            # NEED TO RECEIVE WIND MSG
            self.waypoint_pub = rospy.Publisher('/planning/waypoints', WaypointList, queue_size=1)

            print('Waypoint planner node initialized')

            # Now just twiddle our thumbs until we need to do something
            r = rospy.Rate(2)
            while not rospy.is_shutdown():
                self.plan_path()
                r.sleep()

    def received_boat_pos_msg(self, msg):
        """
        Called when a ROS message is received containing the boat's current GPS coordinates.
        :param msg: a ROS message where x and y correspond to the longitude and latitude, respectively, of the boat
        :type msg: Pose2D
        """
        print('received_boat_pos_msg')
        self.current_pos = (msg.x, msg.y)

    def received_desired_pos_msg(self, msg):
        """
        Called when a ROS message is received containing the goal position's GPS coordinates.
        :param msg: a ROS message where x and y correspond to the longitude and latitude, respectively
        :type msg: Pose2D
        """
        print('received_desired_pos_msg')
        self.goal_pos = (msg.x, msg.y)

    def received_wind_msg(self, msg):
        """
        Called when a ROS message is received containing information about the wind.
        :param msg: a ROS message where x and theta correspond to the speed and direction, respectively, of the wind
        :type msg: Pose2D
        """
        self.wind_speed = msg.x.data
        self.wind_angle = msg.theta.data

    def received_grid_map_msg(self, msg):
        """
        Called when a ROS message is received containing a GridMap.
        :param msg: the ROS message
        :type msg: GridMap
        """
        # Convert the ROS Image message to a Grid
        print('received_boat_pos_msg')
        msg.grid.data = list(array("B", msg.grid.data))
        self.grid_map = Grid(msg.grid)
        self.grid_lower_left_coord = (msg.minLongitude, msg.minLatitude)
        self.grid_upper_right_coord = (msg.maxLongitude, msg.maxLatitude)

    def plan_path(self):
        """
        Calculates the best course to follow given the boat's current position, desired end location, and environment
        conditions.
        :return: a list of waypoints (tuples) to navigate, or None if we don't possess the needed inputs
        """
        # Make sure we have the info we need to plan a path
        if self.current_pos is None or self.goal_pos is None or self.grid_map is None:
            # or self.wind_angle is None (once that exists)
            print('not enough information')
            return

        # TODO Incorporate actual wind direction in planning

        # Plan a path based on the map and our current location and environment conditions
        planner = AStarPlanner(self.grid_map)
        # Find an open starting and ending coordinate
        start = gps_coords_to_cell(self.current_pos, self.grid_lower_left_coord, self.grid_upper_right_coord,
                                   self.grid_map.width, self.grid_map.height)
        end = gps_coords_to_cell(self.goal_pos, self.grid_lower_left_coord, self.grid_upper_right_coord,
                                 self.grid_map.width, self.grid_map.height)
        print(start, end)
        while not self.grid_map.get_cell(start).is_water:
            start = (start[0] + 1, start[1] + 1)
        while not self.grid_map.get_cell(end).is_water:
            end = (end[0] - 1, end[1] - 1)
        print(start, end)
        # Run the path planner and save the path in an image
        path = planner.plan(start, end)
        waypoints = make_waypoints(path)
        gps_waypoints_lat = []
        gps_waypoints_long = []
        gps_waypoints = None
        for point in waypoints:
            gps_point = cell_to_gps_coords(point, self.grid_lower_left_coord, self.grid_upper_right_coord,
                                           self.grid_map.width, self.grid_map.height)
            gps_waypoints_lat.append(gps_point[0])
            gps_waypoints_lat.append(gps_point[1])
        if self.using_ros:
            gps_waypoints = WaypointList(latitudes=gps_waypoints_lat, longitudes=gps_waypoints_long)
            self.waypoint_pub.publish(gps_waypoints)
            print('published')
        return gps_waypoints


if __name__ == "__main__":
    PathPlanner()
