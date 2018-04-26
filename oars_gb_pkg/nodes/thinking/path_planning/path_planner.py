#!/usr/bin/env python
import rospy
from array import array
from std_msgs.msg import Float32MultiArray, Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from oars_gb.msg import GridMap, WaypointList
from oars_gb_pkg.helpers.path_planning import *
from oars_gb_pkg.helpers.path_planning.waypoint_generator import *


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
            rospy.Subscriber('/weather/wind/true', Pose2D, self.received_wind_msg, queue_size=1)
            self.image_pub = rospy.Publisher('/planning/image', Image, queue_size=1)
            self.waypoint_pub = rospy.Publisher('/planning/waypoints', WaypointList, queue_size=1)

            print('Waypoint planner node initialized')

            # Now just twiddle our thumbs until we need to do something
            r = rospy.Rate(10)
            while not rospy.is_shutdown():
                self.plan_path()
                r.sleep()

    def received_boat_pos_msg(self, msg):
        """
        Called when a ROS message is received containing the boat's current GPS coordinates.
        :param msg: a ROS message where x and y correspond to the longitude and latitude, respectively, of the boat
        :type msg: Pose2D
        """
        print('Received_boat_pos_msg')
        self.current_pos = (msg.x, msg.y)

    def received_desired_pos_msg(self, msg):
        """
        Called when a ROS message is received containing the goal position's GPS coordinates.
        :param msg: a ROS message where x and y correspond to the longitude and latitude, respectively
        :type msg: Pose2D
        """
        print('Received_desired_pos_msg')
        self.goal_pos = (msg.x, msg.y)

    def received_wind_msg(self, msg):
        """
        Called when a ROS message is received containing information about the wind.
        :param msg: a ROS message where x and theta correspond to the speed and direction, respectively, of the wind
        :type msg: Pose2D
        """
        print('Received wind velocity message')
        self.wind_speed = msg.x
        self.wind_angle = msg.theta

    def received_grid_map_msg(self, msg):
        """
        Called when a ROS message is received containing a GridMap.
        :param msg: the ROS message
        :type msg: GridMap
        """
        # Convert the ROS Image message to a Grid
        print('Received map message')
        msg.grid.data = list(array("B", msg.grid.data))
        self.grid_map = Grid(msg.grid)
        self.grid_lower_left_coord = (msg.minLongitude, msg.minLatitude)
        self.grid_upper_right_coord = (msg.maxLongitude, msg.maxLatitude)

    def draw_map(self, waypoints, path, width, height):
        """ Creates an image with the map image as a background and the cells in
            the final path highlighted in green. """
        header = Header()
        encoding = 'rgb8'
        step = 3 * width
        data = []

        for y in range(height):
            for x in range(width):
                if (x, y) in waypoints:
                    data.extend([254, 0, 0])
                elif (x, y) in path:
                    data.extend([254, 254, 254])
                elif self.grid_map.get_cell((x, y)).is_water:
                    data.extend([20, 80, 200])
                else:
                    data.extend([20, 200, 80])

        img = Image(header=header, height=height, width=width, encoding=encoding, \
                    is_bigendian=False, step=step, data=data)
        return img

    def plan_path(self):
        """
        Calculates the best course to follow given the boat's current position, desired end location, and environment
        conditions.
        :return: a list of waypoints (tuples) to navigate, or None if we don't possess the needed inputs
        """
        # Make sure we have the info we need to plan a path
        if self.current_pos is None or self.goal_pos is None \
            or self.wind_angle is None or self.grid_map is None:
            return

        # Plan a path based on the map and our current location and environment conditions
        print("Starting path planner...")
        planner = AStarPlanner(self.grid_map)
        width = self.grid_map.width
        height = self.grid_map.height
        # Find an open starting and ending coordinate
        start = gps_coords_to_cell(self.current_pos, self.grid_lower_left_coord, self.grid_upper_right_coord, width, height)
        end = gps_coords_to_cell(self.goal_pos, self.grid_lower_left_coord, self.grid_upper_right_coord, width, height)
        while not self.grid_map.get_cell(start).is_water:
            start = (start[0] + 1, start[1] + 1)
        while not self.grid_map.get_cell(end).is_water:
            end = (end[0] - 1, end[1] - 1)
        # Run the path planner and save the path in an image
        print("Planning path...")
        path = planner.plan(start, end, self.wind_angle)
        print("Getting waypoints...")
        waypoints = make_waypoints(path)
        gps_waypoints_lat = []
        gps_waypoints_lon = []
        for point in waypoints:
            gps_point = cell_to_gps_coords(point, self.grid_lower_left_coord, self.grid_upper_right_coord, width, height)
            gps_waypoints_lat.append(gps_point[0])
            gps_waypoints_lon.append(gps_point[1])

        gps_waypoints = WaypointList(
            latitudes=Float32MultiArray(data=gps_waypoints_lat),
            longitudes=Float32MultiArray(data=gps_waypoints_lon)
        )
        if self.using_ros:
            map_image = self.draw_map(waypoints, path, width, height)
            self.image_pub.publish(map_image)
            self.waypoint_pub.publish(gps_waypoints)
            print('Waypoints published')
        return gps_waypoints


if __name__ == "__main__":
    PathPlanner()
