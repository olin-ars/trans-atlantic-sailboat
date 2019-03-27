"""
This program implements the short course routing algorithm described in Section 5.1 of Roland Stelzer's Autonomous
Sailboat Navigation paper. This algorithm uses a polar efficiency function to determine the efficiency of all possible
angles that a boat can turn, and then compares the efficiencies to determine the optimal boat heading.

Read the paper here: https://www.dora.dmu.ac.uk/bitstream/handle/2086/7364/thesis-optimized-300dpi.pdf

@Authors: Duncan Mazza and Shreya Chowdhary
"""

import numpy as np
import path_functions as pf
import math
from turtle import Turtle

'''
Runs the sample course provided below and displays the resulting path using Turtle.
Also uses the sleep function to simulate the passage of time (to determine how
fast/efficient the path actually is).
'''

if __name__ == '__main__':
    # Initialize planner object
    planner = pf.ShortCoursePlanner()
    # Asks user to input a wind angle
    wind_dir = int(input("wind angle: "))
    # Initializes the boat's current direction, target path
    boat_curr_dir = 0
    # Initializes the list of waypoints (target positions)
    target_list = [(0, 120)]
    # Draw the boat; initialize boat Turtle object
    boat, wind = pf.draw_wind_boat(wind_dir, boat_curr_dir)

    # Initialize the target drawer turtle object
    target_drawer = Turtle()
    target_drawer.pencolor("blue")
    target_drawer.fillcolor("blue")

    # Initialize the obstacle drawer turtle object
    obstacle_drawer = Turtle()
    obstacle_drawer.pencolor("red")
    obstacle_drawer.speed(0)

    # TODO: read from ROS node
    obstacles = [["obj", 120, 70], ["obj2", 10, 0]]

    # Draw target; initialize target_drawer Turtle object
    pf.obstacle_draw(obstacles, target_list, obstacle_drawer, target_drawer)

    for target_pos in target_list:

        path = np.array(target_pos) - np.array(boat.position())
        print("Original Path: ", path)
        if path[0] != 0:
            path_angle = math.atan(path[1] / path[0]) * 180 / math.pi
        else:
            path_angle = 90

        flag = False  # for whether there are obstacles in the way
        # Identifies the angle ranges for the obstacles
        angles_ranges = []
        if len(obstacles) > 0:  # if there are obstacles
            for obstacle in obstacles:
                if obstacle[2] < path_angle < obstacle[1]:  # if there is an obstacle in the way
                    flag = True
                    angle_range = (obstacle[2] - 5, obstacle[1] + 5)
                    print("angle_range: ", angle_range)
                    angles_ranges.append(angle_range)

            if flag:  # if there are obstacles in the way
                path = pf.recalculate_path(wind_dir, angles_ranges, path, planner)
                print("Recalculated Path: ", path)
                target_pos = path  # - boat.pos()
                print("New target pos: {}".format(target_pos))
                pf.target_draw_circle(target_pos, target_drawer)

        print("Current Target: ", target_pos)
        while boat.distance(target_pos) > 5:  # update the boat position
            boat_new_dir = planner.run(path, wind_dir, boat.heading(), boat.position())
            print(boat_new_dir)
            pf.update_boat(boat, wind, boat_new_dir)
