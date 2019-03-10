"""
This program implements the short course routing algorithm described in Section 5.1 of Roland Stelzer's Autonomous
Sailboat Navigation paper. This algorithm uses a polar efficiency function to determine the efficiency of all possible
angles that a boat can turn, and then compares the efficiencies to determine the optimal boat heading.

Read the paper here: https://www.dora.dmu.ac.uk/bitstream/handle/2086/7364/thesis-optimized-300dpi.pdf
"""

import numpy as np
import path_functions as pf
import math

'''
Runs the sample course provided below and displays the resulting path using Turtle.
Also uses the sleep function to simulate the passage of time (to determine how
fast/efficient the path actually is).
'''

if __name__ == '__main__':
    # Initialize planner object
    planner = pf.ShortCoursePlanner()
    # Asks user to input a wind angle
    windDir = int(input("wind angle: "))
    # Initializes the boat's current direction, target path
    boatCurrDir = 0
    # Initializes the list of waypoints (target positions)
    target_list = [(0, 120)]

    # Draw the boat; initialize boat Turtle object
    boat, wind = pf.draw_wind_boat(windDir, boatCurrDir)

<<<<<<< HEAD
    # TODO: read from ROS node
    obstacles = [["obj", 110, 80]]

    # Draw target; initialize target_drawer Turtle object
    target_drawer = pf.obstacle_draw(obstacles, target_list)

    for target_pos in target_list:

        path = np.array(target_pos) - np.array(boat.position())
        print("Original Path: ", path)
        if path[0] != 0:
            path_angle = math.atan(path[1] / path[0]) * 180 / math.pi
        else:
            path_angle = 90

        # Identifies the angle ranges for the obstacles
        angles_ranges = []
        if len(obstacles) > 0:
            for obstacle in obstacles:
                if obstacle[2] < path_angle < obstacle[1]:
                    angle_range = (obstacle[2] - 5, obstacle[1] + 5)
                    print("angle_range: ", angle_range)
                    angles_ranges.append(angle_range)

            path = pf.recalculate_path(windDir, angles_ranges, path, planner)

            print("Recalculated Path: ", path)

            target_pos = path  # - boat.pos()
            print("New target pos: {}".format(target_pos))

            pf.target_draw_circle(target_drawer, target_pos)

        print("Current Target: ", target_pos)
        while boat.distance(target_pos) > 5:
            # velocities = []  # initialize_velocities()
            # # print("Initialized Velocities: ", velocities)

            boatNewDir = planner.run(path, windDir, boat.heading(), boat.position())
            print(boatNewDir)
            pf.update_boat(boat, wind, boatNewDir)
