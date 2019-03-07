"""
This program implements the short course routing algorithm described in Section 5.1 of Roland Stelzer's Autonomous
Sailboat Navigation paper. This algorithm uses a polar efficiency function to determine the efficiency of all possible
angles that a boat can turn, and then compares the efficiencies to determine the optimal boat heading.

Read the paper here: https://www.dora.dmu.ac.uk/bitstream/handle/2086/7364/thesis-optimized-300dpi.pdf
"""
import numpy as np
import math
import path_functions as pf
from path_functions import ShortCoursePlanner

'''
Runs the sample course provided below and displays the resulting path using Turtle.
Also uses the sleep function to simulate the passage of time (to determine how
fast/efficient the path actually is).
'''

def recalculate_path(wind_angle, angle_ranges, original_path):
    max_velocity = 0
    max_abs_angle = 0
    magnitude = lambda arr: math.sqrt(arr[0]**2 + arr[1]**2)

    for angle_range in angle_ranges:
        max_angle = angle_range[1] + wind_angle
        min_angle = angle_range[0] + wind_angle

        max_ang_velocity = ShortCoursePlanner._get_polar_efficiency(planner, wind_angle, max_angle)
        print("max_ang_velocity: ", magnitude(max_ang_velocity))
        min_ang_velocity = ShortCoursePlanner._get_polar_efficiency(planner, wind_angle, min_angle)
        print("min ang velocity: ", magnitude(min_ang_velocity))

        if magnitude(max_ang_velocity) > max_velocity:
            max_velocity = magnitude(max_ang_velocity)
            max_abs_angle = max_angle - wind_angle
        if magnitude(min_ang_velocity) > max_velocity:
            max_velocity = magnitude(min_ang_velocity)
            max_abs_angle = min_angle - wind_angle

    print("max abs angle: ", max_abs_angle)
    path_distance = magnitude(original_path)
    print("path distance: ", path_distance)
    optimal_path = np.array((2*path_distance*math.cos(max_abs_angle*math.pi/180), 2*path_distance*math.sin(max_abs_angle*math.pi/180)))
    print("optimal path: ", optimal_path)
    dot_product = np.dot(optimal_path, original_path)
    print("dot_product: ", dot_product)
    optimal_path_distance = magnitude(optimal_path)
    projected_path = ((dot_product/optimal_path_distance**2)*optimal_path[0], (dot_product/optimal_path_distance**2)*optimal_path[1])

    return projected_path

if __name__ == '__main__':

    from turtle import *
    from time import sleep
    import math

    planner = ShortCoursePlanner()
    # Asks user to input a wind angle
    windDir = int(input("wind angle: "))

    # Initializes the boat's current direction, target path
    boatCurrDir = 0
    targetList = [(0, 120)]  # (120, 120), (0, 0), (120, 0), (120, -120), (0, -120), (0, 0)]

    # Draws the wind vector and positions/orients it correctly
    wind = Turtle()
    wind.pu()
    wind.setpos(250 * np.cos(windDir * np.pi / 180), 250 * np.sin(windDir * np.pi / 180))

    # Draw the boat vector and orients it correctly
    boat = Turtle()
    boat.setheading(boatCurrDir)

    r_min = 60
    r_max = 120
    obstacles = [["obj", 110, 80]]


    obstacle_drawer = Turtle()
    obstacle_drawer.pencolor("red")
    obstacle_drawer.speed(0)
    for obstacle in obstacles:
        start_angle = obstacle[2]
        end_angle = obstacle[1]
        for angle in range(start_angle, end_angle):
            obstacle_drawer.pu()
            obstacle_drawer.goto(0, 0)
            obstacle_drawer.seth(angle)
            obstacle_drawer.pd()
            obstacle_drawer.fd(120)

    target_drawer = Turtle()
    target_drawer.pencolor("blue")
    target_drawer.fillcolor("blue")
    for targetPos in targetList:
        target_drawer.pu()
        target_drawer.goto(targetPos)
        target_drawer.pd()
        target_drawer.begin_fill()
        target_drawer.circle(10)
        target_drawer.end_fill()
    target_drawer.hideturtle()

    for targetPos in targetList:

        original_path = np.array(targetPos) - np.array(boat.position())
        print("Original Path: ", original_path)
        if (original_path[0] != 0):
            path_angle = math.atan(original_path[1] / original_path[0]) * 180 / math.pi
        else:
            path_angle = 90

        angles_ranges = []

        path = original_path
        if (len(obstacles) > 0):
            for obstacle in obstacles:
                if (obstacle[2] < path_angle < obstacle[1]):
                    angle_range = (obstacle[2] - 5, obstacle[1] + 5)
                    print("angle_range: ", angle_range)
                    angles_ranges.append(angle_range)

            path = recalculate_path(windDir, angles_ranges, original_path)
            print("Recalculated Path: ", path)

            targetPos = path #- boat.pos()
            print("New target pos: ", targetPos)

            target_drawer.pu()
            target_drawer.goto(targetPos)
            target_drawer.pd()
            target_drawer.begin_fill()
            target_drawer.circle(10)
            target_drawer.end_fill()
            target_drawer.hideturtle()

        print("Current Target: ", targetPos)
        while boat.distance(targetPos) > 5:

            velocities = []  # initialize_velocities()
            # print("Initialized Velocities: ", velocities)

            boatNewDir = planner.run(path, windDir, boat.heading(), boat.position())
            print(boatNewDir)

            boat.setheading(boatNewDir)
            boat.forward(4)
            sleep(0.05)
            boat.position()

            wind.setheading(windDir + 180)
