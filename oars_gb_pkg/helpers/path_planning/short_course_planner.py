"""
This program implements the short course routing algorithm described in Section 5.1 of Roland Stelzer's Autonomous
Sailboat Navigation paper. This algorithm uses a polar efficiency function to determine the efficiency of all possible
angles that a boat can turn, and then compares the efficiencies to determine the optimal boat heading.

Read the paper here: https://www.dora.dmu.ac.uk/bitstream/handle/2086/7364/thesis-optimized-300dpi.pdf
"""
import numpy as np

def _get_best_dir(vt_max_r, vt_max_l, windangle_max_r, windangle_max_l, p_c, path, boat_heading):
    """
    Checks whether a port tack or starboard tack would be more efficient.

    :param vt_max_r: the maximum velocity on the right hand side
    :param vt_max_l: the maximum velocity on the left hand side
    :param windangle_max_r: angle (in degrees) corresponding to the max velocity on the right hand side
    :param windangle_max_l: angle (in degrees) corresponding to the max velocity on the left hand side
    :param p_c: beating parameter (controls the length of the tacks)
    :param path: the path the boat is following as (a numpy array)
    :param boat_heading: the boat's heading, relative to 0 (with 0 as east)
    :return: the new boat direction and speed
    """

    # print("Determining best direction based on hysteresis")

    # Calculates the hysteresis factor
    n = 1 + (p_c / (abs(np.sqrt(path[0] ** 2 + path[1] ** 2))))

    # Choose the direction that is closest to the current boat heading
    r_diff = 180 - abs(abs(windangle_max_r - boat_heading) - 180)
    l_diff = 180 - abs(abs(windangle_max_l - boat_heading) - 180)
    if r_diff < l_diff:
        # Choose the faster velocity
        if vt_max_r * n < vt_max_l:
            new_boat_heading = windangle_max_l
        else:
            new_boat_heading = windangle_max_r
    else:
        if vt_max_l * n < vt_max_r:
            new_boat_heading = windangle_max_r
        else:
            new_boat_heading = windangle_max_l

    return new_boat_heading

class ShortCoursePlanner:

    def _get_polar_efficiency(self, wind_angle, alpha):
        """
        Calculates the hypothetical speed for a given angle alpha, using the polar efficiency function.

        :param wind_angle: the wind direction in degrees (relative to 0 degrees, with 0 degrees being east)
        :param alpha: angle being tested (relative to the wind angle)
        :return: the vector components for the velocity for the angle as a numpy array
        """
        # Convert angle to radians
        angle = (wind_angle + alpha) * np.pi / 180
        alpha *= np.pi / 180

        # Calculate magnitude using the polar effeciency function
        mag = (1 - np.cos(alpha)) * (1 + 0.3 * np.cos(alpha)) / (1 - 0.5 * np.cos(alpha))

        # Return vector components as a numpy array
        return np.array([mag * np.cos(angle), mag * np.sin(angle)])

    def _get_optima(self, wind_angle, path):
        """
        Finds the maximum velocity and angle that can be achieved for every possible new heading for the boat to
        determine the most optimal headings on the port and starboard sides of the boat.

        :param wind_angle: angle representing the wind.
        :param path: vector representing the path taken to reach from boat's position to the target position
        :return: A tuple containing the following information:
           windangle_max_r - max boat heading on right side
            windangle_max_l - max boat heading on left side
            vt_max_r - max velocity on right side
            vt_max_l - max velocity on left side
        """

        # print("Getting optima")
        # Initializing the right side and left side optimum wind angles
        windangle_max_r = wind_angle
        windangle_max_l = wind_angle

        # Initializing the right side and left side optimum max velocities
        vt_max_r = 0
        vt_max_l = 0

        # Initializes alpha, or the angle being tested (relative to the wind direction)
        alpha = 0

        # Checks all alphas between 0 and 180 and calculates the theoretical speeds
        # to find an optimal heading for a starboard tack
        for alpha in range(180):

            # Uses the polar effeciency function to calculcate the hypothetical velocity of the boat at that angle
            vb_hyp = self._get_polar_efficiency(wind_angle, alpha)

            # Projecting the max speed in the direction of the path
            vt_test = np.dot(vb_hyp, path)

            # velocities[alpha] = vt_test

            # Check if this velocity is the new max
            if vt_test > vt_max_r:
                vt_max_r = vt_test
                windangle_max_r = (wind_angle + alpha) % 360

        # Analogous to the starboard tack, but for a port tack
        for alpha in range(180, 360):
            vb_hyp = self._get_polar_efficiency(wind_angle, alpha)
            vt_test = np.dot(vb_hyp, path)
            # velocities[alpha] = vt_test
            if vt_test > vt_max_l:
                vt_max_l = vt_test
                windangle_max_l = (wind_angle + alpha) % 360

        return vt_max_r, vt_max_l, windangle_max_r, windangle_max_l

    def run(self, path, w, b_h):
        """
        This method directs the boat by using the boat's position and the target position to calculate the path
        and comparing the boat's heading to the wind heading to determine the most efficient way for the boat to get
        to its target position.

        :param path - the boat's path as a vector
        :param b_h - the boat's current heading as an angle in degrees
        :param w - the wind angle in degrees, relative to 0 (with 0 as east)

        :return new_dir - the boat's new direction as a numpy array
        """

        # Calculates the path of the boat
        # path = np.array(t) - np.array(b_p)
        # print("Path: ", path)

        # The beating parameter (which controls the length of a tack)
        p_c = 20

        # Runs the optimum function and compares the optima returned to determine the best direction for
        # boat to go
        a, b, c, d = self._get_optima(w, path)
        new_dir = _get_best_dir(a, b, c, d, p_c, path, b_h)

        return new_dir



def magnitude(arr):
    '''
    Calculates the magnitude of a vector with 2 components.
    :param arr: a vector (represented as a list or numpy array)
    :return: the magnitude of the vector
    '''
    return math.sqrt(arr[0]**2 + arr[1]**2)

def recalculate_path(wind_angle, angle_ranges, original_path):
    '''
    Recalculates the path to avoid obstacles.

    :param wind_angle: the angle of the wind relative to 0 degrees (with east as 0 degrees)
    :param angle_ranges: a list of angle ranges (tuples with a min and max angle defining the range) that should be avoided (corresponding to obstacles)
    :param original_path: the original path of the boat, represented as a vector

    :return: the new path of the boat, represented as a vector
    '''

    # Determines the best path to take based on velocity & angle
    max_velocity = 0
    max_abs_angle = 0

    for angle_range in angle_ranges:
        # Calculates the bounds for the given angle range & resets frame of reference to absolute frame of reference
        # rather than with reference to the wind angle
        max_angle = angle_range[1] + wind_angle
        min_angle = angle_range[0] + wind_angle

        # Calculates the hypothetical velocity of the boat on either end of the range (to help determine the best angle)
        max_ang_velocity = ShortCoursePlanner._get_polar_efficiency(ShortCoursePlanner, wind_angle, max_angle)
        # print("max_ang_velocity: ", magnitude(max_ang_velocity))
        min_ang_velocity = ShortCoursePlanner._get_polar_efficiency(ShortCoursePlanner, wind_angle, min_angle)
        # print("min ang velocity: ", magnitude(min_ang_velocity))

        # Updates the best velocity found thus far
        if magnitude(max_ang_velocity) > max_velocity:
            max_velocity = magnitude(max_ang_velocity)
            max_abs_angle = max_angle - wind_angle
        if magnitude(min_ang_velocity) > max_velocity:
            max_velocity = magnitude(min_ang_velocity)
            max_abs_angle = min_angle - wind_angle

    # print("max abs angle: ", max_abs_angle)
    # Calculates the distance of the original path
    path_distance = magnitude(original_path)
    # print("path distance: ", path_distance)

    # Calculates the new optimal path -- a vector in the direction of the best angle found that is twice as long as the
    # distance of the path (to make projection easier)
    optimal_path = np.array((2*path_distance*math.cos(max_abs_angle*math.pi/180), 2*path_distance*math.sin(max_abs_angle*math.pi/180)))
    # print("optimal path: ", optimal_path)

    # Projects the original path onto the new path
    dot_product = np.dot(optimal_path, original_path)
    # print("dot_product: ", dot_product)
    optimal_path_distance = magnitude(optimal_path)

    # Calculates a vector representing the new path
    projected_path = ((dot_product/optimal_path_distance**2)*optimal_path[0], (dot_product/optimal_path_distance**2)*optimal_path[1])

    return projected_path


'''
Runs the sample course provided below and displays the resulting path using Turtle.
Also uses the sleep function to simulate the passage of time (to determine how 
fast/efficient the path actually is).
'''
if __name__ == '__main__':

    from turtle import *
    from time import sleep
    import math

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

    # Define the obstacles
    obstacles = [["obj", 110, 80]]

    # Draws the obstacles
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

    # Draws the target positions
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

        # Calculates the original path
        original_path = np.array(targetPos) - np.array(boat.position())
        print("Original Path: ", original_path)

        # Calculates the angle of the original path
        if (original_path[0] != 0):
            path_angle = math.atan(original_path[1] / original_path[0]) * 180 / math.pi
        else:
            path_angle = 90

        # Identifies the angle ranges for the obstacles
        angles_ranges = []
        path = original_path
        if (len(obstacles) > 0):
            for obstacle in obstacles:
                if (obstacle[2] < path_angle < obstacle[1]):
                    angle_range = (obstacle[2] - 5, obstacle[1] + 5)
                    print("angle_range: ", angle_range)
                    angles_ranges.append(angle_range)

            # If obstacles are found, recalculates path to avoid the identified angle rangles
            path = recalculate_path(windDir, angles_ranges, original_path)
            print("Recalculated Path: ", path)

            # Recalculates the target position
            targetPos = boat.pos() + path
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
            planner = ShortCoursePlanner()

            velocities = []  # initialize_velocities()
            # print("Initialized Velocities: ", velocities)

            boatNewDir = planner.run(path, windDir, boat.heading())

            boat.setheading(boatNewDir)
            boat.forward(4)
            sleep(0.05)
            boat.position()

            wind.setheading(windDir + 180)
