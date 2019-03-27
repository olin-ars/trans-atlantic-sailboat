import numpy as np
from turtle import Turtle
import math
from time import sleep


def calc_obstacle_penalties(velocities, angles):
    for angle in angles:
        velocities[angle] = -100
    return velocities


def find_maximums(velocities):
    vt_max_r = -10000
    vt_max_l = -10000

    windangle_max_l = 0
    windangle_max_r = 0
    for angle in range(-90, 99):
        velocity = velocities[angle]
        print("(angle, velocity): ", angle, velocity)

        if velocity > vt_max_r:
            vt_max_r = velocity
            windangle_max_r = angle

    for angle in range(90, 270):
        print("(angle, velocity): ", angle, velocity)
        if velocity > vt_max_l:
            vt_max_l = velocity
            windangle_max_l = angle

    print("max windangle r, max windangle l: ", windangle_max_r, windangle_max_l)
    return vt_max_r, vt_max_l, windangle_max_r, windangle_max_l


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


def target_draw_circle(target_pos, target_drawer):
    target_drawer.pu()
    target_drawer.goto(target_pos)
    target_drawer.pd()
    target_drawer.begin_fill()
    target_drawer.circle(10)
    target_drawer.end_fill()
    target_drawer.ht()


def obstacle_draw(obstacles, target_list, obstacle_drawer_obj, target_drawer_obj):
    for obstacle in obstacles:
        start_angle = obstacle[2]
        end_angle = obstacle[1]
        for angle in range(start_angle, end_angle):
            obstacle_drawer_obj.pu()
            obstacle_drawer_obj.goto(0, 0)
            obstacle_drawer_obj.seth(angle)
            obstacle_drawer_obj.pd()
            obstacle_drawer_obj.fd(120)
    for target_pos in target_list:
        target_draw_circle(target_pos, target_drawer_obj)


def draw_wind_boat(wind_dir, direction):
    # Draws the wind vector and positions/orients it correctly
    wind = Turtle()
    wind.pu()
    wind.setpos(250 * np.cos(wind_dir * np.pi / 180), 250 * np.sin(wind_dir * np.pi / 180))

    # Draw the boat vector and orient it
    boat = Turtle()
    boat.setheading(direction)

    return boat, wind


def recalculate_path(wind_angle, angle_ranges, original_path, planner):
    # Initialize these at 0 so the maximum values can be calculated later
    max_velocity = 0
    max_abs_angle = 0

    # Magnitude function (euclidian length of a vector)
    def magnitude(arr):
        return math.sqrt(arr[0] ** 2 + arr[1] ** 2)

    for angle_range in angle_ranges:
        # for each pair of angles there is a max and a min angle value
        max_angle = angle_range[1] + wind_angle
        min_angle = angle_range[0] + wind_angle

        # Velocity of boat associated with following the larger angle of the pair
        max_ang_velocity = planner._get_polar_efficiency(wind_angle, max_angle)
        print("max_ang_velocity: ", magnitude(max_ang_velocity))

        # Velocity of the boat associated with following the smaller angle of the pair
        min_ang_velocity = planner._get_polar_efficiency(wind_angle, min_angle)
        print("min ang velocity: ", magnitude(min_ang_velocity))

        # Update the maximum and minimum velocities
        if magnitude(max_ang_velocity) > max_velocity:
            max_velocity = magnitude(max_ang_velocity)
            max_abs_angle = max_angle - wind_angle
        if magnitude(min_ang_velocity) > max_velocity:
            max_velocity = magnitude(min_ang_velocity)
            max_abs_angle = min_angle - wind_angle

    # The
    print("max abs angle: ", max_abs_angle)
    path_distance = magnitude(original_path)
    print("path distance: ", path_distance)
    optimal_path = np.array((2 * path_distance * math.cos(max_abs_angle * math.pi / 180),
                             2 * path_distance * math.sin(max_abs_angle * math.pi / 180)))
    print("optimal path: ", optimal_path)
    dot_product = np.dot(optimal_path, original_path)
    print("dot_product: ", dot_product)
    optimal_path_distance = magnitude(optimal_path)
    projected_path = ((dot_product / optimal_path_distance ** 2) * optimal_path[0],
                      (dot_product / optimal_path_distance ** 2) * optimal_path[1])

    return projected_path


def update_boat(boat, wind, dir):
    boat.setheading(dir)
    boat.forward(4)
    sleep(0.05)
    boat.position()
    wind.setheading(dir + 180)


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

        # velocities = calc_obstacle_penalties(velocities, angles)
        # vt_max_r, vt_max_l, windangle_max_r, windangle_max_l = find_maximums(velocities)

        return vt_max_r, vt_max_l, windangle_max_r, windangle_max_l

    def run(self, path, w, b_h, b_p):
        """
        This method directs the boat by using the boat's position and the target position to calculate the path
        and comparing the boat's heading to the wind heading to determine the most efficient way for the boat to get
        to its target position.

        :param b_p - the boat's current position as a Python list
        :param b_h - the boat's current heading as an angle in degrees
        :param w - the wind angle in degrees, relative to 0 (with 0 as east)
        :return new_dir - the boat's new direction as a numpy array
        """

        # print("Starting planner")

        # Calculates the path of the boat
        path = np.array(path) - np.array(b_p)
        # print("Path: ", path)

        # The beating parameter (which controls the length of a tack)
        p_c = 20

        # Runs the optimum function and compares the optima returned to determine the best direction for
        # boat to go
        a, b, c, d = self._get_optima(w, path)
        new_dir = _get_best_dir(a, b, c, d, p_c, path, b_h)

        return new_dir
