"""
This program implements the short course routing algorithm described in Section 5.1 of Roland Stelzer's Autonomous
Sailboat Navigation paper. This algorithm uses a polar efficiency function to determine the efficiency of all possible
angles that a boat can turn, and then compares the efficiencies to determine the optimal boat heading.

Read the paper here: https://www.dora.dmu.ac.uk/bitstream/handle/2086/7364/thesis-optimized-300dpi.pdf
"""
import numpy as np

class ShortCoursePlanner:

    @staticmethod
    def _get_polar_efficiency(wind_angle, alpha):
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

    def initialize_velocities(self):
        velocities = []
        for i in range(360):
            velocities.append(-1)
        return velocities

    def _get_optima(self, wind_angle, path, velocities, obstacles, r_min, r_max):
        """
        Finds the maximum velocity and angle that can be achieved for every possible new heading for the boat to
        determine the most optimal headings on the port and starboard sides of the boat.

        :param wind: angle representing the wind.
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

            velocities[alpha] = vt_test

            # Check if this velocity is the new max
            if vt_test > vt_max_r:
                vt_max_r = vt_test
                windangle_max_r = (wind_angle + alpha) % 360

        # Analogous to the starboard tack, but for a port tack
        for alpha in range(-180, 0):
            vb_hyp = self._get_polar_efficiency(wind_angle, alpha)
            vt_test = np.dot(vb_hyp, path)
            velocities[alpha + 360] = vt_test
            if vt_test > vt_max_l:
                vt_max_l = vt_test
                windangle_max_l = (wind_angle + alpha) % 360

        orig_vt_max_r, orig_vt_max_l, orig_windangle_max_r, orig_windangle_max_l = self.find_maximums(vt_max_r,
                                                                            vt_max_l, windangle_max_r,
                                                                                windangle_max_l, velocities)
        print("Original best on right: ", orig_vt_max_r, orig_windangle_max_r)
        print("Original best on left: ", orig_vt_max_l, orig_windangle_max_l)

        velocities = self.calc_obstacle_penalties(velocities, obstacles, r_min, r_max)
        vt_max_r, vt_max_l, windangle_max_r, windangle_max_l = self.find_maximums(vt_max_r, vt_max_l, windangle_max_r,
                                                                             windangle_max_l, velocities)
        print("New best on right: ", vt_max_r, windangle_max_r)
        print("New best on left: ", vt_max_l, windangle_max_l)

        # print("vt_max_r: ", vt_max_r)
        # print("vt_max_l: ", vt_max_l)
        # print("wind angle max r: ", windangle_max_r)
        # print("wind angle max l: ", windangle_max_l)

        return vt_max_r, vt_max_l, windangle_max_r, windangle_max_l

    def calc_obstacle_penalties(self, velocities, obstacles, r_min, r_max):
        for obstacle in obstacles:
            angle = obstacle[1]
            # print("Original velocity: ", velocities[angle])
            qb = min(1, max(0, (obstacle[0] - r_min) / (r_max - r_min)))
            # print("Angle: ", angle)
            velocities[angle] = velocities[angle] * qb
            # print("Velocity: ", velocities[angle])
            velocities[angle + 1] = velocities[angle + 1]*qb/2
            velocities[angle - 1] = velocities[angle - 1]*qb/2
            # print("Updated velocity: ", velocities[angle])
        return velocities

    def find_maximums(self, vt_max_R, vt_max_L, windangle_max_R, windangle_max_L, velocities):
        for angle in range(len(velocities)):
            velocity = velocities[angle]

            if angle < 180:
                if velocity > vt_max_R:
                    vt_max_R = velocity
                    windangle_max_R = angle
            else:
                if velocity > vt_max_L:
                    vt_max_L = velocity
                    windangle_max_L = angle - 360
        return vt_max_R, vt_max_L, windangle_max_R, windangle_max_L

    def _get_best_dir(self, vt_max_R, vt_max_L, windangle_max_R, windangle_max_L, p_c, path, boat_heading):
        """
        Checks whether a port tack or starboard tack would be more efficient.

        :param vt_max_R: the maximum velocity on the right hand side
        :param vt_max_L: the maximum velocity on the left hand side
        :param windangle_max_R: angle (in degrees) corresponding to the max velocity on the right hand side
        :param windangle_max_L: angle (in degrees) corresponding to the max velocity on the left hand side
        :param p_c: beating parameter (controls the length of the tacks)
        :param path: the path the boat is following as (a numpy array)
        :param boat_heading: the boat's heading, relative to 0 (with 0 as east)
        :return: the new boat direction and speed
        """

        # print("Determining best direction based on hysteresis")

        # Calculates the hysteresis factor
        n = 1 + (p_c / (abs(np.sqrt(path[0] ** 2 + path[1] ** 2))))

        # Choose the direction that is closest to the current boat heading
        r_diff = 180 - abs(abs(windangle_max_R - boat_heading) - 180)
        l_diff = 180 - abs(abs(windangle_max_L - boat_heading) - 180)
        if r_diff < l_diff:
            # Choose the faster velocity
            if vt_max_R * n < vt_max_L:
                new_boat_heading = windangle_max_L
            else:
                new_boat_heading = windangle_max_R
        else:
            if vt_max_L * n < vt_max_R:
                new_boat_heading = windangle_max_R
            else:
                new_boat_heading = windangle_max_L

        return new_boat_heading

    def run(self, b_p, b_h, t, w, velocities, obstacles, r_min, r_max):
        """
        This method directs the boat by using the boat's position and the target position to calculate the path
        and comparing the boat's heading to the wind heading to determine the most efficient way for the boat to get
        to its target position.

        :param b_p - the boat's current position as a Python list
        :param b_h - the boat's current heading as an angle in degrees
        :param t - the target position as a Python list
        :param w - the wind angle in degrees, relative to 0 (with 0 as east)
        :return new_dir - the boat's new direction as a numpy array
        """

        # print("Starting planner")

        # Calculates the path of the boat
        path = np.array(t) - np.array(b_p)
        # print("Path: ", path)

        # The beating parameter (which controls the length of a tack)
        p_c = 20

        # Runs the optimum function and compares the optima returned to determine the best direction for
        # boat to go
        a, b, c, d = self._get_optima(w, path, velocities, obstacles, r_min, r_max)
        new_dir = self._get_best_dir(a, b, c, d, p_c, path, b_h)

        return new_dir


'''
Runs the sample course provided below and displays the resulting path using Turtle.
Also uses the sleep function to simulate the passage of time (to determine how 
fast/efficient the path actually is).
'''
if __name__ == '__main__':

    from turtle import *
    from time import sleep

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
    obstacles = []
    rad = 10
    while (rad < 60):
        angle = 90
        while (angle < 100):
            obstacles.append((rad, angle))
            angle += 2
        rad += 5

    print(obstacles)
    # obstacles = [(10, 88), (10, 86), (10, 84), (10, 82), (10, 80),
    #             (50, 90), (50, 92), (50, 94), (50, 96), (50, 98),
    #              (40, 90), (40, 92), (40, 94), (40, 96), (40, 98),
    #             (30, 90), (30, 92), (30, 94), (30, 96), (30, 98),]
    obstacle_drawer = Turtle()
    obstacle_drawer.pencolor("red")
    obstacle_drawer.fillcolor("red")

    index = 0
    while index < len(obstacles):
        obstacle = obstacles[index]
        # print("angle: ", obstacle[1])
        angle = np.radians(obstacle[1])
        rad = obstacle[0]

        obstacle_drawer.pu()
        obstacle_drawer.setpos(rad*np.cos(angle), rad*np.sin(angle))

        obstacle_drawer.pd()
        obstacle_drawer.begin_fill()
        obstacle_drawer.circle(1)
        obstacle_drawer.end_fill()

        index += 1

    for targetPos in targetList:
        print("Current Target: ", targetPos)
        while boat.distance(targetPos) > 5:
            planner = ShortCoursePlanner()

            velocities = planner.initialize_velocities()
            # print("Initialized Velocities: ", velocities)

            boatNewDir = planner.run(boat.position(), boat.heading(), targetPos, windDir, velocities, obstacles, r_min, r_max)

            boat.setheading(boatNewDir)
            boat.forward(4)
            sleep(0.05)
            boat.position()

            wind.setheading(windDir + 180)
