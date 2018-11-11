'''
This program implements the short course routing algorithm described in Section 5.1 of Roland Stelzer's Autonomous
Sailboat Navigation paper. This algorithm uses a polar efficiency function to determine the efficiency of all possible
angles that a boat can turn, and then compares the efficiencies to determine the optimal boat heading.

Read the paper here: https://www.dora.dmu.ac.uk/bitstream/handle/2086/7364/thesis-optimized-300dpi.pdf
'''
import numpy as np
from turtle import *
from time import sleep

class ShortCoursePlanner:

    def polar_effeciency(self, windangle, alpha):
        '''
        Calculates the speed for a certain angle
        :param windangle: the wind direction
        :param alpha: angle being tested (relative to the wind angle)
        :return: the vector components for the velocity for the angle
        '''
        # Convert angle to radians
        angle = (windangle + alpha) * 180
        alpha *= np.pi / 180

        # Calculate magnitude using the polar effeciency function
        mag = (1 - np.cos(alpha)) * (1 + 0.3 * np.cos(alpha)) / (1 - 0.5 * np.cos(alpha))

        # Return vector components
        return [mag * np.cos(angle), mag * np.sin(angle)]

    # finds the maximum velocity and angle that can be achieved for each value of alpha for the right side and the left side
    def optimum(self, windangle, path):
        '''
            :param wind: angle representing the wind
            :param path: vector representing the path taken to reach from boat's position to the target position
            :return:
            A tuple containing the following information:
            windangle_max_R - max boat heading on right side
            windangle_max_L - max boat heading on left side
            vt_max_R - max velocity on right side
            vt_max_L - max velocity on left side
            '''

        # Initializing the right side and left side optimum wind angles
        windangle_max_R = windangle
        windangle_max_L = windangle

        # Initializing the right side and left side optimum max velocities
        vt_max_R = 0
        vt_max_L = 0

        # Initializes alpha, or the angle being tested (relative to the wind direction)
        alpha = 0

        # Checks all alphas between 0 and 180 and calculates the theoretical speeds
        # to find an optimal heading on the right hand side
        for alpha in range(180):

            # Uses the polar effeciency function to calculcate the hypothetical velocity of the boat at that angle
            vb_hyp = self.polar_effeciency(windangle, alpha)

            # Projecting the max speed in the direction of the path
            vt_test = np.dot(vb_hyp,path)

            # Check if this velocity is the new max
            if(vt_test>vt_max_R):
                vt_max_R = vt_test
                windangle_max_R = (windangle + alpha) % 360

        # Analogous to the right side, but for the left side
        for alpha in range (-180,0):
            vb_hyp = self.polar_effeciency(windangle, alpha)
            vt_test = np.dot(vb_hyp, path)
            if (vt_test>vt_max_L):
                vt_max_L = vt_test
                windangle_max_L = (windangle + alpha) % 360

        return vt_max_R,vt_max_L,windangle_max_R,windangle_max_L



    def get_new_dir(self, vt_max_R, vt_max_L, windangle_max_R, windangle_max_L, p_c, path, boat_heading):
        '''
        checks whether right or left is better and returns the new direction (using the hysteresis)
        :param vt_max_R: the maximum velocity on the right hand side
        :param vt_max_L: the maximum velocity on the left hand side
        :param windangle_max_R: angle corresponding to the max velocity on the right hand side
        :param windangle_max_L: angle corresponding to the max velocity on the left hand side
        :param p_c: beating parameter (controls the length of the tacks)
        :param path: the path the boat is following
        :param boat_heading: the boat's heading
        :return: the new boat direction and speed
        '''
        # Calculates the hysteresis factor
        n = 1 + (p_c / (abs(np.sqrt(path[0] ** 2 + path[1] ** 2))))

        # Choose the direction that is closest to the current boat heading
        R_diff = 180 - abs(abs(windangle_max_R - boat_heading) - 180);
        L_diff = 180 - abs(abs(windangle_max_L - boat_heading) - 180);
        if R_diff < L_diff:
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

    def run(self, Bp, Bh, T, w):
        path = np.array(T) - np.array(Bp)
        p_c = 20
        # this runs above function to get the optimums and stores it into an array.
        a,b,c,d = self.optimum(w, path)
        new_dir = self.get_new_dir(a,b,c,d, p_c, path, Bh)

        return new_dir

if __name__ == '__main__':
    windDir = int(input("wind angle: "))  # 90  #wind source direction
    boatCurrDir = 0
    targetList = [(0, 120), (120, 120), (0, 0), (120, 0), (120, -120), (0, -120), (0, 0)]
    wind = Turtle()
    wind.pu()
    wind.setpos(250 * np.cos(windDir * np.pi / 180), 250 * np.sin(windDir * np.pi / 180))
    boat = Turtle()
    boat.setheading(boatCurrDir)

    for targetPos in targetList:
        while boat.distance(targetPos) > 5:
            # windVector = [np.cos(windDir*np.pi/180), np.sin(windDir*np.pi/180)]
            boatNewDir = ShortCoursePlanner.run(boat.position(), boat.heading(), targetPos, windDir)
            # boatCurrDir = adjustTo(boatCurrDir, boatNewDir)

            boat.setheading(boatNewDir)
            boat.forward(4)
            sleep(0.05)
            boat.position()

            # windDir += randint(-4, 4)
            wind.setheading(windDir + 180)
