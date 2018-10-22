import numpy as np
def get_path (B, T):
    '''
    :param B: vector representing the boat's position
    :param T: vector representing the target position
    :return: path - vector representing path from B to T
    '''

    path = [T[0] - B[0], T[1] - B[1]]

    return path

# finds the maximum velocity and angle that can be achieved for each value of alpha for the right side and the left side
def optimum(wind,path):
    '''
        :param wind: vector representing the wind
        :param path: vector representing the path taken to reach from boat's position to the target position
        windangle_max_R: max wind angle right side
        windangle_max_L: max wind angle left side
        vt_max_R: max wind speed right side
        vt_max_L: max wind speed left side
        alpha: turn angle
        :return:
            windangle_max_R - max boat heading on right side
            windangle_max_L - max boat heading on left side
            vt_max_R - max velocity on right side
            vt_max_L - max velocity on left side
        '''

    # angle of the wind; also inversed so that it's easier for calculations
    windangle = np.arctan2(-wind[1],-wind[0])

    # wind speed -- calculating the magnitude of the wind vector
    windspeed = np.sqrt((wind[0]**2)+(wind[1]**2))

    # Initializing the right side and left side optimum wind angles
    windangle_max_R = windangle
    windangle_max_L = windangle

    # Initializing the right side and left side optimum max velocities
    vt_max_R = 0
    vt_max_L = 0

    # Initialize the counger
    alpha = 0
    for alpha in range(180): # This finds the optimum for the right side -- 0 --> 180 deg = right side

        # Test polar function
        vb_hyp = function(windspeed,windangle + alpha)

        # Projecting the max speed in the direction of the path
        vt_test = np.dot(vb_hyp,path)

        # Check if this velocity is the new max
        if(vt_test>vt_max_R):
            vt_max_R = vt_test
            windangle_max_R = windangle + alpha

    # Analogous to the right side
    for alpha in range (180,360): # This finds the optimum for the left side
        vb_hyp = function(windspeed, windangle + alpha)
        vt_test = np.dot(vb_hyp, path)
        if (vt_test>vt_max_L):
            vt_max_L = vt_test
            windangle_max_L = windangle + alpha

    return vt_max_R,vt_max_L,windangle_max_R,windangle_max_L

def test_get_path():
    # testing get_path
    B = [1, 1]
    T = [2, 2]
    path = get_path(B, T, 0, 0, [0, 0])
    print(path)

# this runs above function to get the optimums and stores it into an array.
maximums = optimum(wind, path)

#checks whether right or left is better and returns the new direction (using the hysteresis)
def get_new_dir(vt_max_R, vt_max_L, windangle_max_R, windangle_max_L):


new_dir = get_new_dir(*results)
