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
def optimum(windangle,path):
    '''
        :param wind: angle representing the wind
        :param path: vector representing the path taken to reach from boat's position to the target position
        :return:
            windangle_max_R - max boat heading on right side
            windangle_max_L - max boat heading on left side
            vt_max_R - max velocity on right side
            vt_max_L - max velocity on left side
        '''

    # angle of the wind; also inversed so that it's easier for calculations
    #windangle = np.arctan2(-wind[1],-wind[0])

    # wind speed -- calculating the magnitude of the wind vector
    #windspeed = np.sqrt((wind[0]**2)+(wind[1]**2))

    # Initializing the right side and left side optimum wind angles
    windangle_max_R = windangle
    windangle_max_L = windangle

    # Initializing the right side and left side optimum max velocities
    vt_max_R = 0
    vt_max_L = 0

    # Initialize the counter
    alpha = 0
    for alpha in range(180): # This finds the optimum for the right side -- 0 --> 180 deg = right side

        # Test polar function
        vb_hyp = polar_effeciency(windangle, alpha)
        # Projecting the max speed in the direction of the path
        vt_test = np.dot(vb_hyp,path)

        # Check if this velocity is the new max
        if(vt_test>vt_max_R):
            vt_max_R = vt_test
            windangle_max_R = (windangle + alpha) % 360

    # Analogous to the right side
    for alpha in range (-180,0): # This finds the optimum for the left side
        vb_hyp = polar_effeciency(windangle, alpha)
        vt_test = np.dot(vb_hyp, path)
        if (vt_test>vt_max_L):
            vt_max_L = vt_test
            windangle_max_L = (windangle + alpha) % 360

    return vt_max_R,vt_max_L,windangle_max_R,windangle_max_L

def polar_effeciency(windangle, alpha):
    # Convert angle to radians
    angle = windangle + alpha
    alpha *= np.pi/180
    angle *= np.pi/180

    # Calculate magnitude using the polar effeciency function
    mag = (1-np.cos(alpha)) * (1 + .3*np.cos(alpha))/(1-0.5*np.cos(alpha))

    # Return vector components
    return [mag*np.cos(angle), mag*np.sin(angle)]

# checks whether right or left is better and returns the new direction (using the hysteresis)
def get_new_dir(vt_max_R, vt_max_L, windangle_max_R, windangle_max_L, p_c, path, boat_heading):
    # Calculates the hysteresis factor
    n = 1 + (p_c / (abs(np.sqrt(path[0] ** 2 + path[1] ** 2))))
    # print(n)

    # Determines the new boat heading by checking the hysteresis factor
    # new_boat_heading = -1

    # Choose the direction that is closest to the current boat heading
    # R_diff = abs(windangle_max_R - boat_heading)
    # L_diff = abs(windangle_max_L - boat_heading)
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

def run(Bp, Bh, T, w):
    path = get_path(Bp, T)
    p_c = 20
    # this runs above function to get the optimums and stores it into an array.
    a,b,c,d = optimum(w, path)
    new_dir = get_new_dir(a,b,c,d, p_c, path, Bh)

    return new_dir
