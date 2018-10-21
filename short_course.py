import numpy as np
def get_new_dir (B, T, wind, p_c, boat_velocity):
    '''

    :param B: vector representing the boat's position
    :param T: vector representing the target position
    :param wind: vector representing the wind
    :param p_c: used to calculate the hysteresis factor; band's length
    :param boat_velocity: vector representing the boat's velocity
    :return:
    '''

    path = [T[0] - B[0], T[1] - B[1]]

    return path

B = [1, 1]
T = [2, 2]

path = get_new_dir(B, T, 0, 0, [0, 0])
print(path)
