import numpy as np
import matplotlib.pyplot as plt

def climb_descend(k, freq, delta_h, delta_x, V_TAS):
    # k - time steps; freq - frequncy (Hz)
    # delta_h = change in height
    # delta_x = change in distance
    slope = delta_h / delta_x    # X - location increases at constant velocity
    x = np.array(k) * (V_TAS) / freq * slope
    for i in range(len(k)):
        if x[i] > delta_h:
            x[i] = delta_h
    return x

