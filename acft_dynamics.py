#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov  5 13:57:39 2021

@author: raduefb
"""

import numpy as np
from scipy.integrate import solve_ivp

from acft_properties import B747


def acft_dynamics(t, x, u):
    # Use noise magnitude to enable system noise
    A, B, _, _, _, _ = B747()
    noise_mag = 0.0
    n = np.random.rand(12,1)
    # Calculate the states
    state = A@x + B@u
    return [state]

def acft_EOM(TS, x_k, u_k):
    # Solve LTI differential equation at the next timestep
    sol = solve_ivp(fun=acft_dynamics, t_span=[0, TS], y0=x_k, method='RK45', 
                    t_eval = [TS], vectorized=True, args=(u_k), rtol=1e-7)
    # Append states at x_k+1 to return vector
    x_k1 = sol.y
    return x_k1


