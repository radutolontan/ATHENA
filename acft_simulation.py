# Import Python functions
import numpy as np
import matplotlib.pyplot as plt

# Import User functions for the quadcopter

from acft_controllers import *
from acft_2Dproperties import *
from acft_plot import multi_plot

plt.close("all")


# Set operating frequencies for the inner and outer loops
freq_outer = 30 # (Hz)

# Set number of time steps
N = 4000
# Set MPC horizon
N_MPC = 10

# Allocate storage for states over time
x = np.empty((6,N+1))
u = np.zeros((1, N))
# Set initial conditions on states
x[:,0] = np.array([0, 0, -10.58, 0, 0, 0]).T  #-16

# Set desired final location
xD = np.array([0, 100, 0, 0, 0, 0]).T #100

# Read aircraft properties
A, B, uL, uU, xL, xU = B747_2DLONG()
feas_list = []

# Run MPC controller
for t in range(N):
    u_CFTOC, feas = acft_MPC(A, B, N_MPC, freq_outer, x[:,t], xL, xU, uL, uU, xD, t)
    x[:,t+1] = x_CFTOC[:,1]
    u[:,t] = u_CFTOC
    print("MPC problem ",t,"OK")
    feas_list.append(feas)

# If all problems are feasile
if (all(feas_list)):
    print("Project ATHENA succesfull!")
    
    # Compute reference trajectory for plotting 
    stages = np.linspace(0, N, N+1)
    profile = climb_descend(stages, freq_outer, 100, 5.2e4, 673)

    # Plot
    void_return = multi_plot(x, u, N, profile)