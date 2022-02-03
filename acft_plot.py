#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec 10 10:25:23 2021

@author: raduefb
"""
import matplotlib.pyplot as plt
import numpy as np

def multi_plot(x, u, N, profile):
    
    plt.figure(1)
    plt.subplot(3,3,1)
    plt.plot(x[0,:]*0.0003048, 20000-x[1,:],'magenta')
    plt.xlabel("X")
    plt.ylabel("Y")

    plt.subplot(3,3,2)
    plt.plot(x[4,:]*(180/3.1415), 'green')
    plt.xlabel("Timestep")
    plt.ylabel("Theta")

    plt.subplot(3,3,3)
    plt.plot(u[0,:], 'orange')
    plt.xlabel("Timestep")
    plt.ylabel("Thrust (lbf)")

    plt.subplot(3,3,4)
    plt.plot(x[2,:], 'red')
    plt.xlabel("Timestep")
    plt.ylabel("X-vel")

    plt.subplot(3,3,5)
    plt.plot(-x[3,:], 'blue')
    plt.xlabel("Timestep")
    plt.ylabel("Z-vel")

    plt.subplot(3,3,6)
    plt.plot(x[5,:], 'yellow')
    plt.xlabel("Timestep")
    plt.ylabel("Pitch Rate")
    
    plt.figure(2,figsize=(3.25,3.25))
    plt.plot(x[0,:]*0.0003048, 20000-x[1,:])
    plt.plot(x[0,:]*0.0003048, 20000-profile)
    plt.xlabel('Distance Traveled (km)', fontsize=16)
    plt.ylabel('Altitude (ft)', fontsize=16)
    for i in range(0,N,900):
        # Bottom of plane
        plt.plot((x[0,i]*0.0003048-2*np.cos(10*x[4,i]), x[0,i]*0.0003048+2*np.cos(10*x[4,i])),
                 (20000-x[1,i]-2*np.sin(10*x[4,i]) , 20000-x[1,i]+2*np.sin(10*x[4,i])),'red', linewidth=3)
        # Engines/Wings
        plt.plot((x[0,i]*0.0003048-0.7*np.cos(10*x[4,i]), x[0,i]*0.0003048+0.7*np.cos(10*x[4,i])),
                 (20000-x[1,i]-0.7*np.sin(10*x[4,i]) , 20000-x[1,i]+0.7*np.sin(10*x[4,i])),'red', linewidth=5)
        # Vertical Stabilizer
        plt.plot((x[0,i]*0.0003048-2*np.cos(10*x[4,i]), x[0,i]*0.0003048-2*np.cos(10*x[4,i]) - 0.15*np.cos(np.pi/2 - 10*x[4,i])),
                 (20000-x[1,i]-2*np.sin(10*x[4,i]) , 20000-x[1,i]-2*np.sin(10*x[4,i]) + 2*np.sin(np.pi/2 - 10*x[4,i])),'red', linewidth=3)
    plt.legend(["Actual Flight Path","Desired Flight Path","Aircraft"])

    
    plt.figure(3,figsize=(3.25,3.25))
    plt.plot(40000 - u[0,:], 'green')
    plt.xlabel('Timestep', fontsize=16)
    plt.ylabel('Engine Thrust (lbf)', fontsize=16)
    
    plt.figure(4, figsize=(3.25,3.25))
    plt.plot(x[4,:]*(180/3.1415), 'green')   
    plt.xlabel('Timestep', fontsize=16)
    plt.ylabel('Theta (deg)', fontsize=16)
    return 0