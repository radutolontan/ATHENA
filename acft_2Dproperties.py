# -*- coding:  utf-8 -*-
"""
Author: Sigurd Janitz Kvaal
Date created: 11/17/2021
Email: sigurd.kvaal@nmbu.no
Purpose:


"""
import numpy as np

class acft_class:
    def __init__(self, A, B, Q, Qf, R, dR, uL, uU, xL, xU):
        self.A = A # xdot = Ax + Bu
        self.B = B # xdot = Ax + Bu
        self.Q = Q # see J_k below
        self.Qf = Qf # see Jf below
        self.R = R # see Jinput below
        self.dR = dR # see Jdinput below        
        self.uL = uL # uL < u < uU
        self.uU = uU
        self.xL = xL # xL < x < xU
        self.xU = xU
        self.g = 9.81 # (m/s^2)


def B747_2DLONG():
    """
    Dynamics for lateral thrust control(2D)
    of B747
    :return:
    """

    # Flight control derivatives and aircraft parameters
    X_u = -0.01187
    X_w = 0.023739

    Z_u = -0.10856
    Z_w = -0.51652
    Z_q = 651.33

    M_u = 4.22*10**(-5)
    M_w = -0.00185
    M_theta = 0.000174
    M_q = -0.64389

    g_sin_theta = 32.18
    g_cos_theta = 1.1052

    m = 636636 # In matrix it is 5.06 *10**5 = g/m

    ze1_Iyy = 3.02 * 10 **(-8)
    ze2_Iyy = 3.02 * 10 **(-8)
    ze3_Iyy = 3.02 * 10 **(-8)
    ze4_Iyy = 3.02 * 10 **(-8)

    # ===============================================
    # LINEAR SYSTEM DESCRIPTION i.e (d/dt)X = AX + BU
    # ===============================================

    A = np.array([[0,0,1,  0,  0,           0],  #x
                  [0,0,0,  1,  0,           0],  #z
                  [0,0,X_u,X_w,-g_cos_theta,0],  #u
                  [0,0,Z_u,Z_w,-g_sin_theta,Z_q],#w
                  [0,0,0,  0,  0,           1],  #theta
                  [0,0,M_u,M_w,M_theta, M_q]     #q
                  ])
    
    B = np.array([[0],
                  [0],
                  [4/ m],
                  [0],
                  [0],
                  [4*ze1_Iyy]])
    
    # ===============================================
    # COST MATRICES 
    # ===============================================

    # J_k = (x_k - x_k_ref).T * Q * (x_k - x_k_ref) (stage or running cost matrix on states)
    Q = np.diag(np.array([0,1,0,0,10,10]))
    
    # Jf = (x_N - x_N_ref).T * Qf * (x_N - x_N_ref) (terminal cost on states)
    Qf =np.diag(np.array([0,0.2,0,0,0,0]))
    
    # Jinput = (u_k).T * R * (u_k) (stage cost on inputs)
    R = np.diag(np.array([0]))
    
    # Jdinput = (u_k+1 - u_k).T * dR * (u_k+1 - u_k) (stage cost on input change)
    dR = 0.0 * np.diag(np.array([5]))

    # ===============================================
    # STATE & INPUT CONSTRAINTS 
    # ===============================================

    #u =[T1, T2, T3, T4] Thrust: lbf
    uL = np.array([-42000])
    uU = np.array([18000])

    #x= [u, v, w, phi, theta, psi, p, q, r]
    xL = np.array([-10**9, -10**5, -373, -10**5, -0.175, -10**5])
    xU = np.array([10**9,  10**5,  165, 10**5,  0.3,  10**5])

    return A, B, Q, Qf, R, dR, uL, uU, xL, xU




