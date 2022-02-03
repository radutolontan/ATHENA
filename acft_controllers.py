#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov 10 18:11:37 2021

@author: raduefb
"""

# !/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov  7 06:37:23 2021

@author: raduefb
"""
from acft_trajectory import climb_descend


def acft_MPC(aircraft, N_MPC, freq, x0, xD, k_cur):
    # aircraft: instance of acft_class
    # N_MPC: horizon of MPC controller
    # freq: frequency at which silmulation is run
    # x0: initial condition of current optimization problem
    # xL, xU: upper and lower bounds for states
    # uL, uU: upper and lower bounds for inputs
    # xD: desired state

    import numpy as np
    import pyomo.environ as pyo
    from pyomo.opt import SolverStatus, TerminationCondition

    # Initialize optimization problem
    model = pyo.ConcreteModel()
    model.N = N_MPC
    model.nx = np.size(aircraft.A, 0)
    model.nu = np.size(aircraft.B, 1)

    # Length of finite optimization problem
    model.tIDX = pyo.Set(initialize=range(model.N + 1), ordered=True)
    model.xIDX = pyo.Set(initialize=range(model.nx), ordered=True)
    model.uIDX = pyo.Set(initialize=range(model.nu), ordered=True)

    # Create pyomo objects for the linear system description
    model.A = aircraft.A
    model.B = aircraft.B
    
    # Create pyomo objects for cost matrices
    model.Q = aircraft.Q
    model.Qf = aircraft.Qf
    model.R = aircraft.R
    model.dR = aircraft.dR

    # Create state and input variables trajectory
    model.x = pyo.Var(model.xIDX, model.tIDX)
    model.u = pyo.Var(model.uIDX, model.tIDX)

    # Import trajectory preview
    stages = np.linspace(k_cur, k_cur+N_MPC, N_MPC+1)
    ref_x = climb_descend(stages, freq, xD[1], 5.2e4, 673)

    # TO DO: CHANGE REF_X to incorporate info on other states!!!    

    # Flight-phase cost selection
    if (min(ref_x)==max(ref_x)):
        # If steady flight is desired
        stage_cost_angles = 1000
        stage_cost_height = 5
        terminal_cost_height = 0.2
    else:
        # If descent is desired
        stage_cost_angles = 10
        stage_cost_height = 1
        terminal_cost_height = 200

    # Objective
    def objective_rule(model):
        costX = 0.0
        costU = 0.0
        costTerminal = 0.0
        
        # STAGE COST ON STATES
        for t in model.tIDX:
            for i in range(0,6):
                if t < model.N:
                    costX += model.Q[i,i] * (model.x[i, t] - xD[i])**2

        # STAGE COST ON INPUTS
        for t in model.tIDX:
            for i in model.uIDX:
                if t < model.N:
                    costU += model.R[i,i] * (model.u[i, t])**2
        
        # TERMINAL COST ON STATES
        for i in range(0,6):
            # TERMINAL COST FOR ALTITUDE
            costTerminal += model.Qf[i,i] * (model.x[i, model.N] - xD[i])**2

        return costX + costU + costTerminal

    model.cost = pyo.Objective(rule=objective_rule, sense=pyo.minimize)

    # System Constraints
    def equality_const_rule(model, i, t):
        return model.x[i, t + 1] == ( model.x[i, t] + (1/freq) * (sum(model.A[i, j] * model.x[j, t] for j in range(1,6))
             + sum(model.B[i, j] * model.u[j, t] for j in model.uIDX))) if t < model.N else pyo.Constraint.Skip
    #model.equality_constraints1 = pyo.Constraint(model.xIDX, model.tIDX, rule=equality_const_rule)

    # Foreword Euler
    # Implement Foreword Euler on x-VAR
    model.equality_constraints2 = pyo.Constraint(model.tIDX, rule=lambda model, t: model.x[0, t + 1] == model.x[0, t] + (1/freq)*
                                        (model.x[2,t] + 673) if t < model.N else pyo.Constraint.Skip)

    model.equality_constraints3 = pyo.Constraint(model.tIDX, rule=lambda model, t: model.x[1, t + 1] == model.x[1, t] + (1 / freq) *
                                                                       (model.x[3, t]) if t < model.N else pyo.Constraint.Skip)

    model.equality_constraints4 = pyo.Constraint(model.tIDX,rule=lambda model, t: model.x[2, t + 1] == model.x[2, t] + (1/freq) *
                                                                                  (sum(model.A[2, j] * model.x[j, t] for j in model.xIDX)
                                                                                   + sum(model.B[2, j] * model.u[j, t] for j in model.uIDX))
                                                                                    if t < model.N else pyo.Constraint.Skip)

    model.equality_constraints5 = pyo.Constraint(model.tIDX,rule=lambda model, t: model.x[3, t + 1] == model.x[3, t] + (1/freq) *
                                                                                  (sum(model.A[3, j] * model.x[j, t] for j in model.xIDX)
                                                                                   + sum(model.B[3, j] * model.u[j, t] for j in model.uIDX))
                                                                                    if t < model.N else pyo.Constraint.Skip)

    model.equality_constraints6 = pyo.Constraint(model.tIDX,rule=lambda model, t: model.x[4, t + 1] == model.x[4, t] + (1/freq) *
                                                                                  (sum(model.A[4, j] * model.x[j, t] for j in model.xIDX)
                                                                                   + sum(model.B[4, j] * model.u[j, t] for j in model.uIDX))
                                                                                    if t < model.N else pyo.Constraint.Skip)

    model.equality_constraints7 = pyo.Constraint(model.tIDX,rule=lambda model, t: model.x[5, t + 1] == model.x[5, t] + (1/freq) *
                                                                                  (sum(model.A[5, j] * model.x[j, t] for j in model.xIDX)
                                                                                   + sum(model.B[5, j] * model.u[j, t] for j in model.uIDX))
                                                                                    if t < model.N else pyo.Constraint.Skip)






    # Initial Conditions Constraints
    model.initial_constraints = pyo.Constraint(model.xIDX, rule=lambda model, i: model.x[i, 0] == x0[i])



    # State Constraints
    model.state_constraints1 = pyo.Constraint(model.xIDX, model.tIDX, rule=lambda model,i,t: model.x[i,t]<=aircraft.xU[i])
    model.state_constraints2 = pyo.Constraint(model.xIDX, model.tIDX, rule=lambda model,i,t: model.x[i,t]>=aircraft.xL[i])


    # Input Constraints
    model.input_constraints1 = pyo.Constraint(model.uIDX, model.tIDX, rule=lambda model,i,t: model.u[i,t]<=aircraft.uU[i])
    model.input_constraints2 = pyo.Constraint(model.uIDX, model.tIDX, rule=lambda model,i,t: model.u[i,t]>=aircraft.uL[i])
    # Final Constraints


    """
    limit = np.array([200, 200, 200, 10, 10 , 50, 0.1, 0.1, 0.1, 0.5, 0.5 , 0.5])
    model.final_constraints1 = pyo.Constraint(model.xIDX, rule=lambda model, i: model.x[i, model.N-1] <= xD[i] + limit[i])
    model.final_constraints2 = pyo.Constraint(model.xIDX, rule=lambda model, i: model.x[i, model.N-1] >= xD[i] - limit[i])
    """
    # Solve optimization problem and extract results
    solver = pyo.SolverFactory("mosek")
    results = solver.solve(model)

    xOpt = np.asarray([[model.x[i, t]() for i in model.xIDX] for t in model.tIDX]).T
    uOpt = np.asarray([model.u[:, t]() for t in model.tIDX]).T
    # If problem is infeasible, return null

    if (results.solver.status == SolverStatus.ok) and (results.solver.termination_condition == TerminationCondition.optimal):
        feas = True
        xOpt = np.asarray([[model.x[i,t]() for i in model.xIDX] for t in model.tIDX]).T
        uOpt = np.asarray([model.u[:,t]() for t in model.tIDX]).T
    else:
        feas = False
        xOpt = 999
        uOpt = 999
        print(results.solver.termination_condition )

    return [uOpt[:,0],feas]

