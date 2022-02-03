import time as tm
import xpc
import math
import numpy as np

# Import user-defined functions
from acft_controllers import acft_MPC
from acft_2Dproperties import B747_2DLONG, acft_class

# Declare addresses of required states and I/O
theta_add = ["sim/flightmodel/position/theta"]
pitch_rate_add = ["sim/flightmodel/position/Q"]
vz_add = ["sim/flightmodel/position/local_vy"]
vy_add = ["sim/flightmodel/position/local_vz"]
vx_add = ["sim/flightmodel/position/local_vx"]
alt_add = ["sim/cockpit2/gauges/indicators/altitude_ft_pilot"]
x_add = ["sim/flightmodel/position/local_z"]
hyd_fail = []
ap_ONOFF = []
        
def initialize_ATHENA():
    print("Project ATHENA & XPlane 11")
    print("Connecting to XPlane...")
    with xpc.XPlaneConnect() as client:
        # Verify connection
        try:
            # If X-Plane does not respond to the request, a timeout error
            # will be raised.
            client.getDREF("sim/test/test_float")
        except:
            print("Error establishing connection to X-Plane.")
            print("Exiting...")
            return
    
        print("Connection succesfull!")
        
        # Wait for system to be initialized
        client.sendTEXT("ATHENA waiting for activation...", 100, 120)
        
        # Initialize system when "1" is input
        print("Press 1 followed by ENTER to activate ATHENA...")
        activation_cmd = 0
        while (activation_cmd != 1):
            activation_cmd = int(input())
            tm.sleep(1)  
        print("ATHENA active. Disabling AP, ATHR, failing Hydraulics...")
        
        # Disable autopilot and hydraulics
        long_restriction = [-998, -998, -998, -998, 0, -998, -998]
        client.sendPOSI(long_restriction, 0)
                
        client.sendTEXT("ATHENA ACTIVATED", 100, 120)


def read_actf_states(adjusted,printer):
     with xpc.XPlaneConnect() as client:
         bulb = True
         while (bulb):
             x_XP = client.getDREFs(x_add)[0][0] # X (???)
             z_XP = client.getDREFs(alt_add)[0][0] # Z (ft)
             vx_WORLD_XP = client.getDREFs(vx_add)[0][0]
             vy_WORLD_XP = client.getDREFs(vy_add)[0][0]
             vx_XP = np.sqrt(vx_WORLD_XP**2+vy_WORLD_XP**2) # vx (m/s.)
             vz_XP = client.getDREFs(vz_add)[0][0] # vz (m/s.)
             theta_XP = client.getDREFs(theta_add)[0][0] # Theta (deg.)
             q_XP = client.getDREFs(pitch_rate_add)[0][0] # q (deg./s.)
         
             # If adjusted, report states relative to equilibrium
             if (adjusted==True):
                # ----------------(???)-------(ft)--------(ft/s)-------(ft/s)
                states = np.array([x_XP, -(z_XP-20000), vx_XP/0.31 - 673, -vz_XP/0.31,
                                   theta_XP * (np.pi/180), q_XP * (np.pi/180)])
                # -------------------------(rad)----------------(rad/s)
             else:
                states = np.array([x_XP, z_XP, vx_XP, vz_XP, theta_XP, q_XP])
          
             # If printer is on, print states
             if (printer==True):
                print (" x: ", states[0], "z: ", states[1]," vx: ", states[2], 
                       " vz: ", states[3], " theta: ", states[4]," q: ", states[5])
                tm.sleep(2)
            
             # Assign printer value to bulb and exit if necesaary
             bulb = printer
            
         return states

def MPC_ATHENA(alt_des):
    with xpc.XPlaneConnect() as client:
        # Set operating frequencies for the inner and outer loops
        freq = 4 # (Hz)

        # Set number of time steps
        N = 100
        # Set MPC horizon
        N_MPC = 10
    
        # Allocate storage for states over time
        x = np.empty((6,N+1))
    
        # Read and assign initial conditions on states
        x[:,0] = read_actf_states(True, False)
    
        # Set desired final location
        xD = np.array([0, -alt_des, 0, 0, 0, 0]).T 
    
        # Read aircraft properties
        A, B, Q, Qf, R, dR, uL, uU, xL, xU = B747_2DLONG()
        aircraft = acft_class(A, B, Q, Qf, R, dR, uL, uU, xL, xU)
    
        start_time = tm.time()
    
        # Run MPC controller
        for t in range(N):
            try: 
                T_req, feas = acft_MPC(A, B, N_MPC, freq, x[:,t], xL, xU, uL, uU, xD, t)
        
                # Check if CFTOC is not feasible
                if (feas!=True):
                    print("INFEASIBLE! ATHENA exiting...")
                    client.sendTEXT("ATHENA FAULT...", 100, 100)
                    client.close()          
                    break
            
                # Adjust required thrust input
                T_XP = (T_req[0] + 42000)/60000

                # Display status in XPlane
                actual_FL = math.floor(read_actf_states(False, False)[1])
                XP_show = "ATHENA OK - HYD FAILED - Altitude:  {alt} - Thrust Command: {thrcmd}".format(alt = actual_FL, thrcmd = T_XP)
                client.sendTEXT(XP_show, 100, 100)

            
                # Send control input to XPlane
                XP_control_input = [-998, -998, -998, T_XP]
                client.sendCTRL(XP_control_input)
                print("MPC prob. ", t, " OK! Thrust desired: ", T_XP)
            
                # Read updated aircraft state
                x[:,t+1] = read_actf_states(True, False)
            
            # If solver raises error messages or CTRL + C is pressed
            except:
                client.sendTEXT("ATHENA FAULT...", 100, 100)
                client.close()    
        
        end_time = tm.time()
        print("Elapsed time: ", end_time-start_time)
                
        
if __name__ == "__main__":
    # INITIALIZE ATHENA
    initialize_ATHENA()
    
    # RUN ATHENA + XPLANE
    MPC_ATHENA(100)

