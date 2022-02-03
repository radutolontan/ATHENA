# ATHENA
Project ATHENA (Aircraft Thrust Enabled-Navigation) is a Propulsion Controlled Aircraft (PCA) approach which uses optimization based control. The control architrecture uses a Model Predictive Controller (MPC) to control aircraft attitude using only thrust settings. The current implementation works for longitudinal motion and simulates a Boeing 747-200 aircraft.

To run the full Python simulation, run acft_simulation.py
The MPC controller is implemented in acft_controller.py
Aircraft control derivatives and properties are available in acft_2Dproperties.py
Linear dynamics used in simulation are defined in acft_dynamics.py

For testing using XPlane 11, first follow instructions at https://github.com/nasa/XPlaneConnect to install XPlane Connect.
Run ATHENA_XPlane11.py to test controller using non-linear dynamics in XPlane. Performance still requires tuning.
