'''
QBiT Dynamic Model Simulation v1
Spencer Folk 2020

The purpose of this script is to handle all the interface between the model, trajectory, and simulate commands. 
'''

import matplotlib.pyplot as plt
import numpy as np
from math import *

from qbit_model import quadrotor_biplane_tailsitter, planar_quad
from qbit_simulate import simulate

# Initialize a qbit model with initial conditions
initial_states = {'x': 0,
					'z': 0,
					'phi': 0,
					'xdot': 1,
					'zdot': 0,
					'phidot': 0}
qbit = quadrotor_biplane_tailsitter(initial_states)
# qbit = planar_quad(initial_states)

tf = 10  # Duration of simulation
dt = 1/100  # Sampling period (s) based on control loop period.
tsim = np.arange(0,tf,dt)

# Simulate
sim_results = np.zeros((15,len(tsim)))
# sim_results = np.zeros((6,len(tsim)))

# simulation_results = simulate(qbit, tf, dt)
for i in range(0,len(tsim)):

	T_top = 0.1
	T_bot = 0.1
	delta = 0
	qbit.setCmds(T_top, T_bot, delta)
	qbit.updateStates(dt)

	# T_left = qbit.m*qbit.goutput['xdot'], output['zdot'], output['phidot']*(0.5 + np.sin(2*tsim[i]))
	# T_right = qbit.m*qbit.g*(0.5 + np.sin(2*tsim[i]))
	# qbit.setCmds(T_left, T_right)
	# qbit.updateStates(dt)

	output = qbit.publishValues()
	outx,outz,outphi = output['x'], output['z'], output['phi']
	outxdot, outzdot, outphidot = output['xdot'], output['zdot'], output['phidot']
	outVa, outVi, outVw = output['Va'], output['Vi'], output['Vw']
	outL, outD, outM_air = output['L'], output['D'], output['M_air']
	outalpha, outalpha_e, outgamma = output['alpha'], output['alpha_e'], output['gamma']

	concatenated_output = np.array([outx, outz, outphi, outxdot, outzdot, outphidot, outVa, outVw, outVi, outalpha, outalpha_e, outgamma, outL, outD, outM_air])
	sim_results[:,i] = concatenated_output


## Plotting
# State
(fig, axes) = plt.subplots(nrows=3, ncols=1, sharex=True, num='State vs Time')

ax = axes[0]
ax.plot(tsim, sim_results[0,:], 'r-')
# ax.legend(('x', 'y', 'z'))
ax.set_ylabel('Position [m]')
ax.grid('major')
ax.set_title('x')
ax.set_ylim(0,10)

ax = axes[1]
ax.plot(tsim, sim_results[1,:], 'b-')
# ax.legend(('x', 'y', 'z'))
ax.set_ylabel('Position [m]')
ax.grid('major')
ax.set_title('z')
ax.set_ylim(0,10)

ax = axes[2]
ax.plot(tsim, sim_results[2,:], 'k-')
ax.set_ylabel('Orientation [rad]')
ax.grid('major')
ax.set_title('phi')
ax.set_xlabel('Time [s]')
ax.set_ylim(-pi/2-0.1,pi/2+0.1)


# State derivatives
(fig, axes) = plt.subplots(nrows=3, ncols=1, sharex=True, num='State Derivatives vs Time')

ax = axes[0]
ax.plot(tsim, sim_results[3,:], 'r-')
# ax.legend(('x', 'y', 'z'))
ax.set_ylabel('Linear Velocity [m/s]')
ax.grid('major')
ax.set_title('xdot')
ax.set_ylim(0,10)

ax = axes[1]
ax.plot(tsim, sim_results[4,:], 'b-')
# ax.legend(('x', 'y', 'z'))
ax.set_ylabel('Linear Velocity [m/s]')
ax.grid('major')
ax.set_title('zdot')
ax.set_ylim(0,10)

ax = axes[2]
ax.plot(tsim, sim_results[5,:], 'k-')
ax.set_ylabel('Angular Velocity [rad/s]')
ax.grid('major')
ax.set_title('phidot')
ax.set_xlabel('Time [s]')
ax.set_ylim(0,10)

# Aero
(fig, axes) = plt.subplots(nrows=3, ncols=1, sharex=True, num='Aerodynamics vs Time')

ax = axes[0]
ax.plot(tsim, sim_results[12,:], 'r-')
# ax.legend(('x', 'y', 'z'))
ax.set_ylabel('Lift [N]')
ax.grid('major')
ax.set_title('Lift')
ax.set_ylim(0,10)

ax = axes[1]
ax.plot(tsim, sim_results[13,:], 'b-')
# ax.legend(('x', 'y', 'z'))
ax.set_ylabel('Drag [N]')
ax.grid('major')
ax.set_title('Drag')
ax.set_ylim(0,10)

ax = axes[2]
ax.plot(tsim, sim_results[14,:], 'k-')
ax.set_ylabel('Pitch Moment [Nm]')
ax.grid('major')
ax.set_title('Pitch Moment')
ax.set_xlabel('Time [s]')
ax.set_ylim(-5,5)

# Airspeeds
(fig, axes) = plt.subplots(nrows=3, ncols=1, sharex=True, num='Airspeed vs Time')

ax = axes[0]
ax.plot(tsim, sim_results[6,:], 'r-')
# ax.legend(('x', 'y', 'z'))
ax.set_ylabel('Speed [m/s]')
ax.grid('major')
ax.set_title('Airspeed Over Wing')
ax.set_ylim(0,10)

ax = axes[1]
ax.plot(tsim, sim_results[7,:], 'b-')
# ax.legend(('x', 'y', 'z'))
ax.set_ylabel('Speed [m/s]')
ax.grid('major')
ax.set_title('Prop Wash Airspeed')
ax.set_ylim(0,10)

ax = axes[2]
ax.plot(tsim, sim_results[8,:], 'k-')
ax.set_ylabel('Speed [m/s]')
ax.grid('major')
ax.set_title('Inertial Airspeed')
ax.set_xlabel('Time [s]')
ax.set_ylim(-5,5)

# Misc orientations
(fig, axes) = plt.subplots(nrows=3, ncols=1, sharex=True, num='Angles of Attack vs Time')

ax = axes[0]
ax.plot(tsim, sim_results[9,:], 'r-')
# ax.legend(('x', 'y', 'z'))
ax.set_ylabel('alpha [rad]')
ax.grid('major')
ax.set_title('Angle of Attack')
ax.set_ylim(-pi/2-0.1,pi/2+0.1)

ax = axes[1]
ax.plot(tsim, sim_results[10,:], 'b-')
# ax.legend(('x', 'y', 'z'))
ax.set_ylabel('alpha_e [rad]')
ax.grid('major')
ax.set_title('Effective Angle of Attack')
ax.set_ylim(-pi/2-0.1,pi/2+0.1)

ax = axes[2]
ax.plot(tsim, sim_results[11,:], 'k-')
ax.set_ylabel('gamma [rad]')
ax.grid('major')
ax.set_title('Inertial Orientation')
ax.set_xlabel('Time [s]')
ax.set_ylim(-pi/2-0.1,pi/2+0.1)

plt.show()


# out1 = qbit.publishValues()
# print(out1)
# qbit.setCmds(qbit.m*qbit.g, qbit.m*qbit.g, 0)
# qbit.updateStates(dt)
# out2 = qbit.publishValues()
# print(out2)