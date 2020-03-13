'''
Quadrotor Biplane Tailsitter simulate holds functions relevant to simulating and saving data
'''

import numpy as np
from math import *


def simulate(qbit, trajectory, controller, tf, dt):
	'''
	Simulate function will iterate the updateStates() and publishValues method from qbit for a given duration
	at a given sampling period.

	INPUTS
	qbit - quadrotor biplane tailsitter object
	trajectory - trajectory to be followed (a path in time)
	controller - qbit controller
	tf - duration of simulation (s)
	dt - sampling period (s)

	'''

	print('Begin simulation...')
	for idx in range(0,tf/dt):
		# At each iteration grab the next state from trajectory, input to controller and get the right inputs to send to the motors.
		x = 0



	print('Simulation end time reached')
	return 0