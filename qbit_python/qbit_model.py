'''
QBiT Dynamic Model
Spencer Folk 2020

This script contains the dynamic model of the qbit.. including the constants and relevant methods. 
'''

import numpy as np
from math import *
from pprint import pprint

class quadrotor_biplane_tailsitter():
	'''
	The QBIT object is our dynamic model of the tailsitter. 
	'''

	def __init__(self, initial_states, m=0.150, Iyy=2.32e-3, l = 6, span=15, chord=5, R=2.5, b_flap=0, c_flap=0):
		'''
		b_flap=7.46e-6, c_flap=2.18e-4
		CONSTANTS
		m 		[kg] 				vehicle mass
		Iyy		[kg-m2] 			moment of inertia in page
		span 	[in] 				wing span (input in inches, converted later)
		chord 	[in] 				wing chord (input in inches, converted later)
		R 		[in] 				propeller radius (input in inches, converted later)
		b_flap	[Nm/(m/s)] 		 	aero elevon characteristic
		c_flap 	[Nm/(m/s)/rad] 		aero elevon characteristic
		S 		[m2] 				wing area
		rho 	[kg/m^3] 			ambient air density
		l 		[m] 				dist between rotors (input in inches, convert later)

		STATES
		[x, z, phi, xdot, zdot, phidot]

		INPUTS
		[T_top, T_bot, delta]

		'''

		## Constants
		in2m = 0.0254
		self.g = 9.81
		self.rho = 1.2 

		self.m, self.Iyy, self.b_flap, self.c_flap = m, Iyy, b_flap, c_flap
		self.span, self.chord, self.R, self.l = span*in2m, chord*in2m, R*in2m, l*in2m

		print("QBIT Initialized")
		print("----------------")
		print("Initial States: ")
		pprint(initial_states)
		print("\n\n")

		self.S = span*chord

		## States
		self.x, self.z, self.phi = initial_states['x'], initial_states['z'], initial_states['phi']
		self.xdot, self.zdot, self.phidot = initial_states['xdot'], initial_states['zdot'], initial_states['phidot']
		self.xdotdot, self.zdotdot, self.phidotdot = 0,0,0
		self.T_top, self.T_bot, self.delta = 0,0,0

		## Intermediate values
		self.Va, self.Vi, self.Vw = 0,0,0
		self.alpha, self.alpha_e, self.gamma = 0,0,0

		self.L, self.D, self.M_air = 0,0,0
		self.Cl, self.Cd, self.Cm = 0,0,0

	def publishValues(self):
		'''
		Creates a dictionary of values to send out to the master script
		'''
		# output = np.array([self.x, self.z, self.phi, self.xdot, self.zdot, self.phidot, \
		# 					self.Va, self.Vi, self.Vw, \
		# 					self.alpha, self.alpha_e, self.gamma, \
		# 					self.L, self.D, self.M_air])

		output = {'x': self.x,
					'z':self.z,
					'phi':self.phi,
					'xdot':self.xdot,
					'zdot':self.zdot,
					'phidot':self.phidot,
					'Va':self.Va,
					'Vw':self.Vw,
					'Vi':self.Vi,
					'alpha':self.alpha,
					'alpha_e':self.alpha_e,
					'gamma':self.gamma,
					'L':self.L,
					'D':self.D,
					'M_air':self.M_air}
		return output

	def setCmds(self, T_top, T_bot, delta):
		'''
		Use this function to set the inputs.. should be run before updateStates() method
		'''
		self.T_top = T_top
		self.T_bot = T_bot
		self.delta = delta


	def updateStates(self, dt):
		'''
		This method updates the states by computing the derivative and doing euler integration
		'''

		# First we need to update our relevant speeds based on our current X
		self.Vi = np.sqrt(self.xdot**2 + self.zdot**2)
		self.Vw = 1.2*np.sqrt( 0.5*(self.T_top + self.T_bot)/(8*self.rho*pi*self.R**2) )  # Note we're estimating Vw with the average thrust force over the wing

		if self.Vi <= 0.1:
			self.alpha = self.phi
		else:
			self.gamma = np.arctan2(self.zdot, self.xdot)
			self.alpha = self.phi - self.gamma

		self.Va = np.sqrt( self.Vi**2 + self.Vw**2 + 2*self.Vi*self.Vw*np.cos(self.alpha) )

		# Now we can find the effective angle of attack
		self.alpha_e = np.arcsin(self.Vi*np.sin(self.alpha)/self.Va)

		# Use this effective angle of attack to compute lift, drag, and pitching moments.
		self.Cl, self.Cd, self.Cm = self.aeroCoefficients(self.alpha_e)
		self.L = 0.5*self.rho*self.Va**2*self.S*self.Cl
		self.D = 0.5*self.rho*self.Va**2*self.S*self.Cd
		self.M_air = 0.5*self.rho*self.Va**2*self.S*self.chord*self.Cm

		# Now we can compute derivatives
		self.xdotdot, self.zdotdot, self.phidotdot = self.calcDerivatives()

		# Update states by doing euler integration
		self.xdot += self.xdotdot*dt
		self.zdot += self.zdotdot*dt
		self.phidot += self.phidotdot*dt

		self.x += self.xdot*dt
		self.z += self.zdot*dt
		self.phi += self.phidot*dt
		
		return self.x, self.z, self.phi, self.xdot, self.zdot, self.phidot


	def calcDerivatives(self):
		'''
		This is our xdot = f(x,u)
		'''
		xdotdot = ( (self.T_top + self.T_bot)*np.cos(self.phi) - self.D*np.cos(self.phi - self.alpha_e) - self.L*np.sin(self.phi - self.alpha_e) )/self.m
		zdotdot = ( -self.m*self.g + (self.T_top + self.T_bot)*np.sin(self.phi) - self.D*np.sin(self.phi - self.alpha_e) + self.L*np.cos(self.phi - self.alpha_e) )/self.m
		phidotdot = ( self.M_air + (self.b_flap + self.c_flap*self.delta)*self.Va**2 + self.l*(self.T_bot - self.T_top) )/self.Iyy

		return xdotdot, zdotdot, phidotdot


	def aeroCoefficients(self, alpha):
		'''
		This returns aerodynamic moment and force coefficients from the wing based on angle of attack
		'''
		Cl = 2*np.sin(alpha)*np.cos(alpha)
		Cd = 2*np.sin(alpha)**2
		Cm = 0*np.sin(alpha)*np.cos(alpha)

		return Cl, Cd, Cm
		

class planar_quad():
	'''
	For testing purposes this planar quad is introduced to debug some basic sim bugs.
	This ensures the simulation is working as expected and not due to error in dynamics.
	'''

	def __init__(self, initial_states, m=0.150, Iyy=2.32e-3, l = 6, R=2.5):

		## Constants
		in2m = 0.0254
		self.g = 9.81
		self.rho = 1.2 

		self.m, self.Iyy = m, Iyy
		self.R, self.l = R*in2m, l*in2m

		print("Planar Quadrotor Initialized")
		print("----------------")
		print("Initial States: ")
		pprint(initial_states)
		print("\n\n")

		## States
		self.x, self.z, self.phi = initial_states['x'], initial_states['z'], initial_states['phi']
		self.xdot, self.zdot, self.phidot = initial_states['xdot'], initial_states['zdot'], initial_states['phidot']
		self.xdotdot, self.zdotdot, self.phidotdot = 0,0,0
		self.T_left, self.T_right = 0,0

	def publishValues(self):
		'''
		Creates a dictionary of values to send out to the master script
		'''
		# output = np.array([self.x, self.z, self.phi, self.xdot, self.zdot, self.phidot, \
		# 					self.Va, self.Vi, self.Vw, \
		# 					self.alpha, self.alpha_e, self.gamma, \
		# 					self.L, self.D, self.M_air])

		output = {'x': self.x,
					'z':self.z,
					'phi':self.phi,
					'xdot':self.xdot,
					'zdot':self.zdot,
					'phidot':self.phidot}
		return output


	def setCmds(self, T_left, T_right):
		'''
		Use this function to set the inputs.. should be run before updateStates() method
		'''
		self.T_left = T_left
		self.T_right = T_right


	def updateStates(self, dt):
		'''
		This method updates the states by computing the derivative and doing euler integration
		'''

		# Now we can compute derivatives
		self.xdotdot, self.zdotdot, self.phidotdot = self.calcDerivatives()

		# Update states by doing euler integration
		self.xdot += self.xdotdot*dt
		self.zdot += self.zdotdot*dt
		self.phidot += self.phidotdot*dt

		self.x += self.xdot*dt
		self.z += self.zdot*dt
		self.phi += self.phidot*dt
		
		return self.x, self.z, self.phi, self.xdot, self.zdot, self.phidot

	def calcDerivatives(self):
		'''
		This is our xdot = f(x,u)
		'''
		xdotdot = (self.T_left + self.T_right)*np.cos(self.phi)
		zdotdot = -self.m*self.g + (self.T_left + self.T_right)*np.sin(self.phi)
		phidotdot = self.l*(self.T_right - self.T_left)

		return xdotdot, zdotdot, phidotdot