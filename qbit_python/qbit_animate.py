'''
This script is meant to take in state variables and simulate data in real time
Spencer Folk 2020
'''
from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np

def animate_quad(time, position, orientation):
	'''
	Animate the completed simulation given a time vector, position states (x,z) and orientation (phi)
	The bulk of this code was supplied by Jimmy Paulos as a simulator for MEAM 620
	'''

	rtf = 1.0 # real tiem factor > 1.0 is faster than real time playback
	render_fps = 30
	close_on_finish = False


	blit = True # Rendering details

	(fig, ax) = plt.figure('Animation')

	ax.set_xlim(-5,5)
	ax.set_ylim(-5,5)
	ax.set_zlim(-5,5)

	# SUPPLY THESE FUNCTIONS
	def update(frame):
		plt.plot(position[0,frame], position[1,frame],'r-')

	ani = animation.FuncAnimation(fig = fig, 
									func = update,
									frames = time.size,
									init_func = init,
									interval = 1000.0/render_fps,
									repeat=False,
									blit=blit)

def test():

	def dummy_sim_result():
		t = np.arange(0,10,1/100)
		position = np.zeros((3,t.shape[0]))
		position[0,:] = np.cos(t)
		position[1,:] = np.sin(t)
		phi = t % (2*np.pi)
		return (t, position, phi)

	(time, position, rotation) = dummy_sim_result()

	animate_quad(time, position, rotation)

if __name__ == '__main__':
	test()
