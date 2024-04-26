import numpy as np
from scipy.integrate import odeint
from math import sin, cos, tan, atan, pi, sqrt, ceil

import matplotlib.pyplot as plt

# Define these globally if they are to be accessed in functions below

#xref = []
#yref = []
pos=[]
ref_trajectory1=[]
x1 =[]
y1 =[]
def initialize_reference_trajectory_and_input():
	global k,T,dt,length,G,kappa,velocity
	G = 1000
	kappa = 0.1
	length = 1
	dt = 0.1 # Timestep
	#T = 10 # Total time for this part of the motion
	velocity = 2 # Constant velocity
	k = 100 # Damping coefficient from the dynamics
	
	# Initial conditions and computation
	

# Make sure to call this function before running simulations or functions that depend on ref_traj and ref_input
initialize_reference_trajectory_and_input()

def ref_traj(t):
	output=[] 
	x_initial=0
	y_initial =0
	heading_initial=np.pi/4
	ref_turning =np.pi/6
	x_ref =x_initial + t*velocity*cos(heading_initial)
	y_ref=y_initial +t*velocity*sin(heading_initial)
	heading_ref = heading_initial
	return [x_ref, y_ref, heading_ref, velocity, ref_turning]

	
def ref_input(t):
	res =[]
	k = 100  # Damping coefficient from the dynamics
	con_velocity = 0
	force = 0
	return [force,con_velocity]
	
# Now ref_traj and ref_input are ready to be used in your simulations or functions

def dynamics(state, t, input):
	x,y,heading,vel,turning = state
	force,torque = input
	xdot = vel*cos(heading)
	ydot = vel*sin(heading)
	headingdot = turning
	veldot = force - k*vel
	turningdot = torque - k*turning
	return [xdot, ydot, headingdot, veldot, turningdot]

def trackingController(state, ref_state, ref_input):
	x,y,heading,vel,turning = state
	xref,yref,headingref,velref,turningref = ref_state
	aref = 0
	z1diff = x + length*cos(heading) - (xref + length*cos(headingref))
	z2diff = y + length*sin(heading) - (yref + length*sin(headingref))

	gx = z1diff*(G/(1 + np.linalg.norm(np.array([z1diff, z2diff]))))
	gy = z2diff*(G/(1 + np.linalg.norm(np.array([z1diff, z2diff]))))

	vx = velref*cos(heading) - length*sin(headingref)*turningref
	vy = velref*sin(heading) + length*cos(headingref)*turningref

	ax = aref*cos(heading)
	ay = aref*sin(heading)

	u1 = ax + k*vx - gx
	u2 = ay + k*vy - gy
	f = cos(heading)*(u1 + vel*turning*sin(heading) +length*(turning**2)*cos(heading)) + sin(heading)*(u2 - vel*turning*cos(heading) + length*(turning**2)*sin(heading))
	tau = (-sin(heading)/length)*(u1 + vel*turning*sin(heading) + length*(turning**2)*cos(heading)) + (cos(heading)/length)*(u2 - vel*turning*cos(heading) + length*(turning**2)*sin(heading))
	return [f, tau]

def controlledDynamics(state, t):
        res =[]
        ref_state1 = ref_traj(t)
        ref_input1 = ref_input(t)
        x,y,heading,vl,turning = state

        input = trackingController(state, ref_state1, ref_input1)
        xdot, ydot, headingdot, veldot, turningdot = dynamics(state, t, input)
        res =[xdot, ydot, headingdot, veldot, turningdot]
        return res

def errorDynamics(error_state, t):
        res =[]
        ref_state1 = ref_traj(t)
        ref_input1 = ref_input(t)
        err_x,err_y,err_heading,err_vl,err_turning = error_state
        ref_x,ref_y,ref_heading,ref_vl,ref_turning = ref_state1
        #xref.append(ref_x)
        #yref.append(ref_y)
        state =[err_x+ref_x,err_y+ref_y,err_heading+ref_heading, err_vl+ref_vl, err_turning+ref_turning]
        input = trackingController(state, ref_state1, ref_input1)
        xdot, ydot, headingdot, veldot, turningdot = dynamics(state, t, input)
        xdot1, ydot1, headingdot1, veldot1, turningdot1 = dynamics(ref_state1, t, ref_input1)
        res =[xdot-xdot1,ydot-ydot1,headingdot-headingdot1,veldot-veldot1,turningdot-turningdot1]
        #print(res)
        return res

def run_simulation(initial_state, T, dt):
	global ref_trajectory1,x1,y1
	ref_trajectory =[]
	x =[]
	y=[]
	number_p = int(np.ceil(T/dt))
	T = float(T)
	t = [i*dt for i in range(0,number_p)]
	if t[-1] != dt:
		t.append(T)
	newt = []
	for step in t:
		newt.append(float(format(step, '.2f')))
	t = newt
	x_init = initial_state[0]
	y_init = initial_state[1]
	head_init = initial_state[2]
	vel_init = initial_state[3]
	turn_init = initial_state[4]
	initial_ode = [x_init,y_init,head_init,vel_init,turn_init]
	sol = odeint(controlledDynamics, initial_ode, newt, hmax=dt)
	t_p = np.arange(0,T+dt, dt)
	#for i in t_p:
		#ref_p = ref_traj(i)
		#ref_trajectory.append(ref_p)
	#pos_mag = np.sqrt(sol[:,0]**2 +sol[:,1]**2)
	#mag = np.linalg.norm(sol, axis=1)
	trace = []

	pos_mag = []
	for i in range(len(newt)):
		tmp =[]
		tmp.append(t[i])
		tmp.append(sol[i,0])
		tmp.append(sol[i,1])
		tmp.append(sol[i,2])
		tmp.append(sol[i,3])
		tmp.append(sol[i,4])
		x.append(sol[i,0])
		y.append(sol[i,1])
		#pos_mag.append(np.sqrt(sol[:,0]**2 +sol[:,1]**2))
		ref_p = ref_traj(i)
		ref_trajectory.append(ref_p)
		trace.append(tmp)
	for j in range(len(newt)):
		a = np.sqrt(x[j]**2+y[j]**2)
		pos_mag.append(a)
		pos.append(a)
	ref_trajectory= np.array(ref_trajectory)
	ref_trajectory1 = ref_trajectory
	x1 =x
	y1= y
	plt.figure(figsize = (8,6))
	#plt.plot(t,x, label='x magnitudes')
	#plt.plot(t,y, label='y magnitudes')
	#plt.plot(t,pos_mag, label= 'magnitudes')
	plt.plot(x,y, label='actual trajectory')
	#plt.plot(ref_trajectory[:,0],ref_trajectory[:,1], label='reference trajectory')
	#plt.plot(xref,yref, label = 'Ref traj')
	plt.title('trajectory.plot')
	plt.xlabel('x coordinate')
	plt.ylabel('y coordinate')
	#plt.xlabel('time')
	#plt.ylabel('magnitudes')
	plt.legend()
	plt.grid(True)
	plt.show()
	return trace

def actualx():
	xx = np.array(x1)
	return xx

def actualy():
	yy = np.array(y1)
	return yy

def reference():
	reff = np.array(ref_trajectory1)
	return reff
	
    # TODO: Implement error bounds
    # TODO: Implement error dynamics
    # TODO: Implement TC Simulate for error dynamics (Can be used with DryVr)
    # TODO: Implement simulations (particularly dynamic-FACTEST simulation)
    
