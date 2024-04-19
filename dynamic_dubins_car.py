import numpy as np
from scipy.integrate import odeint
from math import sin, cos, tan, atan, pi, sqrt, ceil



# Define these globally if they are to be accessed in functions below


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
	heading_initial=0
	ref_turning =0
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

def controlledDynamics(error_state, t):
        res =[]
        ref_state1 = ref_traj(t)
        ref_input1 = ref_input(t)
        err_x,err_y,err_heading,err_vl,err_turning = error_state
        ref_x,ref_y,ref_heading,ref_vl,ref_turning = ref_state1
        state =[err_x+ref_x,err_y+ref_y,err_heading+ref_heading, err_vl+ref_vl, err_turning+ref_turning]
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
        state =[err_x+ref_x,err_y+ref_y,err_heading+ref_heading, err_vl+ref_vl, err_turning+ref_turning]
        input = trackingController(state, ref_state1, ref_input1)
        xdot, ydot, headingdot, veldot, turningdot = dynamics(state, t, input)
        xdot1, ydot1, headingdot1, veldot1, turningdot1 = dynamics(ref_state1, t, ref_input1)
        res =[xdot-xdot1,ydot-ydot1,headingdot-headingdot1,veldot-veldot1,turningdot-turningdot1]
        return res

def run_simulation(initial_state, T, dt):
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
	sol = odeint(errorDynamics, initial_ode, newt, hmax=dt)

	trace = []
	for i in range(len(newt)):
		tmp =[]
		tmp.append(t[i])
		tmp.append(sol[i,0])
		tmp.append(sol[i,1])
		tmp.append(sol[i,2])
		tmp.append(sol[i,3])
		tmp.append(sol[i,4])
		trace.append(tmp)
	return trace


    # TODO: Implement error bounds
    # TODO: Implement error dynamics
    # TODO: Implement TC Simulate for error dynamics (Can be used with DryVr)
    # TODO: Implement simulations (particularly dynamic-FACTEST simulation)
    
