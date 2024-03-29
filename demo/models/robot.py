import numpy as np
from scipy.integrate import odeint
from scipy.linalg import solve_continuous_are, inv, solve_discrete_are
from math import sin, cos, tan, atan, pi, sqrt


class dubins_car:
    def __init__(self) -> None:
        #############################
        # Tracking controller gains #
        #############################
        self.k1 = 1000
        self.k2 = 1000
        self.k3 = 1000

        self.a = 1000
        self.b = 1
        #######################################################
        # Reference state-input trajectories (For simulation) #
        #######################################################
        self.state_ref = None
        self.input_ref = None

        self.state_ref_traj = None
        self.input_ref_traj = None

        # ##########################
        # # Initializing car state #
        # ##########################
        # self.initial_state = initial_state 
        # # self.initial_mode = initial_mode
        # self.curr_state = initial_state # For simulation

    def dubinsDynamics(self, state, t, input):
        x,y,s,c = state
        v, w = input

        xdot = v*cos(theta)
        ydot = v*sin(theta)
        sdot = w*cos(theta)
        cdot = w*sin(theta)
        
        return [xdot, ydot, sdot, cdot]

    def trackingControl(self, curr_state, ref_state, ref_input):
        x,y,s,c = curr_state
        xref,yref,thetaref = ref_state
        vref,wref = ref_input

        xerr = (xref - x)*c + (yref - y)*s
        yerr = (xref - x)*(-s) + (yref - y)*s
        serr = sin(thetaref)*c - cos(thetaref)*s
	    cerr = cos(thetaref)*c + sin(thetaref)*s

        v = self.k2*xerr
        w = self.k1*vref*yerr*(1+cerr/1000)**2 + self.k2*serr*((1+cerr/a)**2)

        input = [v, w]

        return input

    def errBound(self, init_poly, i):
        err_0 = init_poly.chebR
        err = sqrt(err_0**2 + (4*i)/(self.k2))
        # err = 0.1
        return err

    def simulate(self, mode, initial_state, time_horizon, time_step):
        time_array = np.arange(0, time_horizon+time_step, time_step)
        if self.state_ref == None and self.input_ref == None:
            # No controller used here!
            input = [0, 0]
        elif (self.state_ref == None and self.input_ref != None) or (self.state_ref != None and self.input_ref == None):
            # Either the state or reference trajectory is not defined
            raise Exception('Both state and input trajectories must be defined!')
        else:
            # This is where both state and reference trajectories are defined
            input = self.trackingControl(initial_state, self.state_ref, self.input_ref)

        state_trace = odeint(self.dubinsDynamics, initial_state, time_array, args=(input,))

        trace = []

        for i in range(len(time_array)):
            trace.append([time_array[i]]+list(state_trace[i]))

        return trace

    # def TC_simulate(self, mode, initialSet, time_horizon, time_step, map=None):
    #     #TODO: Implement TC simulate for reachability
    #     return None
