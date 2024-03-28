import numpy as np
from scipy.integrate import odeint
from math import sin, cos, tan, atan, pi, sqrt, ceil

class dynamic_planar_dubins:
    def __init__(self) -> None:
        ##################
        # Car parameters #
        ##################
        self.length = 1
        self.k = 100

        #########################
        # Controller parameters #
        #########################
        self.kappa = 0.1
        self.G = 1000

        #######################################################
        # Reference state-input trajectories (For simulation) #
        #######################################################
        self.ref_traj = None
        self.ref_input = None
        
        self.dt = 0.01

    def errBound(self, init_poly, i):
        return 0.1

    def dynamics(self, state, t, input):
        x,y,heading,vel,turning = state
        force,torque = input

        xdot = vel*cos(heading)
        ydot = vel*sin(heading)
        headingdot = turning
        veldot = force - self.k*vel
        turningdot = torque - self.k*turning

        return [xdot, ydot, headingdot, veldot, turningdot]

    def trackingController(self, state, ref_state, ref_input):
        x,y,heading,vel,turning = state
        xref,yref,headingref,velref,turningref = ref_state

        aref = 0

        z1diff = x + self.length*cos(heading) - (xref + self.length*cos(headingref))
        z2diff = y + self.length*sin(heading) - (yref + self.length*sin(headingref))

        gx = z1diff*(self.G/(1 + np.linalg.norm(np.array([z1diff, z2diff]))))
        gy = z2diff*(self.G/(1 + np.linalg.norm(np.array([z1diff, z2diff]))))

        vx = velref*cos(heading) - self.length*sin(headingref)*turningref
        vy = velref*sin(heading) + self.length*cos(headingref)*turningref

        ax = aref*cos(heading)
        ay = aref*sin(heading)

        u1 = ax + self.k*vx - gx
        u2 = ay + self.k*vy - gy

        f = cos(heading)*(u1 + vel*turning*sin(heading) +self.length*(turning**2)*cos(heading)) + sin(heading)*(u2 - vel*turning*cos(heading) + self.length*(turning**2)*sin(heading))
        tau = (-sin(heading)/self.length)*(u1 + vel*turning*sin(heading) + self.length*(turning**2)*cos(heading)) + (cos(heading)/self.length)*(u2 - vel*turning*cos(heading) + self.length*(turning**2)*sin(heading))

        # forceref,torqueref = ref_input

        # aref = (forceref - self.k*velref)

        # zx = x + self.length*cos(heading) - (xref + self.length*cos(headingref))
        # zy = y + self.length*sin(heading) - (yref + self.length*sin(headingref))

        # vx = velref*cos(heading) - self.length*sin(headingref)*turningref
        # vy = velref*sin(heading) + self.length*cos(headingref)*turningref

        # ax = aref*cos(headingref)
        # ay = aref*sin(headingref)

        # gx = zx*(self.G/(1 + np.linalg.norm(np.array([zx, zy]))))
        # gy = zy*(self.G/(1 + np.linalg.norm(np.array([zx, zy]))))

        # ux = ax + self.k*vx - gx
        # uy = ay + self.k*vy - gy

        # force = cos(heading)*(ux + vel*turning*sin(heading)) + self.length*(turning**2)*cos(heading) + sin(heading)*(uy - vel*turning*cos(heading) + self.length*(turning**2)*sin(heading)) + self.length*turning**2
        # torque = (-sin(heading)/self.length)*(ux + vel*turning*sin(heading) + self.length*(turning**2)*cos(heading)) + (cos(heading)/self.length)*(uy - vel*turning*cos(heading) + self.length*(turning**2)*sin(heading)) - (vel*turning/self.length)

        return [f, tau]

    def controlledDynamics(self, state, t):
        ref_idx = ceil(t/self.dt)
        if ref_idx >= len(self.ref_traj):
            ref_idx = len(self.ref_traj)-1

        ref_state = self.ref_traj[ref_idx]
        ref_input = self.ref_input[ref_idx]

        input = self.trackingController(state, ref_state, ref_input)
        xdot, ydot, headingdot, veldot, turningdot = self.dynamics(state, t, input)

        return [xdot, ydot, headingdot, veldot, turningdot]

    def set_timed_ref(self, xref):
        self.ref_traj = []
        self.ref_input = []

        curr_time = 0
        prev_t = 0
        for i in range(len(xref)-1):
            p1 = xref[i]
            p2 = xref[i+1]

            mx = p2[0] - p1[0]
            bx = p1[0]
            
            my = p2[1] - p1[1]
            by = p1[1]
            
            theta_ref = np.arctan2((np.array(p2) - np.array(p1))[1], (np.array(p2) - np.array(p1))[0])

            t = p2[2] - p1[2]
            vref = np.linalg.norm(np.array(p2) - np.array(p1))/t

            while curr_time <= t + prev_t:
                px = mx*((curr_time - prev_t)/t) + bx
                py = my*((curr_time - prev_t)/t) + by
                self.ref_traj.append((px,py,theta_ref,vref,0))
                self.ref_input.append((0,0))
                curr_time += self.dt

            prev_t += t
            
        return None

    def run_simulation(self, xref, initial_state, T, vref = 1, sim_type = "base"): #TODO: MAY WANT TO COME UP WITH A BETTER WAY TO DO THIS
        if sim_type == "base":
            self.set_ref(xref, vref)
        else:
            self.set_timed_ref(xref)

        time_array = np.arange(0,T,self.dt)
        state_trace = odeint(self.controlledDynamics, initial_state, time_array)
        return state_trace

    # TODO: Implement error bounds
    # TODO: Implement error dynamics
    # TODO: Implement TC Simulate for error dynamics (Can be used with DryVr)
    # TODO: Implement simulations (particularly dynamic-FACTEST simulation)
    
