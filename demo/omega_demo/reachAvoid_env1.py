#######################################
# Import required libraries and files #
#######################################

## Set up file paths ##
#######################
import sys, os
currFile = os.path.abspath(__file__)
modelPath = currFile.replace('/demo/omega_demo/reachAvoid_env1.py', '')
sys.path.append(modelPath)
factestPath = currFile.replace('/demo/omega_demo/reachAvoid_env1.py', '/factest/synthesis')
sys.path.append(factestPath)

## Import Python libraries ##
#############################
import numpy as np
import matplotlib.pyplot as plt
import polytope as pc

from math import pi

## Import FACTEST files ##
##########################
from factest.synthesis.omega_factest_v2 import buchi_from_ltl, omega_FACTEST
from demo.models.dubins_car import dubins_car

## Import plotting stuff ##
###########################
from factest.plotting.plot_polytopes import plotPoly

if __name__=="__main__":
    model = dubins_car()

    A = np.array([[-1,0],[1,0],[0,-1],[0,1]])

    b_goal1 = np.array([-5,7,-5,7])
    b_goal2 = np.array([7,-5,7,-5])

    b_unsafe1 = np.array([11,-10,11,11])
    b_unsafe2 = np.array([-10,11,11,11])
    b_unsafe3 = np.array([11,11,-10,11])
    b_unsafe4 = np.array([11,11,11,-10])
    b_unsafe5 = np.array([11,-2,1,1])
    b_unsafe6 = np.array([-2,11,1,1])

    b_workspace = np.array([10,10,10,10])

    workspace_poly = pc.Polytope(A, b_workspace)

    E1 = pc.Polytope(A, b_goal1) # goal set 1
    E2 = pc.Polytope(A, b_goal2) # goal set 2
    E3 = pc.Polytope(A, b_unsafe5) # unsafe set 1
    E4 = pc.Polytope(A, b_unsafe6) # unsafe set 2

    env = {'E1':E1,'E2':E2,'E3':E3,'E4':E4}   

    reach_str = 'F E1 & F E2 & G (E1 -> F E2) & G (E2 -> F E1)'
    avoid_str = 'G !E3 & G !E4'
    ltl_formula = reach_str + ' & ' + avoid_str

    myBuchi = omega_FACTEST(ltl_formula=ltl_formula,env=env,model=model, workspace=workspace_poly)
    myBuchi.runOmega()

    sample_run = myBuchi.exampleRun()
    print(sample_run)

    init_poly = env[sample_run[0]]
    print(init_poly.b)

    min_x = init_poly.b[0]*-1
    min_y = init_poly.b[2]*-1
    max_x = init_poly.b[1]
    max_y = init_poly.b[3]

    curr_state = [6,6,0]
    rand_init = [np.random.random()*(max_x-min_x)+min_x, np.random.random()*(max_y-min_y)+min_y,0]

    sim_states = model.simulate_run(rand_init, sample_run, myBuchi.flow_cache, myBuchi.init_sets)

    xsim = [state[0] for state in sim_states]
    ysim = [state[1] for state in sim_states]

    fig = plt.figure()
    
    ax = fig.add_subplot(111)
    plotPoly(E1,ax,'green')
    plotPoly(E2,ax,'green')

    plotPoly(E3,ax,'red')
    plotPoly(E4,ax,'red')

    for init_key in myBuchi.init_sets.keys():
        init_poly = myBuchi.init_sets[init_key]
        plotPoly(init_poly,ax,'green')

    
    ax.plot(rand_init[0], rand_init[1], marker = '*',markersize=5)
    ax.plot(xsim,ysim, linestyle = '--')

    ax.set_xlim(-10,10)
    ax.set_ylim(-10,10)
    ax.xaxis.set_tick_params(labelbottom=False)
    ax.yaxis.set_tick_params(labelleft=False)

    plt.show()