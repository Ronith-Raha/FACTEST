#######################################
# Import required libraries and files #
#######################################

## Set up file paths ##
#######################
import sys, os
currFile = os.path.abspath(__file__)
modelPath = currFile.replace('/demo/omega_demo/reachAvoid_env2.py', '')
sys.path.append(modelPath)
factestPath = currFile.replace('/demo/omega_demo/reachAvoid_env2.py', '/factest/synthesis')
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

    b_obstacle = np.array([[-1.5,5.5,-1.3,3.7]])

    b_1 = np.array([-3.7,5.2,-0.2,1.3])
    b_2 = np.array([-3.7,5.2,-3.7,5.2])

    b_workspace = np.array([0,5.2,0,5.2])
    # b_workspace2 = np.array([-0.5,5,0,5])


    obstacle_poly = pc.Polytope(A,b_obstacle)

    goal_poly_1 = pc.Polytope(A,b_1)
    goal_poly_2 = pc.Polytope(A,b_2)

    workspace_poly = pc.Polytope(A,b_workspace)
    # workspace_poly2 = pc.Polytope(A,b_workspace2)

    env = {'E1':goal_poly_1,'E2':goal_poly_2,'O1':obstacle_poly}   

    ltl_formula = 'G (F E1) & G (F E2) & G !O1'

    myBuchi = omega_FACTEST(ltl_formula=ltl_formula,env=env,model=model, workspace=workspace_poly)

    sample_run = myBuchi.exampleRun()
    print(sample_run)

    init_poly = myBuchi.init_sets[sample_run[0]]
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
    plotPoly(goal_poly_1,ax,'green')
    plotPoly(goal_poly_2,ax,'green')

    plotPoly(obstacle_poly,ax,'red')

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