#######################################
# Import required libraries and files #
#######################################

## Set up file paths ##
#######################
import sys, os
currFile = os.path.abspath(__file__)
modelPath = currFile.replace('/demo/omega_demo/nursery.py', '')
sys.path.append(modelPath)
factestPath = currFile.replace('/demo/omega_demo/nursery.py', '/factest/synthesis')
sys.path.append(factestPath)

## Import Python libraries ##
#############################
import numpy as np
import matplotlib.pyplot as plt
import polytope as pc

from math import pi

## Import FACTEST files ##
##########################
from factest.synthesis.omega_factest_v2_modified import buchi_from_ltl, omega_FACTEST
from demo.models.dubins_car import dubins_car

## Import plotting stuff ##
###########################
from factest.plotting.plot_polytopes import plotPoly

if __name__=="__main__":
    model = dubins_car()

    A = np.array([[-1,0],[1,0],[0,-1],[0,1]])

    # Initial set
    b0 = np.array([6,-5,0.5,0.5])
    init_set = pc.Polytope(A,b0)

    # Baby
    b1 = np.array([-5,7,-5,7])
    baby = pc.Polytope(A,b1)

    # Charger
    b2 = np.array([7,-5,7,-5])
    charger = pc.Polytope(A,b2)

    # Adult
    b3 = np.array([-5,7,7,-5])
    adult = pc.Polytope(A,b3)

    # Obstacles 
    b4 = np.array([2.5,-2,11,4])
    b5 = np.array([-2,11,0.5,0.5])
    b6 = np.array([2.5,-2,-9,11])
    b7 = np.array([-2,2.5,11,-9])

    unsafe1 = pc.Polytope(A,b4)
    unsafe2 = pc.Polytope(A,b5)
    unsafe3 = pc.Polytope(A,b6)
    unsafe4 = pc.Polytope(A,b7)

    b_workspace = np.array([10,10,10,10])
    workspace_poly = pc.Polytope(A,b_workspace)
    
    env = {'b':baby,'c':charger,'a':adult,'O1':unsafe1,'O2':unsafe2,'O3':unsafe3,'O4':unsafe4}   

    ltl_formula = """c & (G !O1 & G !O2 & G !O3 & G !O4) & 
                     (G ((b & !(X b)) -> (X !b U (a | c)))) &
                     (G (a -> X (!a U b))) &
                     (G ((!b & X b & !(X (X b))) -> (!a U c))) &
                     (G (c -> (!a U b))) &
                     G ((b & X b) -> F a) &
                     F (b & X b) & G (F c) & G (F b)"""

    myBuchi = omega_FACTEST(ltl_formula=ltl_formula,env=env,model=model, workspace=workspace_poly)

    sample_run = myBuchi.exampleRun()
    print('sample run!',sample_run)

    init_poly = myBuchi.init_sets[sample_run[0]]
    print('init set!',init_poly.b)

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
    # plotPoly(workspace_poly,ax,'yellow')

    plotPoly(baby,ax,'green')
    plotPoly(adult,ax,'blue')
    plotPoly(charger,ax,'yellow')
    # plotPoly(init_set,ax,'purple')

    plotPoly(unsafe1,ax,'red')
    plotPoly(unsafe2,ax,'red')
    plotPoly(unsafe3,ax,'red')
    plotPoly(unsafe4,ax,'red')

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
