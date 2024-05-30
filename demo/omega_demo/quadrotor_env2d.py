#######################################
# Import required libraries and files #
#######################################

## Set up file paths ##
#######################
import sys, os
currFile = os.path.abspath(__file__)
modelPath = currFile.replace('/demo/omega_demo/quadrotor_env2d.py', '')
sys.path.append(modelPath)
factestPath = currFile.replace('/demo/omega_demo/quadrotor_env2d.py', '/factest/synthesis')
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

    # P0
    x0_min, x0_max = [0,1.5]
    y0_min, y0_max = [-1,1]
    b0 = np.array([-x0_min, x0_max, -y0_min, y0_max])

    # P1
    x1_min, x1_max = [1.501,3.499]
    y1_min, y1_max = [-1,1]
    b1 = np.array([-x1_min, x1_max, -y1_min, y1_max])

    # P2
    x2_min, x2_max = [3.5,5]
    y2_min, y2_max = [-1,1]
    b2 = np.array([-x2_min, x2_max, -y2_min, y2_max])

    # P3
    x3_min, x3_max = [1.5,1.501]
    y3_min, y3_max = [-1,1]
    b3 = np.array([-x3_min, x3_max, -y3_min, y3_max])

    # P4
    x4_min, x4_max = [3.499,3.5]
    y4_min, y4_max = [-1,1]
    b4 = np.array([-x4_min, x4_max, -y4_min, y4_max])



    b_workspace = np.array([0.5,15.5,0.5,11])
    # b_workspace2 = np.array([-0.5,5,0,5])

    poly0 = pc.Polytope(A,b0)
    poly1 = pc.Polytope(A,b1)
    poly2 = pc.Polytope(A,b2)
    poly3 = pc.Polytope(A,b3)
    poly4 = pc.Polytope(A,b4)

    workspace_poly = pc.Polytope(A,b_workspace)
    # workspace_poly2 = pc.Polytope(A,b_workspace2)

    # env = {'P1':pickup_1,'P2':pickup_2,'D1':delivery_1,'D2':delivery_2,'C':charger,'O1':obstacle_poly_1,'O2':obstacle_poly_2}   

    # ltl_formula = '(G !O1 & G !O2) & G (F P1) & G (F P2) & G (P1 -> ((!C & !P2 & !D2) U D1)) & G (P2 -> ((!C & !P1 & !D1) U D2))'

    # myBuchi = omega_FACTEST(ltl_formula=ltl_formula,env=env,model=model, workspace=workspace_poly)

    # sample_run = myBuchi.exampleRun()
    # print(sample_run)

    # init_poly = myBuchi.init_sets[sample_run[0]]
    # print(init_poly.b)

    # min_x = init_poly.b[0]*-1
    # min_y = init_poly.b[2]*-1
    # max_x = init_poly.b[1]
    # max_y = init_poly.b[3]

    # curr_state = [6,6,0]
    # rand_init = [np.random.random()*(max_x-min_x)+min_x, np.random.random()*(max_y-min_y)+min_y,0]

    # sim_states = model.simulate_run(rand_init, sample_run, myBuchi.flow_cache, myBuchi.init_sets)

    # xsim = [state[0] for state in sim_states]
    # ysim = [state[1] for state in sim_states]

    fig = plt.figure()
    
    ax = fig.add_subplot(111)
    # plotPoly(workspace_poly,ax,'yellow')

    plotPoly(poly0,ax,'red')
    plotPoly(poly1,ax,'orange')
    plotPoly(poly2,ax,'yellow')
    plotPoly(poly3,ax,'green')
    plotPoly(poly4,ax,'blue')

    # for init_key in myBuchi.init_sets.keys():
    #     init_poly = myBuchi.init_sets[init_key]
    #     plotPoly(init_poly,ax,'green')

    # ax.plot(rand_init[0], rand_init[1], marker = '*',markersize=5)
    # ax.plot(xsim,ysim, linestyle = '--')

    ax.set_xlim(-1,16)
    ax.set_ylim(-3,13)
    ax.xaxis.set_tick_params(labelbottom=False)
    ax.yaxis.set_tick_params(labelleft=False)

    plt.show()