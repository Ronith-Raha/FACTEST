#######################################
# Import required libraries and files #
#######################################

## Set up file paths ##
#######################
import sys, os
currFile = os.path.abspath(__file__)
modelPath = currFile.replace('/demo/omega_demo/puzzle2.py', '')
sys.path.append(modelPath)
factestPath = currFile.replace('/demo/omega_demo/puzzle2.py', '/factest/synthesis')
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

    # dist: 0.12649110640673517
    # dist: 0.282842712474619

    A = np.array([[-1,0],[1,0],[0,-1],[0,1]])

    # KEY 1
    x1_min, x1_max = [3,5]
    y1_min, y1_max = [10,12]
    b1 = np.array([-x1_min, x1_max, -y1_min, y1_max])

    # KEY 2
    x2_min, x2_max = [13,15]
    y2_min, y2_max = [10,12]
    b2 = np.array([-x2_min, x2_max, -y2_min, y2_max])

    # DOOR 1
    x3_min, x3_max = [5,7]
    y3_min, y3_max = [3.5,5.5]
    b3 = np.array([-x3_min, x3_max, -y3_min, y3_max])

    # DOOR 2
    x4_min, x4_max = [13,15]
    y4_min, y4_max = [1,3]
    b4 = np.array([-x4_min, x4_max, -y4_min, y4_max])

    # OBSTACLES
    x5_min, x5_max = [0,4.75]
    y5_min, y5_max = [4,5]
    b5 = np.array([-x5_min, x5_max, -y5_min, y5_max])

    x6_min, x6_max = [7.3,15]
    y6_min, y6_max = [4,5]
    b6 = np.array([-x6_min, x6_max, -y6_min, y6_max])

    x7_min, x7_max = [0,10]
    y7_min, y7_max = [8,9]
    b7 = np.array([-x7_min, x7_max, -y7_min, y7_max])

    x8_min, x8_max = [13,15]
    y8_min, y8_max = [8,9]
    b8 = np.array([-x8_min, x8_max, -y8_min, y8_max])

    b_workspace = np.array([0,15,0,15])
    # b_workspace2 = np.array([-0.5,5,0,5])

    obstacle_poly_1 = pc.Polytope(A,b5)
    obstacle_poly_2 = pc.Polytope(A,b6)
    obstacle_poly_3 = pc.Polytope(A,b7)
    obstacle_poly_4 = pc.Polytope(A,b8)

    key_1 = pc.Polytope(A,b1)
    key_2 = pc.Polytope(A,b2)

    door_1 = pc.Polytope(A,b3)
    door_2 = pc.Polytope(A,b4)

    workspace_poly = pc.Polytope(A,b_workspace)

    env = {'K1':key_1,'K2':key_2,'D1':door_1,'D2':door_2,'O1':obstacle_poly_1,'O2':obstacle_poly_2,'O3':obstacle_poly_3,'O4':obstacle_poly_4}   

    ltl_formula = 'K1 & (G !O1 & G !O2 & G !O3 & G !O4) & G (F D1) & G (F D2) & G (K1 -> ((!D2) U K2)) & G (K2 -> ((!D1) U K1))'

    myBuchi = omega_FACTEST(ltl_formula=ltl_formula,env=env,model=model, workspace=workspace_poly)

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

    plotPoly(key_1,ax,'green')
    plotPoly(key_2,ax,'green')

    plotPoly(door_1,ax,'blue')
    plotPoly(door_2,ax,'blue')

    # plotPoly(charger,ax,'yellow')

    plotPoly(obstacle_poly_1,ax,'red')
    plotPoly(obstacle_poly_2,ax,'red')
    plotPoly(obstacle_poly_3,ax,'red')
    plotPoly(obstacle_poly_4,ax,'red')

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