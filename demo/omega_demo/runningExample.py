#######################################
# Import required libraries and files #
#######################################

## Set up file paths ##
#######################
import sys, os
currFile = os.path.abspath(__file__)
modelPath = currFile.replace('/demo/omega_demo/runningExample.py', '')
sys.path.append(modelPath)
factestPath = currFile.replace('/demo/omega_demo/runningExample.py', '/factest/synthesis')
sys.path.append(factestPath)

## Import Python libraries ##
#############################
import numpy as np
import matplotlib.pyplot as plt
import polytope as pc

from math import pi

## Import FACTEST files ##
##########################
from factest.synthesis.factest_base_z3 import FACTEST_Z3
from factest.synthesis.factest_base_gurobi import FACTEST_gurobi
# from factest.synthesis.omega_factest_z3 import hybrid_from_ltl
# from models.dubins_plane import dubins_plane
from demo.models.dubins_car import dubins_car
# from models.dynamic_dubins_car import dynamic_planar_dubins
# #TODO: Need to make a 3d testing file

## Import plotting stuff ##
###########################
from factest.plotting.plot_polytopes import plotPoly


## Setting up OmegaThreads autonomous vehicle example ##
########################################################

A = np.array([[-1,0],[1,0],[0,-1],[0,1]])
b_obstacle = np.array([[-1.5,6,-1.5,3.5]])
b_1 = np.array([-3.7,4.8,-0.2,1.3])
b_2 = np.array([-3.7,4.8,-3.7,4.8])
b_workspace = np.array([0,5,0,5])
b_workspace2 = np.array([-0.5,5,0,5])


obstacle_poly = pc.Polytope(A,b_obstacle)
goal_poly_1 = pc.Polytope(A,b_1)
goal_poly_2 = pc.Polytope(A,b_2)

workspace_poly = pc.Polytope(A,b_workspace)
workspace_poly2 = pc.Polytope(A,b_workspace2)



## Running single reach-avoid ##
################################
model = dubins_car()
FACTEST_prob = FACTEST_Z3(goal_poly_1, goal_poly_2, [obstacle_poly], workspace=workspace_poly, model=model)
result_dict = FACTEST_prob.run()
result_keys = list(result_dict.keys())

FACTEST_prob2 = FACTEST_Z3(goal_poly_2, goal_poly_1, [obstacle_poly], workspace=workspace_poly2, model=model)
result_dict2 = FACTEST_prob2.run()
result_keys2 = list(result_dict2.keys())

currPlot = '3'

if currPlot == '1':
    ## Getting trajectories for the first Figure 1 ##
    #################################################

    rand_init = [np.random.random()*(4.5-3.75)+3.8, np.random.random()*(0.75-0.25)+0.2]

    for result_id in result_keys:
        if result_dict[result_id]['poly'].__contains__(np.array(rand_init).T):
            break

    xref = result_dict[result_id]['xref']


    model.set_ref(xref,1)

    initial_state = [rand_init[0],rand_init[1],np.random.random()*pi/2 + pi/2]
    T = 0
    for point_idx in range(len(xref)-1):
        print(xref[point_idx], xref[point_idx+1])
        T += np.linalg.norm(np.array(xref[point_idx+1]) - np.array(xref[point_idx]))


    states = model.run_simulation(xref, initial_state, T, vref = 1, sim_type = "base")

    x_sim = [state[0] for state in states]
    y_sim = [state[1] for state in states]

    for result_id in result_keys2:
        if result_dict2[result_id]['poly'].__contains__(np.array([x_sim[-1], y_sim[-1]])):
            break


    xref2 = result_dict2[3]['xref']

    model.set_ref(xref2,1)
    initial_state = [x_sim[-1],y_sim[-1],states[-1][2]]

    T = 0
    for point_idx in range(len(xref2)-1):
        print(xref2[point_idx], xref2[point_idx+1])
        T += np.linalg.norm(np.array(xref2[point_idx+1]) - np.array(xref2[point_idx]))


    states2 = model.run_simulation(xref2, initial_state, T, vref = 1, sim_type = "base")

    x_sim2 = [state[0] for state in states2]
    y_sim2 = [state[1] for state in states2]


    xref3 = result_dict[1]['xref']

    model.set_ref(xref3,1)
    initial_state = [x_sim2[-1],y_sim2[-1],states2[-1][2]]

    T = 0
    for point_idx in range(len(xref3)-1):
        print(xref3[point_idx], xref3[point_idx+1])
        T += np.linalg.norm(np.array(xref3[point_idx+1]) - np.array(xref3[point_idx]))


    states3 = model.run_simulation(xref3, initial_state, T, vref = 1, sim_type = "base")

    x_sim3 = [state[0] for state in states3]
    y_sim3 = [state[1] for state in states3]

    xval_ref_1 = [xval[0] for xval in xref]
    yval_ref_1 = [xval[1] for xval in xref]

    xval_ref_2 = [xval[0] for xval in xref2]
    yval_ref_2 = [xval[1] for xval in xref2]

    xval_ref_3 = [xval[0] for xval in xref3]
    yval_ref_3 = [xval[1] for xval in xref3]


    fig, ax = plt.subplots(figsize=(6,6))
    plotPoly(obstacle_poly,ax,'red')
    plotPoly(goal_poly_1,ax,'blue')
    plotPoly(goal_poly_2,ax,'green')
    ax.annotate(r'$O$', xy=(3, 2.5))
    ax.annotate(r'$G_1$', xy=(4.5, 0.5))
    ax.annotate(r'$G_2$', xy=(4.5, 4.5))

    ax.plot([rand_init[0]], [rand_init[1]], color = 'teal', marker = '*',markersize=15)
    ax.plot(x_sim, y_sim, color='teal', alpha=0.6, linestyle = '--')
    ax.plot(x_sim2, y_sim2, color='purple', alpha=0.75, linestyle = '--')
    ax.plot(x_sim2[0], y_sim2[0], color='purple', marker = 'o')
    ax.plot(x_sim3[:200], y_sim3[:200], color='teal', linestyle = '--')
    ax.plot(x_sim3[0], y_sim3[0], color='teal', marker = 'o')

    plt.arrow(x_sim3[200], y_sim3[200], x_sim3[205] - x_sim3[200], y_sim3[205] - y_sim3[200], head_width=0.1, color='teal')

    ax.set_xlim(0,5)
    ax.set_ylim(0,5)

    plt.xticks([])
    plt.yticks([])

    plt.show()

elif currPlot == '2':
    ## Constucting Figure 3 ##
    ##########################

    xref = result_dict[1]['xref']
    xref2 = result_dict2[3]['xref']

    xval_ref_1 = [xval[0] for xval in xref]
    yval_ref_1 = [xval[1] for xval in xref]

    xval_ref_2 = [xval[0] for xval in xref2]
    yval_ref_2 = [xval[1] for xval in xref2]


    # RESULT 1

    fig1, ax1 = plt.subplots(figsize=(6,6))
    fig2, ax2 = plt.subplots(figsize=(6,6))

    ax1.set_xticks([])
    ax2.set_xticks([])

    ax1.set_yticks([])
    ax2.set_yticks([])

    plotPoly(obstacle_poly,ax1,'red')
    plotPoly(goal_poly_2,ax1,'green')
    


    times = np.arange(0,1,0.01)
    colors = ['thistle','plum','violet']
    for i in range(len(xref) - 1):
        rad = model.errBound(result_dict[1]['poly'], i)
        x1 = xref[i][0]
        y1 = xref[i][1]
        x2 = xref[i+1][0]
        y2 = xref[i+1][1]

        for t in times:
            ax1.add_patch(plt.Circle((x1 + t*(x2 - x1), y1 + (y2 - y1)*t), rad, color=colors[i]))

    ax1.add_patch(plt.Circle((x2, y2), rad, facecolor='salmon', edgecolor='black'))

    for key in result_keys:
        plotPoly(result_dict[key]['poly'],ax1,'blue')

    
    ax1.plot(xval_ref_1, yval_ref_1, color='black', marker = 'o')
    ax1.plot(xval_ref_1[-1], yval_ref_1[-1], color='black', marker = '*', markersize=15)

    T = 0
    for point_idx in range(len(xref)-1):
        T += np.linalg.norm(np.array(xref[point_idx+1]) - np.array(xref[point_idx]))

    for i in range(1,10):
        initial_state = [np.random.random()*(4.8-4.25) + 4.25, np.random.random()*(1.3 - 0.75) + 0.75, np.random.random()*2*pi - pi]
        states = model.run_simulation(xref, initial_state, T, vref = 1, sim_type = "base")

        x_sim = [state[0] for state in states]
        y_sim = [state[1] for state in states]

        ax1.plot(x_sim, y_sim, color='teal', linestyle = ':', alpha = 0.6)

    
    ax1.annotate(r'$O$', xy=(3, 2.5))
    ax1.annotate(r'$G_1$', xy=(4.5, 0.5))
    ax1.annotate(r'$G_2$', xy=(4.55, 4.6))

    ax1.set_xlim(0,5)
    ax1.set_ylim(0,5)

    # RESULT 2

    plotPoly(obstacle_poly,ax2,'red')
    plotPoly(goal_poly_1,ax2,'blue')

    for i in range(len(xref2) - 1):
        rad = model.errBound(result_dict2[3]['poly'], i)
        x1 = xref2[i][0]
        y1 = xref2[i][1]
        x2 = xref2[i+1][0]
        y2 = xref2[i+1][1]

        for t in times:
            ax2.add_patch(plt.Circle((x1 + t*(x2 - x1), y1 + (y2 - y1)*t), rad, color=colors[i]))

    ax2.add_patch(plt.Circle((x2, y2), rad, facecolor='salmon', edgecolor='black'))

    for key2 in result_keys2:
        plotPoly(result_dict2[key2]['poly'],ax2,'green')


    T = 0
    for point_idx in range(len(xref2)-1):
        T += np.linalg.norm(np.array(xref2[point_idx+1]) - np.array(xref2[point_idx]))

    for i in range(1,10):
        initial_state = [np.random.random()*(4.8-4.25) + 4.25, np.random.random()*(4.25 - 3.7) + 3.7, np.random.random()*2*pi - pi]
        states = model.run_simulation(xref2, initial_state, T, vref = 1, sim_type = "base")

        x_sim = [state[0] for state in states]
        y_sim = [state[1] for state in states]

        ax2.plot(x_sim, y_sim, color='purple', linestyle = ':', alpha = 0.6)


    ax2.plot(xval_ref_2, yval_ref_2, color='black', marker = 'o')
    ax2.plot(xval_ref_2[-1], yval_ref_2[-1], color='black', marker = '*', markersize=15)

    ax2.annotate(r'$O$', xy=(3, 2.5))
    ax2.annotate(r'$G_1$', xy=(4.55, 0.4))
    ax2.annotate(r'$G_2$', xy=(4.5, 4.5))

    ax2.set_xlim(0,5)
    ax2.set_ylim(0,5)

    plt.show()

else:
    rand_init = [np.random.random()*(4.5-3.75)+3.8, np.random.random()*(0.75-0.25)+0.2]

    for result_id in result_keys:
        if result_dict[result_id]['poly'].__contains__(np.array(rand_init).T):
            break

    xref = result_dict[result_id]['xref']


    model.set_ref(xref,1)

    initial_state = [rand_init[0],rand_init[1],np.random.random()*pi/2 + pi/2]
    T = 0
    for point_idx in range(len(xref)-1):
        T += np.linalg.norm(np.array(xref[point_idx+1]) - np.array(xref[point_idx]))


    states = model.run_simulation(xref, initial_state, T, vref = 1, sim_type = "base")

    x_sim = [state[0] for state in states]
    y_sim = [state[1] for state in states]

    for result_id in result_keys2:
        if result_dict2[result_id]['poly'].__contains__(np.array([x_sim[-1], y_sim[-1]])):
            break


    xref2 = result_dict2[3]['xref']

    model.set_ref(xref2,1)
    initial_state = [x_sim[-1],y_sim[-1],states[-1][2]]

    T = 0
    for point_idx in range(len(xref2)-1):
        print(xref2[point_idx], xref2[point_idx+1])
        T += np.linalg.norm(np.array(xref2[point_idx+1]) - np.array(xref2[point_idx]))


    states2 = model.run_simulation(xref2, initial_state, T, vref = 1, sim_type = "base")

    x_sim2 = [state[0] for state in states2]
    y_sim2 = [state[1] for state in states2]

    xref3 = result_dict[1]['xref']

    initial_state = [x_sim2[-1],y_sim2[-1],states2[-1][2]]

    T = 0
    for point_idx in range(len(xref3)-1):
        T += np.linalg.norm(np.array(xref3[point_idx+1]) - np.array(xref3[point_idx]))


    states3 = model.run_simulation(xref3, initial_state, T, vref = 1, sim_type = "base")

    x_sim3 = [state[0] for state in states3]
    y_sim3 = [state[1] for state in states3]



    initial_state = [x_sim3[-1],y_sim3[-1],states3[-1][2]]

    T = 0
    for point_idx in range(len(xref2)-1):
        T += np.linalg.norm(np.array(xref2[point_idx+1]) - np.array(xref2[point_idx]))


    states4 = model.run_simulation(xref2, initial_state, T, vref = 1, sim_type = "base")

    x_sim4 = [state[0] for state in states4]
    y_sim4 = [state[1] for state in states4]

    fig, ax = plt.subplots(figsize=(6,6))

    rad = model.errBound(result_dict[1]['poly'], 3)
    ax.add_patch(plt.Circle((xref3[-1][0], xref3[-1][1]), rad, facecolor='silver', edgecolor='black'))
    
    rad = model.errBound(result_dict2[3]['poly'], 3)
    ax.add_patch(plt.Circle((xref2[-1][0], xref2[-1][1]), rad, facecolor='silver', edgecolor='black'))


    plotPoly(obstacle_poly,ax,'red')

    for key in result_keys:
        plotPoly(result_dict[key]['poly'],ax,'blue')

    for key2 in result_keys2:
        plotPoly(result_dict2[key2]['poly'],ax,'green')

    ax.annotate(r'$O$', xy=(3, 2.5))
    ax.annotate(r'$G_1$', xy=(4.55, 0.4))
    ax.annotate(r'$G_2$', xy=(4.55, 4.6))

    ax.plot([rand_init[0]], [rand_init[1]], color = 'navy', marker = '*',markersize=15)

    ax.plot(x_sim, y_sim, color='navy', alpha=0.6, linestyle = '--')

    ax.plot(x_sim2, y_sim2, color='crimson', alpha=0.5, linestyle = '--')
    ax.plot(x_sim2[0], y_sim2[0], color='crimson', marker = 'o')

    ax.plot(x_sim3, y_sim3, color='navy', alpha = 0.6, linestyle = '--')
    ax.plot(x_sim3[0], y_sim3[0], color='navy', marker = 'o')

    ax.plot(x_sim4, y_sim4, color='crimson', alpha = 0.5, linestyle = '--')
    ax.plot(x_sim4[0], y_sim4[0], color='crimson', marker = 'o')


    ax.set_xlim(0,5)
    ax.set_ylim(0,5)

    plt.xticks([])
    plt.yticks([])

    plt.show()