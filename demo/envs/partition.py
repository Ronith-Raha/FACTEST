import polytope as pc
import numpy as np

from math import sqrt

A = np.array([[-1, 0],
              [ 1, 0],
              [ 0,-1],
              [ 0, 1]])

b1 = np.array([-1.55, 1.9, 0.0, 2.0])
b2 = np.array([-1.0, 2.5, -2.6, 3.0])
b3 = np.array([0.0, 4.0, 0.1, 0.0])
b4 = np.array([0.0, 4.0, -4.9, 4.1])
b5 = np.array([-4.0, 4.1, 0.0, 4.0])
b6 = np.array([0.1, 0.0, 0.0, 4.0])

O1 = pc.Polytope(A, b1)
O2 = pc.Polytope(A, b2)
O3 = pc.Polytope(A, b3)
O4 = pc.Polytope(A, b4)
O5 = pc.Polytope(A, b5)
O6 = pc.Polytope(A, b6)

O = [O1,O2,O3,O4,O5,O6] # Border sets: O19,O20,O21,O22]

b_init = np.array([-0.5, 1.5, -1.5, 2.5])
Theta = pc.Polytope(A, b_init)

b_goal = np.array([-3.0, 3.675, -3.0, 3.675])
G = pc.Polytope(A, b_goal)

b_workspace = np.array([1.0, 5.0, 1.0, 5.0])
workspace = pc.Polytope(A, b_workspace)

if __name__=="__main__":
    import sys, os
    currFile = os.path.abspath(__file__)
    modelPath = currFile.replace('/demo/envs/maze_2d.py', '')
    sys.path.append(modelPath)

    import matplotlib.pyplot as plt

    from factest.plotting.plot_polytopes import plotPoly

    fig, ax = plt.subplots()

    plotPoly(workspace,ax,'yellow')

    plotPoly(G,ax,'green')
    plotPoly(Theta,ax,'blue')
    
    i = 1
    for obstacle in O:
        print('plotting poly #',i)
        plotPoly(obstacle,ax,'red')
        i+=1

    ax.set_xlim(-1,11)
    ax.set_ylim(-1,11)
    plt.show()