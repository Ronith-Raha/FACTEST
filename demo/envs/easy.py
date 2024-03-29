import polytope as pc
import numpy as np

from math import sqrt

A = np.array([[-1, 0],
              [ 1, 0],
              [ 0,-1],
              [ 0, 1]])

b1 = np.array([-4,6,2,0])
b2 = np.array([-2,4,0,2])
b3 = np.array([0,2,-2,4])

O1 = pc.Polytope(A, b1)
O2 = pc.Polytope(A, b2)
O3 = pc.Polytope(A, b3)

O = [O1, O2, O3]

b_goal = np.array([-4,5,-4,5])
G = pc.Polytope(A, b_goal)

b_init = np.array([0,1,0,1])
Theta = pc.Polytope(A, b_init)

b_workspace = np.array([1,10,1,10])
workspace = pc.Polytope(A, b_workspace)

if __name__=="__main__":
    import sys, os
    currFile = os.path.abspath(__file__)
    modelPath = currFile.replace('/demo/envs/easy.py', '')
    sys.path.append(modelPath)

    import matplotlib.pyplot as plt

    from factest.plotting.plot_polytopes import plotPoly

    fig, ax = plt.subplots()

    plotPoly(workspace,ax,'yellow')

    plotPoly(G,ax,'green')
    plotPoly(Theta,ax,'blue')
    for obstacle in O:
        plotPoly(obstacle,ax,'red')

    ax.set_xlim(-1,11)
    ax.set_ylim(-1,11)
    plt.show()
