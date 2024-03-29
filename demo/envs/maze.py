import polytope as pc
import numpy as np

from math import sqrt

A = np.array([[-1, 0],
              [ 1, 0],
              [ 0,-1],
              [ 0, 1]])

b1 = np.array([-10, 30, -9, 10])*0.1
b2 = np.array([-10, 50, -39, 40])*0.1
b3 = np.array([-9, 10, -20, 30])*0.1
b4 = np.array([-10, 20, -29, 30])*0.1
b5 = np.array([-19, 20, -20, 30])*0.1
b6 = np.array([-20, 40, -19, 20])*0.1
b7 = np.array([-39, 40, -10, 30])*0.1
b8 = np.array([-29, 30, -30, 40])*0.1
b9 = np.array([-40, 60, -9, 10])*0.1
b10 = np.array([-49, 50, -20, 40])*0.1
b11 = np.array([-50, 60, -19, 20])*0.1
b13 = np.array([-49, 50, 0, 10])*0.1
b12 = np.array([-59, 60, -20, 50])*0.1
b14 = np.array([0, 70, 0, 1])*0.1
b15 = np.array([0, 1, 0, 30])*0.1
b16 = np.array([0, 1, -40, 50])*0.1
b17 = np.array([0, 60, -49, 50])*0.1
b18 = np.array([-69, 70, 0, 50])*0.1
b19 = np.array([1, 0, 0, 50])*0.1
b20 = np.array([-70, 71, 0, 50])*0.1
b21 = np.array([0, 70, 1, 0])*0.1
b22 = np.array([0, 70, -50, 51])*0.1

O1 = pc.Polytope(A, b1)
O2 = pc.Polytope(A, b2)
O3 = pc.Polytope(A, b3)
O4 = pc.Polytope(A, b4)
O5 = pc.Polytope(A, b5)
O6 = pc.Polytope(A, b6)
O7 = pc.Polytope(A, b7)
O8 = pc.Polytope(A, b8)
O9 = pc.Polytope(A, b9)
O10 = pc.Polytope(A, b10)
O11 = pc.Polytope(A, b11)
O12 = pc.Polytope(A, b12)
O13 = pc.Polytope(A, b13)
O14 = pc.Polytope(A, b14)
O15 = pc.Polytope(A, b15)
O16 = pc.Polytope(A, b16)
O17 = pc.Polytope(A, b17)
O18 = pc.Polytope(A, b18)
O19 = pc.Polytope(A, b19)
O20 = pc.Polytope(A, b20)
O21 = pc.Polytope(A, b21)
O22 = pc.Polytope(A, b22)

O = [O1, O2, O3, O4, O5, O6, O7, O8, O9, O10, O11, O12, O13, O14, O15, O16, O17, O18, O19, O20, O21, O22]

b_goal = np.array([-62.5, 67.5, -45, 50])*0.1
G = pc.Polytope(A, b_goal)

b_init = np.array([-(.5 - 0.2/sqrt(2)), .5 + 0.2/sqrt(2), -(3.5 - 0.2/sqrt(2)), 3.5 + 0.2/sqrt(2)])
Theta = pc.Polytope(A, b_init)

b_workspace = np.array([10,8,10,6])
workspace = pc.Polytope(A, b_workspace)

if __name__=="__main__":
    import sys, os
    currFile = os.path.abspath(__file__)
    modelPath = currFile.replace('/demo/envs/maze.py', '')
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
