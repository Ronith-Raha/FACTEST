import polytope as pc
import numpy as np

from math import sqrt

A_rect = np.array([[-1, 0],
                   [ 1, 0],
                   [ 0,-1],
                   [ 0, 1]])
A_tri1 = np.array([[-1, -1],
                   [ 1, -1],
                   [ 0, 1]])
A_tri2 = np.array([[-1, 1],
                   [ 1, 1],
                   [ 0, 1]])

b1 = np.array([5,20,0])*0.1
b2 = np.array([-10,35,0])*0.1
b3 = np.array([-30,0,30])*0.1
b4 = np.array([-15,-15,30])*0.1
b5 = np.array([-45,15,30])*0.1
b6 = np.array([15,50,1,0])*0.1
b7 = np.array([15,50,-30,31])*0.1
b8 = np.array([16,-15,0,30])*0.1
b9 = np.array([-50,51,0,30])*0.1

O1 = pc.Polytope(A_tri2, b1)
O2 = pc.Polytope(A_tri2, b2)
O3 = pc.Polytope(A_tri1, b3)
O4 = pc.Polytope(A_tri1, b4)
O5 = pc.Polytope(A_tri1, b5)
O6 = pc.Polytope(A_rect, b6)
O7 = pc.Polytope(A_rect, b7)
O8 = pc.Polytope(A_rect, b8)
O9 = pc.Polytope(A_rect, b9)

O = [O1,O2,O3,O4,O5,O6,O7,O8,O9]

b_init = np.array([7.5+2/sqrt(2), -(7.5-2/sqrt(2)), -(7.5-2/sqrt(2)), 7.5+2/sqrt(2)])*0.1
Theta = pc.Polytope(A_rect, b_init)

b_goal = np.array([-40,45,-10,15])*0.1
G = pc.Polytope(A_rect, b_goal)

b_workspace = np.array([1.5, 5.0, 0.0, 3.0])
workspace = pc.Polytope(A_rect, b_workspace)

if __name__=="__main__":
    import sys, os
    currFile = os.path.abspath(__file__)
    modelPath = currFile.replace('/demo/envs/zigzag1.py', '')
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
        print(obstacle)
        plotPoly(obstacle,ax,'red')
        i+=1
    plt.fill("j", "k", facecolor='red', alpha=0.25, 
         data={"j": [0.0, 1.5, 3.0], 
               "k": [3.0, 1.5, 3.0]})  # here 'm' for magenta
    
    ax.set_xlim(-1,11)
    ax.set_ylim(-1,11)
    plt.show()