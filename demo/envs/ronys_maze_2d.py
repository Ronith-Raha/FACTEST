import polytope as pc
import numpy as np

from math import sqrt

A = np.array([[-1, 0],
              [ 1, 0],
              [ 0,-1],
              [ 0, 1]])

b1 = np.array([-2,10,-1.9,2])
b2 = np.array([-3.9,4,-2,3])
b3 = np.array([-2,3,-2.9,3])
b4 = np.array([-6,7,-2.9,3])
b5 = np.array([-8,9,-2.9,3])
b6 = np.array([-1.9,2,-3,10])
b7 = np.array([-2.9,3,-3,4])
b8 = np.array([-4.9,5,-3,6])
b9 = np.array([-5.9,6,-3,4])
b10 = np.array([-6.9,7,-3,5])
b11 = np.array([-7.9,8,-3,4])
b13 = np.array([-8.9,9,-3,5])
b12 = np.array([-9.9,10,-2,10])
b14 = np.array([3,4,-3.9,4])
b15 = np.array([-6,8,-4.9,5])
b16 = np.array([-2.9,3,-5,6])
b17 = np.array([-3.9,4,-5,6])
b18 = np.array([-7.9,8,-5,6])
b19 = np.array([-3,5,-5.9,6])
b20 = np.array([-6,10,-5.9,6])
b21 = np.array([-5.9,6,-6,8])
b22 = np.array([-3,5,-6.9,7])
b23 = np.array([-5,6,-6.9,7])
b24 = np.array([-8,9,-6.9,7])
b25 = np.array([-4.9,5,-7,8])
b26 = np.array([-7.9,8,-7,9])
b27 = np.array([-8.9,9,-7,9])
b28 = np.array([-3,4,-7.9,8])
b29 = np.array([-5,6,-7.9,8])
b30 = np.array([-7,8,-7.9,8])
b31 = np.array([-2.9,3,-8,9])
b32 = np.array([-3.9,4,-8,9])
b33 = np.array([-6.9,7,-8,9])
b34 = np.array([-4,7,-8.9,9])
b35 = np.array([-2,9,-9.9,10])




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
O23 = pc.Polytope(A, b23)
O24 = pc.Polytope(A, b24)
O25 = pc.Polytope(A, b25)
O26 = pc.Polytope(A, b26)
O27 = pc.Polytope(A, b27)
O28 = pc.Polytope(A, b28)
O29 = pc.Polytope(A, b29)
O30 = pc.Polytope(A, b30)
O31 = pc.Polytope(A, b31)
O32 = pc.Polytope(A, b32)
O33 = pc.Polytope(A, b33)
O33 = pc.Polytope(A, b34)
O35 = pc.Polytope(A, b35)

O = [O1,O2,O3,O4,O5,O6,O7,O8,O9,O10,O11,O12,O13,O14,O15,O16,O17,O18,O19,O20,O21,O22,O23,O24,O25,O26,O27,O28,O29,O30,O31,O32,O33,O34,O35]

b_init = np.array([-1.4,1.6,-2.4,2.6])
Theta = pc.Polytope(A, b_init)

b_goal = np.array([-9.25, 9.75, -9.5, 10.0])
G = pc.Polytope(A, b_goal)

b_workspace = np.array([-1,10,-1,10])
workspace = pc.Polytope(A, b_workspace)

if __name__=="__main__":
    import sys, os
    currFile = os.path.abspath(__file__)
    modelPath = currFile.replace('/demo/envs/ronys_maze_2d.py', '')
    sys.path.append(modelPath)

    import matplotlib.pyplot as plt

    from factest.plotting.plot_polytopes import plotPoly

    fig, ax = plt.subplots()

    plotPoly(workspace,ax,'yellow')

    #plotPoly(G,ax,'green')
    #plotPoly(Theta,ax,'blue')
    
    i = 1
    for obstacle in O:
        print('plotting poly #',i)
        plotPoly(obstacle,ax,'red')
        i+=1

    ax.set_xlim(-1,13)
    ax.set_ylim(-1,13)
    plt.show()
