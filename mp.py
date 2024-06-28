#User GUI to create chosen environment for FACTEST
import PySimpleGUI as sg 
import polytope as pc
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from math import sqrt

A = np.array([[-1, 0],
              [ 1, 0],
              [ 0,-1],
              [ 0, 1]])

global x_min, x_max, y_min, y_max, workspace, Theta, G

def create_plot():
    import sys, os
    currFile = os.path.abspath(__file__)
    modelPath = currFile.replace('/demo/envs/ronys_GUI.py', '')
    sys.path.append(modelPath)

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

    ax.set_xlim(x_min,x_max)
    ax.set_ylim(y_min,y_max)
    return fig, ax

def draw_figure(canvas, figure):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
    return figure_canvas_agg

layout =    [
            [sg.Text("  Enter the dimensions of the workspace as <x_min, x_max, y_min, y_max> : "), sg.Input(key="-WORK-")],
            [sg.Text("  Enter the starting position coordinates as <x_min, x_max, y_min, y_max> : "), sg.Input(key="-START-")],
            [sg.Text("  Enter the end goal coordinates as <x_min, x_max, y_min, y_max> : "), sg.Input(key="-END-")],
            [sg.Column([], key="-COL-")],
            [sg.Exit(), sg.Button("Add Obstacle"), sg.Button("Construct")]
            ,[sg.Canvas(key='-CANVAS-')]
            ]


window = sg.Window("2d Maze Creator", layout, finalize=True)
O = []
obstacles = []
fig, ax = plt.subplots()
canvas_elem = window['-CANVAS-']
canvas = canvas_elem.TKCanvas
figure_canvas_agg = draw_figure(canvas, fig)

draw_figure(window['-CANVAS-'].TKCanvas, create_plot())
while True:
    event, values = window.read()
    if event in (sg.WINDOW_CLOSED, "Exit"):
        break
    if event == "Add Obstacle":
        new_row = [sg.Text("Enter obstacle coordinates as <x_min, x_max, y_min, y_max> within the workspace: "), sg.Input(key="-IN-")]
        window.extend_layout(window["-COL-"], [new_row])
    if event == "Construct":
        obstacle_values = [list(map(float, value.split(','))) for key, value in values.items() if key.startswith("-IN-")]
        obstacles.extend(obstacle_values)
        for i in range(len(obstacles)):
            for j in range(len(obstacles[i])):
                if j%2 == 0:
                    obstacles[i][j] *= -1
            obstacle_array = np.array(obstacles[i])
            O.append(pc.Polytope(A, obstacle_array))

        work_values = [list(map(float, value.split(','))) for key, value in values.items() if key.startswith("-WORK-")]
        start_values = [list(map(float, value.split(','))) for key, value in values.items() if key.startswith("-START-")]
        end_values = [list(map(float, value.split(','))) for key, value in values.items() if key.startswith("-END-")]
        x_min = work_values[0][0]
        x_max = work_values[0][1]
        y_min = work_values[0][2]
        y_max = work_values[0][3]
        for q in range(len(start_values[0])):
            if q%2 == 0:
                work_values[0][q] *= -1
                start_values[0][q] *= -1
                end_values[0][q] *= -1
        work_array = np.array(work_values[0])
        start_array = np.array(start_values[0])
        end_array = np.array(end_values[0])
        workspace = pc.Polytope(A, work_array)
        Theta = pc.Polytope(A, start_array)
        G = pc.Polytope(A, end_array)
        ax.clear()
        fig = create_plot()
        figure_canvas_agg.get_tk_widget().pack_forget()
        figure_canvas_agg = draw_figure(canvas, fig)

window.close()