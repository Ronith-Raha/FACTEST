import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import polytope as pc

# Define the constraints matrix A
A = np.array([
    [-1, 0],
    [1, 0],
    [0, -1],
    [0, 1]
])

# Define the workspace boundaries
workspace_min = 0
workspace_max = 10

# Define the vertices for the initial polytope
initial_vertices = np.array([[1, 1], [3, 1], [3, 3], [1, 3]])  # [xmin, xmax, ymin, ymax]
initial_poly = pc.Polytope(A, initial_vertices.flatten())

# Define the vertices for the goal polytope
goal_vertices = np.array([[7, 7], [9, 7], [9, 9], [7, 9]])  # [xmin, xmax, ymin, ymax]
goal_poly = pc.Polytope(A, goal_vertices.flatten())

# Define the vertices for the obstacle polytopes
obstacle1_vertices = np.array([[4, 4], [6, 4], [6, 6], [4, 6]])  # Example obstacle 1
obstacle2_vertices = np.array([[2, 7], [3, 7], [3, 9], [2, 9]])  # Example obstacle 2
obstacle1_poly = pc.Polytope(A, obstacle1_vertices.flatten())
obstacle2_poly = pc.Polytope(A, obstacle2_vertices.flatten())

# Plotting the workspace and polytopes
fig, ax = plt.subplots()
ax.set_xlim([workspace_min, workspace_max])
ax.set_ylim([workspace_min, workspace_max])
ax.set_aspect('equal')

# Plot workspace boundaries
workspace_boundary = [[workspace_min, workspace_min], [workspace_max, workspace_min],
                      [workspace_max, workspace_max], [workspace_min, workspace_max]]
workspace_poly = Polygon(workspace_boundary, alpha=0.1, edgecolor='black', facecolor='white')
ax.add_patch(workspace_poly)

# Plot initial polytope (initial state)
initial_poly_patch = Polygon(initial_vertices, alpha=0.5, label='Initial Polytope', edgecolor='blue', facecolor='blue')
ax.add_patch(initial_poly_patch)

# Plot goal polytope (goal state)
goal_poly_patch = Polygon(goal_vertices, alpha=0.5, label='Goal Polytope', edgecolor='green', facecolor='green')
ax.add_patch(goal_poly_patch)

# Plot obstacle polytopes
obstacle1_poly_patch = Polygon(obstacle1_vertices, alpha=0.5, label='Obstacle 1', edgecolor='red', facecolor='red')
ax.add_patch(obstacle1_poly_patch)

obstacle2_poly_patch = Polygon(obstacle2_vertices, alpha=0.5, label='Obstacle 2', edgecolor='orange', facecolor='orange')
ax.add_patch(obstacle2_poly_patch)

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('New Scenario: Disjoint Polytopes in a Workspace')

# Add legend
ax.legend()

# Show plot
plt.show()

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import polytope as pc

# Define the constraints matrix A
A = np.array([
    [-1, 0],
    [1, 0],
    [0, -1],
    [0, 1]
])

# Define the workspace boundaries
workspace_min = 0
workspace_max = 10

# Define the vertices for the initial polytope
initial_vertices = np.array([[0, 2], [2, 2], [2, 4], [0, 4]])  # [xmin, xmax, ymin, ymax]
initial_poly = pc.Polytope(A, initial_vertices.flatten())

# Define the vertices for the goal polytope
goal_vertices = np.array([[7, 7], [9, 7], [9, 9], [7, 9]])  # [xmin, xmax, ymin, ymax]
goal_poly = pc.Polytope(A, goal_vertices.flatten())

# Define the vertices for the obstacle polytopes
obstacle1_vertices = np.array([[4, 4], [5, 4], [5, 5], [4, 5]])  # Example obstacle 1
obstacle2_vertices = np.array([[2, 6], [3, 6], [3, 8], [2, 8]])  # Example obstacle 2
obstacle1_poly = pc.Polytope(A, obstacle1_vertices.flatten())
obstacle2_poly = pc.Polytope(A, obstacle2_vertices.flatten())

# Plotting the workspace and polytopes
fig, ax = plt.subplots()
ax.set_xlim([workspace_min, workspace_max])
ax.set_ylim([workspace_min, workspace_max])
ax.set_aspect('equal')

# Plot workspace boundaries
workspace_boundary = [[workspace_min, workspace_min], [workspace_max, workspace_min],
                      [workspace_max, workspace_max], [workspace_min, workspace_max]]
workspace_poly = Polygon(workspace_boundary, alpha=0.1, edgecolor='black', facecolor='white')
ax.add_patch(workspace_poly)

# Plot initial polytope (initial state)
initial_poly_patch = Polygon(initial_vertices, alpha=0.5, label='Initial Polytope', edgecolor='blue', facecolor='blue')
ax.add_patch(initial_poly_patch)

# Plot goal polytope (goal state)
goal_poly_patch = Polygon(goal_vertices, alpha=0.5, label='Goal Polytope', edgecolor='green', facecolor='green')
ax.add_patch(goal_poly_patch)

# Plot obstacle polytopes
obstacle1_poly_patch = Polygon(obstacle1_vertices, alpha=0.5, label='Obstacle 1', edgecolor='red', facecolor='red')
ax.add_patch(obstacle1_poly_patch)

obstacle2_poly_patch = Polygon(obstacle2_vertices, alpha=0.5, label='Obstacle 2', edgecolor='orange', facecolor='orange')
ax.add_patch(obstacle2_poly_patch)

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Disjoint Polytopes in a Workspace')

# Add legend
ax.legend()

# Define the constraints matrix A
A = np.array([
    [-1, 0],
    [1, 0],
    [0, -1],
    [0, 1]
])

# Define the workspace boundaries
workspace_min = 0
workspace_max = 10

# Define the vertices for the initial polytope
initial_vertices = np.array([[1, 1], [3, 1], [3, 3], [1, 3]])  # [xmin, xmax, ymin, ymax]
initial_poly = pc.Polytope(A, initial_vertices.flatten())

# Define the vertices for the goal polytope
goal_vertices = np.array([[7, 7], [9, 7], [9, 9], [7, 9]])  # [xmin, xmax, ymin, ymax]
goal_poly = pc.Polytope(A, goal_vertices.flatten())

# Define the vertices for the obstacle polytopes
obstacle1_vertices = np.array([[4, 4], [6, 4], [6, 6], [4, 6]])  # Example obstacle 1
obstacle2_vertices = np.array([[2, 7], [3, 7], [3, 9], [2, 9]])  # Example obstacle 2
obstacle1_poly = pc.Polytope(A, obstacle1_vertices.flatten())
obstacle2_poly = pc.Polytope(A, obstacle2_vertices.flatten())

# Plotting the workspace and polytopes
fig, ax = plt.subplots()
ax.set_xlim([workspace_min, workspace_max])
ax.set_ylim([workspace_min, workspace_max])
ax.set_aspect('equal')

# Plot workspace boundaries
workspace_boundary = [[workspace_min, workspace_min], [workspace_max, workspace_min],
                      [workspace_max, workspace_max], [workspace_min, workspace_max]]
workspace_poly = Polygon(workspace_boundary, alpha=0.1, edgecolor='black', facecolor='white')
ax.add_patch(workspace_poly)

# Plot initial polytope (initial state)
initial_poly_patch = Polygon(initial_vertices, alpha=0.5, label='Initial Polytope', edgecolor='blue', facecolor='blue')
ax.add_patch(initial_poly_patch)

# Plot goal polytope (goal state)
goal_poly_patch = Polygon(goal_vertices, alpha=0.5, label='Goal Polytope', edgecolor='green', facecolor='green')
ax.add_patch(goal_poly_patch)

# Plot obstacle polytopes
obstacle1_poly_patch = Polygon(obstacle1_vertices, alpha=0.5, label='Obstacle 1', edgecolor='red', facecolor='red')
ax.add_patch(obstacle1_poly_patch)

obstacle2_poly_patch = Polygon(obstacle2_vertices, alpha=0.5, label='Obstacle 2', edgecolor='orange', facecolor='orange')
ax.add_patch(obstacle2_poly_patch)

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('New Scenario: Disjoint Polytopes in a Workspace')

# Add legend
ax.legend()

# Show plot
plt.show()
