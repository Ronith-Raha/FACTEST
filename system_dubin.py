import sys
sys.path.append('..')
 
from dynamic_dubins_car import run_simulation
import numpy as np
from scipy.integrate import odeint
import scipy as scipy
import scipy.special as sc
import random

TMAX = 50
dt = 0.1

# range of initial states

lower = np.array([-10, -10, 0, 0, 0])
higher = np.array([10, 10, 0, 5, 0])


X0_center_range = np.array([lower, higher]).T
X0_r_max = 0.5

    
def sample_X0():
    center = X0_center_range[:,0] + np.random.rand(X0_center_range.shape[0]) * (X0_center_range[:,1]-X0_center_range[:,0])
    r = np.random.rand()*X0_r_max
    X0 = np.concatenate([center, np.array(r).reshape(-1)])
    return X0

def sample_t():
    return (np.random.randint(int(TMAX/dt))+1) * dt

def sample_x0(X0):
    center = X0[:-1]
    r = X0[-1]

    n = len(center)
    direction = np.random.randn(n)
    direction = direction / np.linalg.norm(direction)

    x0 = center + direction * r
    x0[x0>X0_center_range[:,1]] = X0_center_range[x0>X0_center_range[:,1],1]
    x0[x0<X0_center_range[:,0]] = X0_center_range[x0<X0_center_range[:,0],0]
    return x0

def sample_x0_uniform(X0):
    center = X0[:-1]
    r = X0[-1]

    n = len(center)
    direction = np.random.randn(n)
    direction = direction / np.linalg.norm(direction)

    dist = np.random.rand()
    x0 = center + direction * dist * r
    x0[x0>X0_center_range[:,1]] = X0_center_range[x0>X0_center_range[:,1],1]
    x0[x0<X0_center_range[:,0]] = X0_center_range[x0<X0_center_range[:,0],0]
    return x0

def simulate(x0):
    return np.array(run_simulation(x0, sample_t(),dt))

def get_init_center(X0):
    center = X0[:-1]
    return center

def get_X0_normalization_factor():
    mean = np.zeros(len(sample_X0()))
    std = np.ones(len(sample_X0()))
    return [mean, std]
