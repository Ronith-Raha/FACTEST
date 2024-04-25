import os
import torch
import torch.nn.functional as F
import numpy as np
import time
import importlib
from tqdm import tqdm
# from mayavi import mlab
from matplotlib import pyplot as plt
import multiprocessing
from multiprocessing import Pool
from dynamic_dubins_car import ref_traj
from dynamic_dubins_car import ref_input
from dynamic_dubins_car import dynamics
import sys
sys.path.append('systems')
sys.path.append('.')

from model import get_model
from model_dryvr import get_model as get_model_dryvr

def mute():
    sys.stdout = open(os.devnull, 'w')

import argparse

parser = argparse.ArgumentParser(description="")
parser.add_argument('--system', type=str,
                        default='dubin')
parser.add_argument('--no_cuda', dest='use_cuda', action='store_false')
parser.set_defaults(use_cuda=True)
parser.add_argument('--layer1', type=int, default=64)
parser.add_argument('--layer2', type=int, default=64)
parser.add_argument('--pretrained', type=str)
parser.add_argument('--no_plot', type=str)
parser.add_argument('--seed', type=int, default=1024)

args = parser.parse_args()
np.random.seed(args.seed)

config = importlib.import_module('system_'+args.system)
use_dryvr = 'dryvr' in args.pretrained
if use_dryvr:
    model, forward = get_model_dryvr(len(config.sample_X0())+1, config.simulate(config.get_init_center(config.sample_X0())).shape[1]-1)
else:
    model, forward = get_model(len(config.sample_X0())+1, config.simulate(config.get_init_center(config.sample_X0())).shape[1]-1, config, args)
    if args.use_cuda:
        model = model.cuda()
    else:
        model = model.cpu()

model.load_state_dict(torch.load(args.pretrained)['state_dict'])

def ellipsoid_surface_2D(P):
    K = 100
    thetas = np.linspace(0, 2 * np.pi, K)
    points = []
    for i, theta in enumerate(thetas):
        point = np.array([np.cos(theta), np.sin(theta)])
        points.append(point)
    points = np.array(points)
    sub_P = P[:2,:2]
    points = np.linalg.inv(sub_P).dot(points.T)
    return points[0,:], points[1,:]

lower = np.array([-10, -10, np.pi/4, 0, np.pi/6])
higher = np.array([10, 10, np.pi/4, 5, np.pi/6])

X0_center_range = np.array([lower, higher]).T
X0_r_max = 0.5

center = X0_center_range[:,0] + np.random.rand(X0_center_range.shape[0]) * (X0_center_range[:,1]-X0_center_range[:,0])
r = np.random.rand()*X0_r_max
X0 = np.concatenate([center, np.array(r).reshape(-1)])

print(X0)

ref = config.simulate(config.get_init_center(X0))

benchmark_name = args.system

reachsets = []

X0_mean, X0_std = config.get_X0_normalization_factor()
X0 = (X0 - X0_mean) / X0_std

for idx_t in range(1, ref.shape[0]):
    s = time.time()
    tmp = torch.tensor(X0.tolist()+[ref[idx_t, 0],]).view(1,-1).float()
    if args.use_cuda:
        tmp = tmp.cuda()
    P = forward(tmp)
    e = time.time()
    P = P.squeeze(0)
    reachsets.append([ref[idx_t, 1:], P.cpu().detach().numpy()])
T = 30
dt =0.1


ref_trajectory =[]
t_p = np.arange(0,T+dt, dt)
act_pos =[]

t = np.array(ref[:,0])
tt =np.array(ref[1:,0])
for i in t_p:
    #ref_state1 = ref_traj(t)
    #ref_input1 = ref_input(t)
    #ref_p = dynamics(ref_state1, t, ref_input1)
    ref_p = ref_traj(i)
    ref_trajectory.append(ref_p)

pos_mag = np.linalg.norm(ref[:,1:3],axis =1)
 

#ref_trajectory = np.array(ref_trajectory)

SMALL_SIZE = 8
MEDIUM_SIZE = 10
BIGGER_SIZE = 13
HUGE_SIZE = 25

#plt.rc('font', size=BIGGER_SIZE)          # controls default text sizes
#plt.rc('axes', titlesize=HUGE_SIZE)     # fontsize of the axes title
#plt.rc('axes', labelsize=15)    # fontsize of the x and y labels
#plt.rc('xtick', labelsize=15)    # fontsize of the tick labels3
#plt.rc('ytick', labelsize=15)    # fontsize of the tick labels
#plt.rc('legend', fontsize=10)    # legend fontsize
#plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title
#plt.rc('axes', axisbelow=True)


plt.figure(figsize=(10,10))

plt.plot(t, pos_mag, label='magnitudes of position')

#plt.plot(ref[:,1],ref[:,2], 'k-', label='Actual Trajectory')
#plt.plot(ref_trajectory[0], ref_trajectory[1], 'b-', label = 'Reference Trajectory')

for reachset in reachsets:
    center = reachset[0]
    x,y = ellipsoid_surface_2D(reachset[1])
    pos = np.sqrt(x**2+y**2)
    plt.plot(tt+center[0],pos+center[1], 'r-', alpha =0.5)
   

    
plt.xlabel('X Coordinates')
plt.ylabel('Y Coordinates')
plt.title('Trajectories in X-Y Space')
plt.legend(loc='best')
plt.grid(True)
# plt.show()
plt.savefig('dubin.pdf')
