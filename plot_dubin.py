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
    points = np.linalg.inv(P).dot(points.T)
    return points[0,:], points[1,:]

lower = np.array([-10, -10, 0, 0, 0])
higher = np.array([10, 10, 0, 5, 0])

c = (lower+higher)/2

X0_r_max = np.array([10,10,0,1,0])
r =[10,10,0,1,0]
X0 = np.array(c.tolist()+r)
print(X0)

ref = config.simulate(config.get_init_center(X0))
sampled_inits = [config.sample_x0(X0) for _ in range(100)]
num_proc = min([1, multiprocessing.cpu_count()-3])
#with Pool(num_proc, initializer=mute) as p:
#    sampled_trajs = list(p.imap(config.simulate, sampled_inits))
sampled_trajs = list(map(config.simulate, sampled_inits))

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

SMALL_SIZE = 8
MEDIUM_SIZE = 10
BIGGER_SIZE = 13
HUGE_SIZE = 25

plt.rc('font', size=BIGGER_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=HUGE_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=15)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=15)    # fontsize of the tick labels
plt.rc('ytick', labelsize=15)    # fontsize of the tick labels
plt.rc('legend', fontsize=10)    # legend fontsize
plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title
plt.rc('axes', axisbelow=True)


plt.figure(figsize=(50,50))

plt.plot(ref[:,1],ref[:,2], 'k-', label='Reference Trajectory')
for reachset in reachsets:
    center = reachset[0]
    x,y = ellipsoid_surface_2D(reachset[1])
    plt.plot(x+center[0],y+center[1], 'r-', alpha=0.5)

for sampled_traj in sampled_trajs:
    plt.plot(sampled_traj[:,1], sampled_traj[:,2], color=(0,0,1), alpha = 0.1, label ='Sampled Trajectories')
    
plt.xlabel('X Coordinates')
plt.ylabel('Y Coordinates')
plt.title('Trajectories in X-Y Space')
plot.legend(loc='best')
plt.grid(True)
# plt.show()
plt.savefig('dubin.pdf')
