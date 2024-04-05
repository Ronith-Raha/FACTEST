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

def ellipsoid_radii(P_tensor):
    P = P_tensor.detach().numpy()
    eigenvalues = np.linalg.eigvals(P[:2,:2])
    radii = np.sqrt(eigenvalues)
    return max(radii), min(radii)

lower = np.array([-10, -10, 0, 0, 0])
higher = np.array([10, 10, 0, 5, 0])

X0_center_range = np.array([lower, higher]).T
X0_r_max = 0.5

center = X0_center_range[:,0] + np.random.rand(X0_center_range.shape[0]) * (X0_center_range[:,1]-X0_center_range[:,0])
r = np.random.rand()*X0_r_max
X0 = np.concatenate([center, np.array(r).reshape(-1)])

print(X0)

ref = config.simulate(config.get_init_center(X0))

benchmark_name = args.system

reachsets = []
times1 = []
radii_major = []
radii_minor = []
X0_mean, X0_std = config.get_X0_normalization_factor()
X0 = (X0 - X0_mean) / X0_std

for idx_t in range(1, ref.shape[0]):
    s = time.time()
    tmp = torch.tensor(X0.tolist()+[ref[idx_t, 0],]).view(1,-1).float()
    if args.use_cuda:
        tmp = tmp.cuda()
    P_tensor = forward(tmp)
    e = time.time()
    P_tensor = P_tensor.squeeze(0)
    reachsets.append([ref[idx_t, 1:], P_tensor])

    major, minor = ellipsoid_radii(P_tensor)
    radii_major.append(major)
    radii_minor.append(minor)
    times1.append(ref[idx_t, 0])

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


plt.figure(figsize=(10,5))

plt.plot(times1,radii_major, label='Semi-major Axis')
#plt.plot(times1, radii_minor, label='Semi-minor Axis')
    
plt.xlabel('Time')
plt.ylabel('Ellispsoid Radius')
plt.title('Ellipsoid Radii Over Time')
plt.legend()
plt.grid(True)
# plt.show()
plt.savefig('dubin_radii.pdf')
