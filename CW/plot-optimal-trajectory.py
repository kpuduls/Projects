# Import functions
from CWHPropagator import CWHPropagator
from ECI2Hill_Vectorized import ECI2Hill_Vectorized
from ECI2RSW import ECI2RSW
from Hill2ECI_Vectorized import Hill2ECI_Vectorized
from keplerUniversal import keplerUniversal
from MILPfunc import optimize
import plotly.graph_objects as go

# Library imports
import numpy as np
import torch
import matplotlib.pyplot as plt

import os

os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'

# Declared Constants:
##################################

# Gravity const
mu = torch.tensor(398600.4418)
# Earth rad
earthRad = torch.tensor(6378.1363)
# Initial Circular Radius of Space Shuttle
h = 800
r_int = torch.tensor(h)

# Specify spacecraft mass
ms = 250

# Create initial coordinates representing the target
#####################################################
xt = earthRad + r_int
yt = torch.tensor(0)
zt = torch.tensor(0)
xdott = torch.tensor(0)
ydott = torch.sqrt(mu / (earthRad + r_int))
zdott = torch.tensor(0)
rT = torch.tensor([[xt], [yt], [zt]])
vT = torch.tensor([[xdott], [ydott], [zdott]])

# Add inclination to the orbit in deg
inc = - torch.tensor(51.65)

RYM = torch.tensor([[torch.cos(inc * (np.pi / 180)), 0, torch.sin(inc * (np.pi / 180))],
                    [0, 1, 0],
                    [- torch.sin(inc * (np.pi / 180)), 0, torch.cos(inc * (np.pi / 180))]])

rT = torch.matmul(RYM, rT)
vT = torch.matmul(RYM, vT)

# Specify the prediction horizon length
N = 160

# Specify sampling time
T_s = 100

# Specify the time interval of interest in seconds
t = torch.tensor(range(0, N * T_s, T_s))

# Specify the chaser initial state and desired final state
x0 = [[0], [0], [0] ,[0] ,[0] ,[4]]
xdes = [[0], [0], [0], [0], [0], [0]]

# Use the Linear Propagator (LOP) to propagate Hill's coordinates forward in time
# This is to find the relative orbital motion of the "Chaser" satellite
rHill, vHill, xstar, ustar = optimize(x0 = x0, xdes = xdes, T_s = T_s, N = N, h = h, ms = ms)

rHill = torch.from_numpy(rHill)
vHill = torch.from_numpy(vHill)

print("third func")
# Use a nonlinear propagator to determine the Target tragectory as well as
# the Chaser tragectory

rTgt, vTgt = keplerUniversal(rT.repeat(1, t.size(0)), vT.repeat(1, t.size(0)), t, mu, nargout=2)

print("kepler funcs")
# Now ... Convert the propagated Hill results back to an ECI reference
# frame:
rCL, vCL = Hill2ECI_Vectorized(rTgt, vTgt, rHill, vHill, nargout=2)

print("func end")

r = earthRad

# Set up 100 points. First, do angles
theta = np.linspace(0, 2 * np.pi, 100)
phi = np.linspace(0, np.pi, 100)

# Set up coordinates for points on the sphere
x0 = r * np.outer(np.cos(theta), np.sin(phi))
y0 = r * np.outer(np.sin(theta), np.sin(phi))
z0 = r * np.outer(np.ones(100), np.cos(phi))

# Set up traces
trace1 = go.Surface(x=x0, y=y0, z=z0)
trace1.update(showscale=False)

trace2 = go.Scatter3d(x=rCL[0, :], y=rCL[1, :], z=rCL[2, :], marker=dict(color='LightSkyBlue', size=3))
trace2.update()

data = go.Data([trace1, trace2])

fig = go.Figure(data=data)

fig.write_html('first_figure.html', auto_open=True)