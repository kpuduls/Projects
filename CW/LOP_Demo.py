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
os.environ['KMP_DUPLICATE_LIB_OK']='True'

# Declared Constants:
##################################  

# Gravity const
mu=torch.tensor(398600.4418)
# Earth rad
earthRad=torch.tensor(6378.1363)
# Initial Circular Radius of Space Shuttle
r_int=torch.tensor(500)

# Create initial coordinates representing the target
#####################################################

xt=earthRad + r_int
yt=torch.tensor(0)
zt=torch.tensor(0)
xdott=torch.tensor(0)
ydott= torch.sqrt(mu / (earthRad + r_int))
zdott=torch.tensor(0)
rT=torch.tensor([[xt],[yt],[zt]])
vT=torch.tensor([[xdott],[ydott],[zdott]])

# Add inclination to the orbit in deg
inc=- torch.tensor(51.65)

RYM=torch.tensor([[torch.cos(inc*(np.pi/180)),0,torch.sin(inc*(np.pi/180))],
              [0,1,0],
              [- torch.sin(inc*(np.pi/180)),0,torch.cos(inc*(np.pi/180))]])

rT=torch.matmul(RYM,rT)

vT=torch.matmul(RYM,vT)

# Create initial coordinates representing the chaser satellite
# Initial Positions and Velocities of the chaser from the target
#################################################################
# All units are in km/s
# Assume that both chaser and target are docked initially    
xi=rT[0]
yi=rT[1]
zi=rT[2]
xdoti=vT[0]
ydoti=vT[1]
zdoti=vT[2]

rI=torch.tensor([[xi],[yi],[zi]])
vI=torch.tensor([[xdoti],[ydoti],[zdoti]])

#Specify the time interval of interest in seconds
t=torch.tensor(range(0,40*60,60))

#Convert the chaser satellite ECI coordinates into the Hill frame of
#reference
rHill,vHill=ECI2Hill_Vectorized(rT,vT,rI,vI,nargout=2)
print("first func")
#Add initial displacement on the x vector
rHill[0]=rHill[0] + 1

#Now ... recompute the chaser satellite vector ...
rI,vI=Hill2ECI_Vectorized(rT,vT,rHill,vHill,nargout=2)
print("second func")
#Find the angular rate of the target orbit
omega=torch.sqrt(mu / torch.sqrt(torch.sum(rT ** 2)) ** 3)

#Use the Linear Propagator (LOP) to propagate Hill's coordinates forward in time
#This is to find the relative orbital motion of the "Chaser" satellite
rHill,vHill=CWHPropagator(rHill,vHill,omega,t,nargout=2)
print("third func")
#Use a nonlinear propagator to determine the Target tragectory as well as
#the Chaser tragectory

rTgt,vTgt=keplerUniversal(rT.repeat(1,t.size(0)),vT.repeat(1,t.size(0)),t,mu,nargout=2)
rChase,vChase=keplerUniversal(rI.repeat(1,t.size(0)),vI.repeat(1,t.size(0)),t,mu,nargout=2)

print("kepler funcs")
#Now ... Convert the propagated Hill results back to an ECI reference
#frame:
rCL,vCL=Hill2ECI_Vectorized(rTgt,vTgt,rHill,vHill,nargout=2)

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

trace2 = go.Scatter3d(x = rCL[0,:], y = rCL[1, :], z = rCL[2, :])
trace2.update()

data = go.Data([trace1, trace2])

fig = go.Figure(data = data)

fig.write_html('first_figure.html', auto_open=True)