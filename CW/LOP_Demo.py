# Import functions
from CWHPropagator import CWHPropagator
from ECI2Hill_Vectorized import ECI2Hill_Vectorized
from ECI2RSW import ECI2RSW
from Hill2ECI_Vectorized import Hill2ECI_Vectorized
from keplerUniversal import keplerUniversal

# Library imports
import numpy as np
import torch
import matplotlib.pyplot as plt
    


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
#Compare Results from using CW propagation to the non-linear propagation!
plt.rcParams["figure.figsize"] = [7.00, 3.50]
plt.rcParams["figure.autolayout"] = True
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
r = earthRad
u, v = np.mgrid[0:2 * np.pi:30j, 0:np.pi:20j]
x = np.cos(u) * np.sin(v)
y = np.sin(u) * np.sin(v)
z = np.cos(v)
#ax.plot_surface(x, y, z, cmap=plt.cm.YlGnBu_r)
ax.plot3D(rCL[0,:], rCL[1,:], rCL[2,:], 'gray')
plt.show()
