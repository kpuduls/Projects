# Import libraries
import numpy as np
import torch
import matplotlib.pyplot
import pandas

# Import functions
from CWHPropagator import CWHPropagator
from ECI2Hill_Vectorized import ECI2Hill_Vectorized
from ECI2RSW import ECI2RSW
from Hill2ECI_Vectorized import Hill2ECI_Vectorized

# Define the target position and velocity vectors in time in Hills reference frame
t=torch.tensor(range(0,4*60,60))
rTgt0  = torch.tensor([[0], [0], [0]])
vTgt0 = torch.tesnor([[0], [0], [0]])

#Now ... Convert the propagated Hill results back to an ECI reference
#frame:
rCL,vCL=Hill2ECI_Vectorized(rTgt,vTgt,rHill,vHill,nargout=2)

