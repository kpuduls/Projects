# Import libraries
import numpy as np
import torch

# Import functions

    
def CWHPropagator(rHillInit=None,vHillInit=None,omega=None,t=None,*args,**kwargs):
    ## Purpose:
    # Take initial position and velocity coordinates in the Hill reference frame
    # and propagate them using the Clohessy-Wiltshire Hill Linearize equation
    # of motion.
    
    ## Inputs:
    #rHillInit                  [3 x 1]                 Hill Position vector
    #                                                   (km) / (m)
    
    #vHillInit                  [3 x 1]                 Hill Velocity vector of
    #                                                   (km/s) / (m/s)
    
    #omega                       double                 Orbital Angular Rate
    #                                                   of the target
    #                                                   (rad/s)
    #                                                   Should be close to
    #                                                   circular for linear propagation
    #                                                   error to be low.
    
    #t                          [1 x N]                 Propagation Time in
    #                                                   seconds
    #
    
    
    
    ## Outputs:
    #rHill                       [3 x N]                Propagated Hill
    #                                                   Position vector (km) /
    #                                                   (m/s)
    
    #vHill                       [3 x N]                Propagated Hill
    #                                                   Velocity vector (km/s)
    #                                                   / (m/s)
    
    

    ## Begin Code Sequence
    x0=rHillInit[0,:]
    y0=rHillInit[1,:]
    z0=rHillInit[2,:]
    x0dot=vHillInit[0,:]
    y0dot=vHillInit[1,:]
    z0dot=vHillInit[2,:]
    
    rHill=torch.vstack([(x0dot / omega)* torch.sin(omega * t) - (torch.tensor(3) * x0 + torch.tensor(2) * y0dot / omega) * torch.cos(omega * t) + (torch.tensor(4) * x0 + torch.tensor(2) * y0dot / omega), (torch.tensor(6) *x0 + torch.tensor(4) * y0dot / omega) * torch.sin(omega * t) + torch.tensor(2) * (x0dot / omega) * torch.cos(omega *t) - (torch.tensor(6) * omega * x0 + torch.tensor(3) * y0dot) * t + (y0 - torch.tensor(2) * x0dot / omega), z0 * torch.cos(omega *t) + (z0dot / omega) * torch.sin(omega * t)])
    vHill=torch.vstack([x0dot * torch.cos(omega * t) + (torch.tensor(3) * omega * x0 + torch.tensor(2) * y0dot) * torch.sin(omega * t), (torch.tensor(6) * omega * x0 + torch.tensor(4) * y0dot) * torch.cos(omega * t) - torch.tensor(2) * x0dot * torch. sin(omega * t) - (torch.tensor(6) * omega * x0 + torch.tensor(3) * y0dot), - z0 * omega * torch.sin(omega * t) + z0dot * torch.cos(omega * t)])
    
    return rHill,vHill
    
if __name__ == '__main__':
    pass
    