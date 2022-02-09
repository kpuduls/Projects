# Import libraries
import numpy as np
import torch

def ECI2RSW(rECI,vECI,*args,**kwargs):

    ## Purpose:
    #Convert ECI Coordinates to RSW Coordinates, also, return the
    #transformation matrix T in which to take a given set of coordinates in ECI
    #and convert them using the same RSW reference frame.
    
    ## Inputs:
    # rECI              [3 x N]                     ECI position Coordinates in
    #                                               km
    
    # vECI              [3 x N]                     ECI velocity Coordinates in
    #                                               km/s
    
    ## Outputs:
    # T                 [3 x 3 x N]                 Transformation matrix
    #                                               necessary to go from
    #                                               rECI -> rRSW
    
    # rRSW              [3 x N]                     RSW Position Coordinates
    #                                               km
    
    # vRSW              [3 x N]                     RSW Velocity Coordinates
    #                                               km/s
    
    ## Begin Code Sequence
    def unitv(x):
        return x/torch.sqrt(torch.sum(x**2, axis = 0))
        
    def matrixMultiply(x,y):
        return torch.vstack([torch.sum(x[0, :, :].permute(1,0) * y.permute(1, 0), 1), torch.sum(x[1, :, :].permute(1,0) * y.permute(1, 0), 1), torch.sum(x[2, :, :].permute(1,0) * y.permute(1, 0), 1)])
        
        
    #Find the Radial component of the RIC position vector
    rvec=unitv(rECI)
    
    #Find the cross-track component of the RIC position vector
    wvec=unitv(torch.cross(rECI.T,vECI.T).T)

    #Find the along-track component of the RIC position vector
    svec=unitv(torch.cross(wvec.T,rvec.T).T)

    #Create the transformation matrix from ECI to RSW
    T = torch.empty((3, 3, rECI.size(dim = 1)))
    T[:, :, :] = np.nan
    
    # Assign values to transformation matrix
    T[0,0,:]=rvec[0,:]
    T[0,1,:]=rvec[1,:]
    T[0,2,:]=rvec[2,:]
    T[1,0,:]=svec[0,:]
    T[1,1,:]=svec[1,:]
    T[1,2,:]=svec[2,:]
    T[2,0,:]=wvec[0,:]
    T[2,1,:]=wvec[1,:]
    T[2,2,:]=wvec[2,:]
    
    #Find the position and velocity vectors in the RSW reference frame!
    rRSW=matrixMultiply(T,rECI)
    vRSW=matrixMultiply(T,vECI)

    return T
    
if __name__ == '__main__':
    pass
    