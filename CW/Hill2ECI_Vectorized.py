# Import libraries
import numpy as np
import torch

# Import functions
from ECI2RSW import ECI2RSW

def Hill2ECI_Vectorized(rTgt=None,vTgt=None,rHill=None,vHill=None,*args,**kwargs):
    ## Purpose:
    # Convert those position (rHill) and velocity (vHill) values back into an
    # ECI coordinate frame of reference using the reference satellite
    # (rTgt,vTgt) position and velocity data.
    
    ## Inputs:
    #rTgt                       [3 x N]                 ECI Position vector of
    #                                                   reference frame (km)
    
    #vTgt                       [3 x N]                 ECI Velocity vector of
    #                                                   reference frame (km/s)
    
    #rHill                      [3 x N]                 Hill's relative
    #                                                   position vector (km)
    
    #vHill                      [3 x N]                 Hill's relative
    #                                                   velocity vector (km/s)
    
    
    
    ## Outputs:
    #rInt                       [3 x N]
    
    #vInt                       [3 x N]
    
    ## Begin Code Sequence
    #Declare Local Functions
    rTgtMag=torch.sqrt(torch.sum(rTgt ** 2,0))
    vTgtMag=torch.sqrt(torch.sum(vTgt ** 2,0))
    
    def matrixMultiply(x,y):
        return torch.vstack([torch.sum(x[0, :, :].permute(1,0) * y.permute(1, 0), 1), torch.sum(x[1, :, :].permute(1,0) * y.permute(1, 0), 1), torch.sum(x[2, :, :].permute(1,0) * y.permute(1, 0), 1)])
    
    #Find the RSW matrix from the target ECI positions
    RSW=ECI2RSW(rTgt,vTgt)
    rIntMag=rTgtMag + rHill[0, :]

    #Compute rotation angles to go from tgt to interceptor
    lambda_int=rHill[1, :] / rTgtMag
    phi_int=torch.sin(rHill[2, :] / rTgtMag)
    
    CLI=torch.cos(lambda_int)
    SLI=torch.sin(lambda_int)
    CPI=torch.cos(phi_int)
    SPI=torch.sin(phi_int)
    
    #find rotation matrix to go from rsw to SEZ of inerceptor
    RSW_SEZ=torch.zeros(3,3, rTgt.size(dim = 1))
    RSW_SEZ[0,0,:]=SPI * CLI
    RSW_SEZ[0,1,:]=SPI * SLI
    RSW_SEZ[0,2,:]=- CPI
    RSW_SEZ[1,0,:]=- SLI
    RSW_SEZ[1,1,:]=CLI
    RSW_SEZ[2,0,:]=CPI * CLI
    RSW_SEZ[2,1,:]=CPI * SLI
    RSW_SEZ[2,2,:]=SPI
    
    #Find velocity component positions by using angular rates in SEZ frame
    vIntSEZ=torch.vstack([- rIntMag * vHill[2,:] / rTgtMag, rIntMag * (vHill[1,:] / rTgtMag + vTgtMag / rTgtMag) * CPI, vHill[0, :]])
    vInt=matrixMultiply(RSW.permute(1,0,2),matrixMultiply(RSW_SEZ.permute(1,0,2),vIntSEZ))
    
    #Find the position components
    rIntRSW=rIntMag * torch.vstack([CPI * CLI, CPI * SLI, SPI])
    rInt=matrixMultiply(RSW.permute(1,0,2),rIntRSW)

    return rInt,vInt
    
if __name__ == '__main__':
    pass
    