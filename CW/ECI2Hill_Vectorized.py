# Import libraries
import numpy as np
import torch

# Import functions
from ECI2RSW import ECI2RSW
    
def ECI2Hill_Vectorized(rTgt=None,vTgt=None,rChase=None,vChase=None,*args,**kwargs):
    ## Purpose:
    # Convert those position (ECI) and velocity (ECI) into Hill's reference
    # frame using both the target and the chaser position/velocity data
    
    ## Inputs:
    #rTgt                       [3 x N]                 ECI Position vector of
    #                                                   reference frame (km)
    
    #vTgt                       [3 x N]                 ECI Velocity vector of
    #                                                   reference frame (km/s)
    #rChase                     [3 x N]
    
    #vChase                     [3 x N]
    
    ## Outputs:
    #rHill                      [3 x N]                 Hill's relative
    #                                                   position vector (km)
    
    #vHill                      [3 x N]                 Hill's relative
    #                                                   velocity vector (km/s)

    ## Begin Code Sequence
    #Declare Local Functions
    def matrixMultiply(x,y):
        return torch.vstack([torch.sum(x[0, :, :].permute(1,0) * y.permute(1, 0), 1), torch.sum(x[1, :, :].permute(1,0) * y.permute(1, 0), 1), torch.sum(x[2, :, :].permute(1,0) * y.permute(1, 0), 1)])

    rTgtMag=torch.sqrt(torch.sum(rTgt ** 2,0))
    rChaseMag=torch.sqrt(torch.sum(rChase ** 2,0))
    vTgtMag=torch.sqrt(torch.sum(vTgt ** 2,0))
    #Determine the RSW transformation matrix from the target frame of reference
    RSW=ECI2RSW(rTgt,vTgt)

    #Use RSW rotation matrix to convert rChase and vChase to RSW
    r_Chase_RSW=matrixMultiply(RSW,rChase)
    v_Chase_RSW=matrixMultiply(RSW,vChase)

    #Find Rotation angles to go from target to interceptor
    phi_chase=torch.asin(r_Chase_RSW[2,:] / rChaseMag)
    lambda_chase=torch.atan2(r_Chase_RSW[1,:],r_Chase_RSW[0,:])
    
    CPC=torch.cos(phi_chase)
    SPC=torch.sin(phi_chase)
    SLC=torch.sin(lambda_chase)
    CLC=torch.cos(lambda_chase)

    #Find Position component rotations
    rHill=torch.vstack([rChaseMag - rTgtMag, lambda_chase * rTgtMag, phi_chase * rTgtMag])

    #Find the rotation matrix RSW->SEZ of chaser
    RSW_SEZ=torch.zeros(3,3, rTgtMag.size(dim = 0))
    
    RSW_SEZ[0,0,:]=SPC * CLC
    RSW_SEZ[0,1,:]=SPC * SLC
    RSW_SEZ[0,2,:]=- CPC
    RSW_SEZ[1,0,:]=- SLC
    RSW_SEZ[1,1,:]=CLC
    RSW_SEZ[2,0,:]=CPC * CLC
    RSW_SEZ[2,1,:]=CPC * SLC
    RSW_SEZ[2,2,:]=SPC

    #Find the velocity component of positions using the angular rates in SEZ frame
    v_Chase_SEZ=matrixMultiply(RSW_SEZ,v_Chase_RSW)
    vHill= torch.vstack([v_Chase_SEZ[2,:], rTgtMag * (v_Chase_SEZ[1,:] / (rChaseMag * CPC) - vTgtMag / rTgtMag), - rTgtMag * v_Chase_SEZ[0, :] / rChaseMag])
    
    return rHill,vHill
    
if __name__ == '__main__':
    pass
    