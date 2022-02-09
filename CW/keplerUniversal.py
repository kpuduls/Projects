# Import libraries
import numpy as np
import torch

# Import functions

def keplerUniversal(r0=None,v0=None,t=None,mu=None,*args,**kwargs):
    #Purpose:
    #Most effecient way to propagate any type of two body orbit using kepler's
    #equations.
    #-------------------------------------------------------------------------#
    #                                                                         #
    # Inputs:                                                                 #
    #--------                                                                  
    #r_ECI                  [3 x N]                         Position Vector in
    #                                                       ECI coordinate
    #                                                       frame of reference
        
    #v_ECI                  [3 x N]                         Velocity vector in
    #                                                       ECI coordinate
    #                                                       frame of reference
        
    #t                      [1 x N]                         time vector in
    #                                                       seconds
        
    #mu                     double                          Gravitational Constant
    #                                                       Defaults to Earth if
    #                                                       not specified
    # Outputs:
    #---------                                                                
    #r_ECI                  [3 x N]                         Final position
    #                                                       vector in ECI
    
    #v_ECI                  [3 x N]                         Final velocity

    tol=torch.tensor(100)
    v0Mag=torch.sqrt(torch.sum(v0 ** 2,0))
    r0Mag=torch.sqrt(torch.sum(r0 ** 2,0))
    alpha=- (v0Mag ** 2) / mu + torch.tensor(2) / r0Mag

    ## Compute initial guess (X0) for Newton's Method
    X0 = torch.empty(t.size(0))
    X0[:] = np.nan
    
    #Check if there are any Eliptic/Circular orbits
    idx=alpha    #> 1e-06
    
    for i in range(idx.size(0)):
        if (idx[i] > 1e-06):
            X0[i]=torch.sqrt(mu) * t[i] * alpha[i]
            print("eliptic/Circular Orbit detected")

    
    #Check if there are any Parabolic orbits
    for i in range(idx.size(0)):
        if (idx[i] < 1e-06):
            h=torch.cross(r0[:,idx],v0[:,idx])
            hMag=torch.sqrt(torch.sum(h ** 2,0))
            p=(hMag ** 2) / mu
            s=torch.atan(np.pi - torch.tensor(3) * torch.sqrt(mu / (p ** 3)) *t[idx]) / torch.tensor(2)
            w=torch.atan(torch.tan(s) ** (1 / 3))
            X0[idx]=torch.sqrt(p) * torch.tensor(2) / torch.tan(torch.tensor(2) * w)
            print("Parabolic Orbit detected")
    
    #Check if there are any Hyperbolic orbits
    for i in range(idx.size(0)):
        if (idx[i] < - 1e-06):
            a=1.0 / alpha[idx]
            X0[i]=multiply(multiply(sign(t(i)),sqrt(- a)),log(multiply(multiply(dot(- 2.0,mu),alpha(i)),t(i)) / (dot(r0(arange(),i),v0(arange(),idx)) + multiply(multiply(sign(t(idx)),sqrt(multiply(- mu,a))),(1 - multiply(r0Mag(idx),alpha(idx)))))))
            print("Hyperbolic Orbit detected")
            
    ## Newton's Method to converge on solution
    # Declare Constants that do not need to be computed within the while loop
    err = torch.empty(3)
    err[:]=np.Inf
    
    dr0v0Smu = torch.empty(t.size(0))
    dr0v0Smu[:] = np.nan
    
    for i in range(t.size(0)):        
        dr0v0Smu[i] = torch.matmul(r0[:, i], v0[:, i].reshape(-1,1))

    Smut=torch.sqrt(mu) * t
    iterate = 1
    while iterate == 1:
        print("While loop iteration")
        iterate = 0
        for i in range(err.size(0)):
            if abs(err[i]) > tol:
                print(abs(err[i]) - tol)
                iterate = 1
      
        X02=X0 ** 2
        X03=X02 * X0
        psi=X02 * alpha
        c2,c3=c2c3(psi,nargout=2)
        X0tOmPsiC3=X0 * (torch.tensor(1) - psi * c3)
        X02tC2=X02 * c2
        r=X02tC2 + dr0v0Smu * X0tOmPsiC3 + r0Mag * (torch.tensor(1) - psi * c2)
        Xn=X0 + (Smut - X03 * c3 - dr0v0Smu * X02tC2 - r0Mag * X0tOmPsiC3) / r
        err=Xn - X0
        X0=Xn
    
    f=1 - (Xn ** 2) * c2 / r0Mag
    g=t - (Xn ** 3) * c3 / torch.sqrt(mu)
    gdot=1 - c2 * (Xn ** 2) / r
    fdot=Xn * (psi * c3 - 1) * torch.sqrt(mu) / (r * r0Mag)
    r=f * r0 + g * v0
    v=fdot * r0 + gdot * v0
    ## Ensure Solution Integrity
    #idx = round((f.*gdot - fdot.*g)./tol).*tol ~= 1; r(:,idx) = NaN; v(:,idx) = NaN;

    return r,v
    
if __name__ == '__main__':
    pass
    
    

def c2c3(psi=None,*args,**kwargs):

    c2 = torch.empty(psi.size(0))
    c3 = torch.empty(psi.size(0))
    c2[:] = np.nan
    c3[:] = np.nan
    
    idx=psi     #> 1e-06
    
    
    for i in range(idx.size(0)):
        if (idx[i] > 1e-06):
            for i in range(idx.size(0)):            
                c2[i]=(torch.tensor(1) - torch.cos(torch.sqrt(psi[i]))) / psi[i]
                c3[i]=(torch.sqrt(psi[i]) - torch.sin(torch.sqrt(psi[i]))) / torch.sqrt(psi[i] ** 3)
        c2[0] = 0
        c3[0] = 0
        
    idx=psi      #< - 1e-06
    for i in range(idx.size(0)):
        if (idx[i] < - 1e-06):
            c2[i]=(1 - cosh(sqrt(- psi(i)))) / psi(i)
            c3[i]=(sinh(sqrt(- psi(i))) - sqrt(- psi(i))) / sqrt(- psi(i) ** 3)
    
    idx=abs(psi)      #<= 1e-06
    for i in range(idx.size(0)):
        if (idx[i] < - 1e-06):
            c2[i]=0.5
            c3[i]=1 / 6
    print("Calculation c2,c3")
    return c2,c3
    
if __name__ == '__main__':
    pass
    