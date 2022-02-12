import numpy as np
import torch
import matplotlib.pyplot as plt
from MILPfunc import optimize

# This is a Demo of using the MPC controller for spacecraft trajectory optimization
# First define the problem

# Determine the optimal spacecraft trajectory in a near circular LEO orbit using Hills equations
# Define the spacecraft mass, target orbit height, initial state and desired final state
ms = 250                            # 250kg
h = 500000                          # 500km

# Initial state and desired final state in Hills reference frame (x = [x, xdot, y, ydot, z, zdot]^T)
x0 = [[100], [0], [0], [0], [0], [0]]
xdes = [[0], [0], [0], [0], [0], [0]]

# Since the problem is of discrete nature, define the smapling time
# This is also the impulse time for thrusters (Note: realistically it will be somewhere about few seconds)
T_s = 10                            # 10 seconds

# Define the prediction horizon length
# Note: this also determines the prediction total amount of time
N =  100

# Call the MPC solver function
# Remember!!!!
# Inputs: (x0, xdes, T_s, N, h, ms)
# Outputs: rHill, vHill (tensors), xstar, ustar (ndarray)
rHill, vHill, xstar, ustar = optimize(x0, xdes, T_s, N, h, ms)

# xstar are the optimal states at each time step
# ustar are the optimal control inputs at each time step

# Plot the result now
fig, ((ax1, ax2, ax3), (ax4, ax5, ax6)) = plt.subplots(2, 3)
fig.suptitle('Optimization results')

ax1.plot(T_s * np.array(range(N+1)), xstar[:, 0], marker='o')
ax2.plot(T_s * np.array(range(N+1)), xstar[:, 2], marker='o')
ax3.plot(T_s * np.array(range(N+1)), xstar[:, 4], marker='o')
ax4.step(T_s * np.array(range(N+1)), ustar[:, 0])
ax5.step(T_s * np.array(range(N+1)), ustar[:, 1])
ax6.step(T_s * np.array(range(N+1)), ustar[:, 2])

plt.show()

