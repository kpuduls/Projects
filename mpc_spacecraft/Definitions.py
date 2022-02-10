import numpy as np
from scipy.optimize import linprog
import pandas as pd
import matplotlib.pyplot as plt
np.set_printoptions(suppress=True)

# State space model matrices (continuous time)
# Space environment, spacecraft and target parameters
h = 400000                                  # altitude in m
Re = 6371000                                # Earth radius in m
R = h + Re                                  # Altitude from earth COMz
mu = 398600000                              # Earth gravitational parameter
T = 2 * np.pi * np.sqrt(R**3 / mu)        # Period of the target orbit
w = 2 * np.pi / T                         # Target orbit angular frequency
ms = 250                                    # Satellite mass in kg


# Defining continuous time State Space Model Matrices based on Hills equations
Ac =np.array([[0,    1,    0,     0,      0,      0],
              [0,    0,    0,     0,      0,    2*w],
              [0,    0,    0,     1,      0,      0],
              [0,    0,    -w**2, 0,      0,      0],
              [0,    0,    0,     0,      0,      1],
              [0,    -2*w, 0,     0,      3*w**2, 0]])

# For 2 spacecraft
#Ac = np.kron(np.eye(2), A)

Bc = np.array([[0,       0,        0],
               [1/ms,    0,        0],
               [0,       0,        0],
               [0,    1/ms,        0],
               [0,       0,        0],
               [0,        0,    1/ms]])

# For 2 spacecraft
#Bc = np.kron(np.eye(2), B)

Cc = np.array([[1, 0, 0, 0, 0, 0],
               [0, 0, 1, 0, 0, 0],
               [0, 0, 0, 0, 1, 0]])

# For 2 Spacecraft
#Cc = np.kron(np.eye(2), C)

Dc = np.array([[0, 0, 0],
               [0, 0, 0],
               [0, 0, 0]])

# For 2 Spacecraft
#Dc = np.kron(np.eye(2), D)


# Discrete State Space Model Matrices
#Sampling time
T_s = 10

# Matrix A
Ad = np.eye(Ac.shape[0])  + Ac * T_s

# Matrix B
Bd = Bc * T_s

# Matrix C
Cd = Cc

# Matrix D
Dd = Dc

# Prediction matrices
# Number of time samples
N = 10

# Define matrix dimensions
n = Ac.shape[0]
m = Bc.shape[1]

Ap = np.empty((0, n))
Bp = np.zeros((n*N,m*N))

for i in range(1, N + 1, 1):
    Ap = np.vstack([Ap, np.linalg.matrix_power(Ad, i)])
    for j in range(1, i + 1, 1):
        Bp[(i-1)*6:(6*i), (j-1)*3:3*j] = np.matmul(np.linalg.matrix_power(Ad, i-1), Bd)

# Cost function definition
# min |u_i|
# min d_i
# min c^T*z, z=[u_i; d_i; b_i]
# number of binary integer constraints b
b = 3

# for 2 spacecraft
# b = 6
# l = 6

# number of desired position state vars
l = 3

# Weighting of terminal cost to fuel cost p
p = 1

c = np.vstack([np.zeros((N * m, 1)), np.ones((N * m, 1)), np.zeros((b * N, 1)), p * np.ones((l, 1))])

# Allocate
# s.t. M*z <= P
M = np.vstack([np.hstack([np.eye((m * N)),  -np.eye((m * N)),  np.zeros((m * N, b * N + l))]),  np.hstack([-np.eye((m * N)), -np.eye((m * N)), np.zeros((m * N, b * N + l))])])
P = np.zeros((2 * m * N, 1))

# Target definition transform matrices for prediction states
# w = T_w * zstar, X_np = T_np * x
T_np = np.hstack([np.zeros((l, n * (N - 1))), np.kron(np.eye((l)), np.array([1, 0]))])
T_w = np.hstack([np.zeros((l, N * (2 * m + b))), np.eye((l))])

# Initial conditions
x01 = np.array([[0],[0],[0],[0],[0],[0]])
#x02 = np.array([[0],[0],[0],[0],[0],[0]])

x0 = x01
#x0 = np.vstack([x01, x02])

# Desired final positions
# target
X_fp = np.array([[0], [0], [0]])
Wn = np.array([[0],[0],[0],[0],[0],[0]])

# For 2 spacecraft
#X_fp = np.array([[0],[0],[0],[0],[0],[0]])
#Wn = np.array([[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0]])

# u = S * z (for accessing the control input time samples)
S = np.hstack([np.eye(m * N), np.zeros((m * N, (m + b) * N + l))])

# Inequalities for minimizing distance to desired positions (receeding horizon)
M_rh=np.vstack([np.matmul(T_np, np.matmul(Bp, S))-T_w, - np.matmul(np.matmul(T_np, Bp), S)-T_w])
P_rh=np.vstack([X_fp - np.matmul(np.matmul(T_np, Ap), x0), - X_fp + np.matmul(np.matmul(T_np, Ap), x0)])

# b_i = Sb * z
Sb = np.hstack([np.zeros((N * b, N * 2 * m)),  np.eye((b * N)), np.zeros((N * b, l))])

# Error box constraints
# Select position state vars
T0 = np.array([1, 0])
T0box = np.kron(np.eye(np.int_(n / 2)), T0)
Tbox = np.kron(np.eye((N)), T0box)

# Large error box
Xmax = 2000
Ymax = 100
Zmax = 2000

# For single s/c
D0box = np.vstack([Xmax, Ymax, Zmax])

# For 2 spacecraft
# D0box = np.vstack([Xmax, Ymax, Zmax, Xmax, Ymax, Zmax])

Dbox = np.kron(np.ones((N, 1)), D0box)

# Large error box constraints
M_leb = np.vstack([-np.matmul(Tbox, np.matmul(Bp, S)), np.matmul(Tbox, np.matmul(Bp, S))])
P_leb = np.vstack([np.matmul(Tbox, np.matmul(Ap, x0)) + Dbox, -np.matmul(Tbox, np.matmul(Ap, x0)) + Dbox])

# Small error box
Xmax = 100
Ymax = 100
Zmax = 200

# For single s/c
d0box = np.vstack([Xmax, Ymax, Zmax])

# For 2 s/c
#d0box = np.vstack([Xmax, Ymax, Zmax, Xmax, Ymax, Zmax])

dbox = np.kron(np.ones((N, 1)), d0box)

# Target at time t
Wt = np.kron(np.ones((N, 1)), Wn)

# Small error box constraints
M_seb = np.vstack([-np.matmul(Tbox, np.matmul(Bp, S)), np.matmul(Tbox, np.matmul(Bp, S))])
P_seb = np.vstack([np.matmul(Tbox, (np.matmul(Ap, x0) - Wt)) + dbox, -np.matmul(Tbox, (np.matmul(Ap, x0) - Wt)) + dbox])

# linprog

# Apply constraint inequalities
M = np.vstack([M, M_rh, M_leb])
P = np.vstack([P, P_rh, P_leb])

# Define lb up for u, d

# Allocate zstar
# zstar = zeros(N * (n), 1);

# Fictious Aeq and Beq
Aeq = np.zeros((1, N * (2 * m + b) + l))
Beq = 0