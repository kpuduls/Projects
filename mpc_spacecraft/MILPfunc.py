import numpy as np
import gurobipy as gp
from gurobipy import *
import matplotlib.pyplot as plt

np.set_printoptions(suppress=True)


def optimize(x0, xdes, T_s, N, h, ms):
    ## Purpose:
    # Calculate the optimal trajectory given initial conditions and desired final position
    # optimizing fuel efficiency, while having thruster saturation constraints etc
    # Dynamcis modelled based on the Hills equations
    # Note: Near circular orbit approximation used

    ## Inputs:
    # x0              ndarray           [6 x 1]                 Hills relative initial state

    # xdes            ndaray           [6 x 1]                 Hills relative final desired state

    # T_s                         const                  Defines the sample time, but also thruster impulse time

    # N                           const                  Defines prediction horizon (number of time samples)

    # h                           const                  Defines the reference target orbit height

    ## Outputs:
    # rHill           tensor            [3 x N]                Hills populated trajectory relative position states

    # vHill           tensor            [3 x N]                Hills populated relative velocity states

    # State space model matrices (continuous time)
    # Space environment, spacecraft and target parameters
    Re = 6371000  # Earth radius in m
    R = h + Re  # Altitude from earth COMz
    mu = 398600000  # Earth gravitational parameter
    T = 2 * np.pi * np.sqrt(R ** 3 / mu)  # Period of the target orbit
    w = 2 * np.pi / T  # Target orbit angular frequency

    x0 = np.array(x0)
    xdes = np.array(xdes)

    # Defining continuous time State Space Model Matrices based on Hills equations
    Ac = np.array([[0, 1, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 2 * w],
                   [0, 0, 0, 1, 0, 0],
                   [0, 0, -w ** 2, 0, 0, 0],
                   [0, 0, 0, 0, 0, 1],
                   [0, -2 * w, 0, 0, 3 * w ** 2, 0]])

    # For 2 spacecraft
    # Ac = np.kron(np.eye(2), A)

    Bc = np.array([[0, 0, 0],
                   [1 / ms, 0, 0],
                   [0, 0, 0],
                   [0, 1 / ms, 0],
                   [0, 0, 0],
                   [0, 0, 1 / ms]])

    # For 2 spacecraft
    # Bc = np.kron(np.eye(2), B)

    Cc = np.array([[1, 0, 0, 0, 0, 0],
                   [0, 0, 1, 0, 0, 0],
                   [0, 0, 0, 0, 1, 0]])

    # For 2 Spacecraft
    # Cc = np.kron(np.eye(2), C)

    Dc = np.array([[0, 0, 0],
                   [0, 0, 0],
                   [0, 0, 0]])

    # For 2 Spacecraft
    # Dc = np.kron(np.eye(2), D)

    # Discrete State Space Model Matrices

    # Matrix A
    Ad = np.eye(Ac.shape[0]) + Ac * T_s

    # Matrix B
    Bd = Bc * T_s

    # Matrix C
    Cd = Cc

    # Matrix D
    Dd = Dc

    # Prediction matrices

    # Define matrix dimensions
    n = Ac.shape[0]
    m = Bc.shape[1]

    Ap = np.empty((0, n))
    Bp = np.zeros((n * N, m * N))

    loop = 0

    for i in range(N):
        Ap = np.vstack([Ap, np.linalg.matrix_power(Ad, i + 1)])
        loop = loop + 1
        for j in range(loop):
            Bp[n * i:n * (i + 1), (j * m):(m * (j + 1))] = np.matmul(np.linalg.matrix_power(Ad, i - j), Bd)

    # number of binary integer constraints b
    b = 0
    # number of desired position state vars
    l = 6
    # Weighting of terminal cost to fuel cost p
    p = 10

    # z = [u(k), d(k), b(k), l(k)]
    # u = S * z (for accessing the control input time samples)
    S = np.hstack([np.eye(m * N), np.zeros((m * N, m * N + l))])

    # z = [u(k), d(k), b(k), l(k)]
    # d = Sc * z (for accessing the control input time samples)
    Sc = np.hstack([np.zeros((m * N, m * N)), np.eye(m * N), np.zeros((m * N, l))])

    # To minimize used fuel constraint: Mcost*z(k) <= Pcost
    Mcost = np.vstack([S - Sc, -S - Sc])
    Pcost = np.zeros((2 * m * N, 1))

    # w = Tw * z
    Tw = np.hstack([np.zeros((l, N * 2 * m)), np.eye(l)])

    # Initial conditions
    x01 = np.array([[10], [0], [0], [0], [0], [0]])
    # x02 = np.array([[0],[0],[0],[0],[0],[0]])

    # x0 = np.vstack([x01, x02])

    # Desired final positions
    # target
    #xdes = np.array([[0], [0], [0]])

    # Target ydes = Cd * x(N)
    # X(N) = Tn * x(k)
    Tn = np.hstack([np.zeros((n, (N - 1) * n)), np.eye(n)])

    # Inequalities for minimizing distance to desired positions (receeding horizon)
    Mt = np.vstack(
        [np.matmul(Tn, np.matmul(Bp, S)), -np.matmul(Tn, np.matmul(Bp, S))])
    Pt = np.vstack([xdes - np.matmul(Tn, np.matmul(Ap, x0)),
                    - xdes + np.matmul(Tn, np.matmul(Ap, x0))])

    # Saturation constraints: Msat * z(k) <= Psat
    # Max thruster impuslive force 500N
    Fmax = 500
    umax = Fmax * np.ones((m * N, 1))
    Msat = np.vstack([S, -S])
    Psat = np.vstack([umax, umax])

    Aiq = np.vstack([Mcost, Mt, Msat])
    Biq = np.vstack([Pcost, Pt, Psat]).flatten()

    # Define for cost function J min f * z(k)
    f = np.hstack([np.zeros((1, N * m)), np.ones((1, N * m)), np.zeros((1, l))]).flatten()

    # Create an optimization model
    model = gp.Model()

    # Define variables
    z = model.addMVar(n * N + l, vtype=GRB.CONTINUOUS, lb=-GRB.INFINITY, ub=GRB.INFINITY)

    # Define objective function
    model.setObjective(f @ z, GRB.MINIMIZE)

    # Define constraints
    model.addConstr(Aiq @ z <= Biq)

    model.optimize()

    zstar = []
    for val in model.getVars():
        zstar.append(val.x)

    zstar = np.array(zstar)

    ustar = np.mat(np.matmul(S, zstar)).T

    xstar = np.matmul(Ap, x0) + np.matmul(Bp, ustar)

    xstar = xstar.reshape((N, n))
    ustar = ustar.reshape((N, m))

    u0 = np.mat([[0], [0], [0]])

    ustar = np.vstack([u0.T, ustar])

    rHill = np.vstack([xstar.T[0, :], xstar.T[2, :], xstar.T[4, :]])
    vHill = np.vstack([xstar.T[1, :], xstar.T[3, :], xstar.T[5, :]])

    return rHill, vHill, xstar, ustar

if __name__ == '__main__':
    print("Main script run")
    # write code to be executed only on direct execution, but not on import
    # This is because direct execution assigns `'__main__'` to `__name__` while import of any way assigns the name under which it is imported
