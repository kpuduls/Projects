import numpy as np
from ortools.linear_solver import pywraplp
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
T_s = 1

# Matrix A
Ad = np.eye(Ac.shape[0]) + Ac * T_s

# Matrix B
Bd = Bc * T_s

# Matrix C
Cd = Cc

# Matrix D
Dd = Dc

# Prediction matrices
# Number of time samples
N = 100

# Define matrix dimensions
n = Ac.shape[0]
m = Bc.shape[1]

Ap = np.empty((0, n))
Bp = np.zeros((n*N,m*N))

for i in range(1, N + 1, 1):
    Ap = np.vstack([Ap, np.linalg.matrix_power(Ad, i)])
    for j in range(1, i + 1, 1):
        Bp[(i-1)*6:(6*i), (j-1)*3:3*j] = np.matmul(np.linalg.matrix_power(Ad, i-1), Bd)

# number of binary integer constraints b
b = 0
# number of desired position state vars
l = 3
# Weighting of terminal cost to fuel cost p
p = 1

# z = [u(k), d(k), b(k), l(k)]
# u = S * z (for accessing the control input time samples)
S = np.hstack([np.eye(m * N), np.zeros((m * N, m * N))])

# z = [u(k), d(k), b(k), l(k)]
# d = Sc * z (for accessing the control input time samples)
Sc = np.hstack([np.zeros((m * N, m * N)), np.eye(m * N)])

# To minimize used fuel constraint: Mcost*z(k) <= Pcost
Mcost = np.vstack([S -Sc, -S -Sc])
Pcost = np.zeros((2 * m * N, 1))

# Initial conditions
x01 = np.array([[10],[0],[0],[0],[0],[0]])
#x02 = np.array([[0],[0],[0],[0],[0],[0]])

x0 = x01
#x0 = np.vstack([x01, x02])

# Desired final positions
# target
xdes = np.array([[0], [0], [0], [0], [0], [0]])

# Target ydes = Cd * x(N)
# X(N) = Tn * x(k)
Tn = np.hstack([np.zeros((n, (N-1)*n)), np.eye(n)])

# Inequalities for minimizing distance to desired positions (receeding horizon)
Mt=np.vstack([np.matmul(Tn, np.matmul(Bp, S)), - np.matmul(Tn, np.matmul(Bp, S))])
Pt=np.vstack([xdes - np.matmul(Tn, np.matmul(Ap, x0)), - xdes + np.matmul(Tn, np.matmul(Ap, x0))])

# Saturation constraints: Msat * z(k) <= Psat
# Max thruster impuslive force 500N
Fmax = 500
umax = Fmax * np.ones((m*N, 1))
Msat = np.vstack([S, -S])
Psat = np.vstack([umax, umax])

Aiq = np.vstack([Mcost, Mt, Msat])
Biq = np.vstack([Pcost, Pt, Psat])

# Define for cost function J min f * z(k)
f = np.hstack([np.zeros((1, N * m)), np.ones((1, N * m))])

# Define for MILP solver
# Create the mip solver with the SCIP backend.
solver = pywraplp.Solver.CreateSolver('SCIP')

inf = solver.infinity()

coefs = Aiq.tolist()

mins = Biq.flatten().tolist()

u = []
d = []
names = ['x', 'y', 'z']

for i in range(N):
    for j in names:
        u.append('u'+j+'{0}'.format(i))
        d.append('d'+j+'{0}'.format(i))

z = u + d

cost_fn = f.flatten().tolist()

# Declare vars
variables = [[]] * len(z)
for i in range(0, len(z)):
    variables[i] = solver.NumVar(-inf, inf, z[i])

# Define constraints
constraints = [0] * len(coefs)

constraints[0] = solver.Constraint(-inf, mins[0])
constraints[0].SetCoefficient(variables[0], coefs[0][0])

for i in range(0, len(coefs)):
    constraints[i] = solver.Constraint(-inf, mins[i])
    for j in range(0, len(coefs[i])):
        constraints[i].SetCoefficient(variables[j], coefs[i][j])

# Define objective function
obj = solver.Objective()
for i in range(0, len(cost_fn)):
    obj.SetCoefficient(variables[i], cost_fn[i])

obj.SetMinimization() # set the problem goal as maximization

# Solve the MILP
status = solver.Solve()
print('Objective value = ', obj.Value())

zcost = []

# Print results
for i in range(0, len(z)):
    print('%s = %f' %(z[i], variables[i].solution_value()))
    zcost.append(variables[i].solution_value())

zcost = np.array(zcost)

ustar = zcost[0:(N*m)]
d = zcost[N*m:]

ustar = np.reshape(ustar, (len(ustar), 1))

zstar = np.matmul(Ap, x0) + np.matmul(Bp, ustar)

zstar = np.reshape(zstar, (N, n))

ustar = np.reshape(ustar, (N, m))

ux = ustar[:, 0]
uy = ustar[:, 1]
uz = ustar[:, 2]

x = zstar[:, 0]
y = zstar[:, 2]
z = zstar[:, 4]
xdot = zstar[:, 1]
ydot = zstar[:, 3]
zdot = zstar[:, 5]

fig, axs = plt.subplots(2, 3, sharex=True, sharey=True)

# marker symbol
axs[0, 0].scatter(range(N), x, s=80, c=z)
axs[0, 0].set_title('x')

# marker from TeX
axs[0, 1].scatter(range(N), y, s=80, c=z)
axs[0, 1].set_title('y')

# marker from path
axs[0, 2].scatter(range(N), z, s=80, c=z)
axs[0, 2].set_title('z')

# regular polygon marker
axs[1, 0].scatter(range(N), ux, s=80, c=z)
axs[1, 0].set_title('ux')

# regular star marker
axs[1, 1].scatter(range(N), uy, s=80, c=z)
axs[1, 1].set_title('uy')

# regular asterisk marker
axs[1, 2].scatter(range(N), uz, s=80, c=z)
axs[1, 2].set_title('uz')

plt.tight_layout()
plt.show()