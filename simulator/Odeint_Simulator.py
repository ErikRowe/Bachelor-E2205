import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from mpl_toolkits.mplot3d import Axes3D

def CombineMatrix (M11, M12, M21, M22):
    #Combines four nxn matrices to one 2nx2n matrix
    return np.concatenate((np.concatenate((M11,M12),axis=1),np.concatenate((M21,M22),axis=1)))

def S(a):
    #Skew symetrical matrix operator
    return np.array([[0, -a[2], a[1]],[a[2], 0, -a[0]],[-a[1],a[0],0]])

def U(q):
    #q = [n, e1, e2, e3]^T = [eta, epsilon]^T
    #Used in Jacobian between {I}- and {B} frame 
    e = np.array([q[1], q[2], q[3]])
    n = q[0]
    bottom = n*I3x3 + S(e)
    return np.array([-e,bottom[0],bottom[1],bottom[2]])

def C(nu):
    #Coriolis matrix
    #nu = [v, omega]^T = [u, v, w, p, q, r]^T
    global _M11, _M12, _M21, _M22
    C11 = O3x3
    C12 = -S(_M11@nu[:3]+_M12@nu[3:])
    C21 = -S(_M11@nu[:3]+_M12@nu[3:])
    C22 = -S(_M22@nu[3:]+_M21@nu[:3])
    return CombineMatrix(C11,C12,C21,C22)

def D(nu):
    #Dampening matrix
    #nu = [v, omega]^T = [u, v, w, p, q, r]^T
    D1 = np.diag((70, 100, 100, 30, 50, 50))  
    D2 = np.diag((100*abs(nu[0]), 200*abs(nu[1]), 200*abs(nu[2]), 50*abs(nu[3]), 100*abs(nu[4]), 100*abs(nu[5])))
    return D1+D2

def QuaternionMultiplication(q1,q2):
    M11 = np.array([q1[0]])
    M12 = np.array([-q1[1], -q1[2], -q1[3]])
    M21 = np.array([[q1[1]], [q1[2]], [q1[3]]])
    M22 = q1[0]*I3x3 + S(q1[1:])
    M1 = np.concatenate((M11, M12))
    M2 = np.concatenate((M21, M22),axis = 1)
    M = np.array([M1, M2[0], M2[1], M2[2]])
    return M@q2

def conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def sign(a):
    return 1 if a >= 0 else -1

def Kp(q):
    return CombineMatrix(np.transpose(R.from_quat(q).as_matrix())*Kxp, O3x3, O3x3, cp*I3x3)

def Ki(q):
    return CombineMatrix(np.transpose(R.from_quat(q).as_matrix())*Kxi, O3x3, O3x3, ci*I3x3)

#Matrices needed inside other matrices
I3x3 = np.identity(3)
O3x3 = np.zeros((3,3))
O4x3 = np.zeros((4,3))

#Interia matrix is constatnt, values from Fossen 1994
_M11 = np.array([[215, 0, 0],[0, 265, 0],[0, 0, 265]])
_M12 = O3x3
_M21 = O3x3
_M22 = np.array([[40, 0, 0],[0, 80, 0],[0, 0, 80]])
M = CombineMatrix(_M11,_M12,_M21,_M22)

def model(state,t):
    global M, mode
    x = state[:3]
    q = state[3:7]
    nu = state[7:]
    z_x = x - x_d
    q_tilde = QuaternionMultiplication(conjugate(q_d),q)
    q_tilde /= np.linalg.norm(q_tilde)
    z_q = sign(q_tilde[0])*np.array([q_tilde[1], q_tilde[2], q_tilde[3]])
    z = np.concatenate([z_x,z_q])
    J11 = np.concatenate([R.from_quat(q).as_matrix(), O3x3], axis=1)
    J21 = np.concatenate([O4x3,1/2*U(q)], axis = 1)
    J = np.concatenate([J11,J21])
    zeta_dot = J@nu
    if mode == 'PID':
        nu_dot = np.linalg.inv(M)@(-C(nu)@nu - D(nu)@nu - Kd@nu - Kp(q)@z - Ki(q)@z)
    elif mode == 'PD':
        nu_dot = np.linalg.inv(M)@(-C(nu)@nu - D(nu)@nu - Kd@nu - Kp(q)@z)
    elif mode == 'P':
        nu_dot = np.linalg.inv(M)@(-C(nu)@nu - D(nu)@nu - Kp(q)@z)

    print(zeta_dot)
    return np.concatenate([zeta_dot,nu_dot])


# Controller parameters
Kxp = 10000                    # Position proportional gain
cp = 2000                     # Attitude proportional gain
Kd = 3000*np.identity((6))       # Derivative gain
Kxi = 50                   # Position integral gain
ci = 200                    # Attitude integral gain

## Simulation parameters
# Timeaxis
t = np.linspace(0,5,1000)

# Inintal values
x_init = np.array([0, 0, 0])
q_init = np.array([1] + [0]*3)
nu_init = np.array([0]*6)
zeta0 = np.concatenate([x_init, q_init, nu_init])

# Desired endpoint and orientation
x_d = np.array([10,10,10])
q_d = np.array([1] + [0]*3)
print(q_d)

# Simulation
modes = ['P','PD','PID']

mode = modes[1]
simulation = odeint(model,zeta0,t)
x = simulation[:,0]
y = simulation[:,1]
z = simulation[:,2]


## Plot

# 3D
# fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
# ax.set_xlabel('x [m]')
# ax.set_ylabel('y [m]')
# ax.set_zlabel('z [m]')
# ax.set_xlim([0,10])
# ax.set_ylim([0,10])
# ax.set_zlim([0,10])
# ax.plot3D(x, y, z)

# 2D
fig, axs = plt.subplots(1, 3, figsize=(12, 3))
axs[0].set_ylabel('x [m]')
axs[1].set_ylabel('y [m]')
axs[2].set_ylabel('z [m]')
axs[0].plot(t, x)
axs[1].plot(t, y)
axs[2].plot(t, z)

for ax in axs: 
    ax.grid()
    ax.set_xlabel('t [s]')
    ax.set_ylim([0, 15])
plt.show()

#fig.savefig('/home/elias/Documents/Bachelor/Simuleringer/Plots/IN3D.eps',format = 'eps', dpi = 1200)
