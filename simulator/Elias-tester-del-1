import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.integrate import odeint
# Constants
m = 11.5                    # Kg
W = 112.8                   # Newton
B = 114.8                   # Newtons
r_b = np.array([[0],        # m
                [0],
                [0]])
r_g = np.array([[0],        # m
                [0],
                [0.02]])
I_x = 0.16                  # Kg m**2
I_y = 0.16                  # Kg m**2
I_z = 0.16                  # Kg m**2

X_u = -4.03                 # Ns/m
N_r = -0.07                 # Ns/m
Y_v = -6.22                 # Ns/m
K_p = -0.07                 # Ns/m
M_q = -0.07                 # Ns/m
Z_w = -5.18

X_u_abs = -18.18            # Ns**2/m**2
Y_v_abs = -21.66            # Ns**2/m**2
Z_w_abs = -36.99            # Ns**2/m**2
K_p_abs = -1.55             # Ns**2/m**2
M_q_abs = -1.55             # Ns**2/m**2
N_r_abs = -1.55

X_udot = -5.5               # Kg
Y_vdot = -12.7              # Kg
Z_wdot = -14.57             # Kg
K_pdot = -0.12              # Kg m**2/rad
M_qdot = -0.12              # Kg m**2/rad
N_rdot = -0.12              # Kg m**2/rad


def split(array, nrows, ncols):
    """Split a matrix into sub-matrices."""
    r, h = array.shape
    return (array.reshape(h//nrows, nrows, -1, ncols)
            .swapaxes(1, 2)
            .reshape(-1, nrows, ncols))

def Combine4(M11, M12, M21, M22):
    return np.array(np.concatenate((np.concatenate((M11, M12), axis=1),
                                    np.concatenate((M21, M22), axis=1))))


def S(a):
    return np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])


def U(q):
    e = np.array([q[1], q[2], q[3]])
    top = -e
    bottom = q[0] * I3x3 + S(e)
    return np.array([top, bottom[0], bottom[1], bottom[2]])


def C(v):
    global _M11, _M12, _M21, _M22
    C11 = Zero3x3
    C12 = -S(_M11@v[:3] + _M12@v[3:])
    C21 = C12
    C22 = -S(_M22@v[:3] + _M21@v[3:])
    return Combine4(C11, C12, C21, C22)


def D(nu):
    DL = -np.diag((X_u, Y_v, Z_w, K_p, M_q, N_r))
    DNL = -np.diag((X_u_abs*abs(nu[0]), Y_v_abs*abs(nu[1]), Z_w_abs*abs(
        nu[2]), K_p_abs*abs(nu[3]), M_q_abs*abs(nu[4]), N_r_abs*abs(nu[5])))
    return DL + DNL


def Kp(q):
    return Combine4(R.from_quat(q).as_matrix()*Kx, Zero3x3, Zero3x3, c*I3x3)


def quatMul(q1, q2):
    M11 = np.array([q1[0]])
    M12 = np.array(-q1[1:])
    M21 = np.array([[q1[1]], [q1[2]], [q1[3]]])
    M22 = np.array(q1[0]*I3x3 + S(q1[1:]))
    Mx = np.concatenate([M11, M12])
    Kk = np.concatenate([M21, M22], axis=1)
    result = np.array([Mx, Kk[0], Kk[1], Kk[2]])
    return result@q2


def conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def sign(a):
    return 1 if a >= 0 else -1


Zero3x3 = np.array([[0]*3]*3)
Zero4x3 = np.array([[0]*3]*4)
I3x3 = np.diag([1]*3)

_M11rb = np.array([[m, 0, 0], [0, m, 0], [0, 0, m]])
#_M12rb = np.array([[0, m*r_g[2][0], 0], [-m*r_g[2][0], 0, 0], [0, 0, 0]])
#_M21rb = np.array([[0, -m*r_g[2][0], 0], [m*r_g[2][0], 0, 0], [0, 0, 0]])
_M12rb = Zero3x3
_M21rb = Zero3x3
_M22rb = np.array([[I_x, 0, 0], [0, I_y, 0], [0, 0, I_z]])
_M11A = -np.array([[X_udot, 0, 0], [0, Y_vdot, 0], [0, 0, Z_wdot]])
_M12A = Zero3x3
_M21A = Zero3x3
_M22A = np.array([[K_pdot, 0, 0], [0, M_qdot, 0], [0, 0, N_rdot]])
Mrb = Combine4(_M11rb, _M12rb, _M21rb, _M22rb)
MA = Combine4(_M11A,_M12A,_M21A,_M22A)
M = Mrb + MA
_M11, _M12, _M21, _M22 = split(M,3,3)


# Python program to implement Runge Kutta method
def f(zeta,t):
    global M
    x = zeta[:3]
    q = zeta[3:7]/np.linalg.norm(zeta[3:7])
    #print(zeta[3:7])
    v = zeta[7:]
    z1 = x - x_d
    q_tilde = quatMul(conjugate(q_d), q)
    q_tilde /= np.linalg.norm(q_tilde)
    z2 = sign(q[0])*np.array([q_tilde[1], q_tilde[2], q_tilde[3]])
    z = np.concatenate([z1, z2])
    zeta11 = np.concatenate([R.from_quat(q).as_matrix(), Zero3x3], axis=1)
    zeta22 = np.concatenate([Zero4x3, 1/2*U(q)], axis=1)
    zetaDot = np.concatenate([zeta11, zeta22])@v
    vDot = -np.linalg.inv(M)@(C(v)@v+D(v)@v+Kd@v+Kp(q)@z)
    a = np.concatenate([zetaDot, vDot])
    return a


x_init = np.array([0, 0, 0])
q_init = np.array([1] + [0]*3)
v_init = np.array([0]*6)
zeta0 = np.concatenate([x_init, q_init, v_init])

x_d = np.array([1, 1, 1])
q_d = np.array([0.71, 0, 0, 0.71]) #90 graders rotasjon med klokka xy planet

t = np.linspace(0,5)

Kp_values = [30]

sim_surge = []
sim_sway = []
sim_heave = []
sim_yaw = []
sim_roll = []
sim_pitch = []

"""
x = surge
y = sway
z = heave
om x akse = pitch
om y akse = roll
om z akse = yaw
"""

for value in range(1):
    c  = 200
    Kd = 1*np.diag([1]*6)
    Kx = Kp_values[value]
    sim = odeint(f,zeta0,t)
    surge = sim[:, 0]
    sway = sim[:, 1]
    heave = sim[:, 2]
    pitch = sim[:, 3]
    roll = sim[:, 4]
    yaw = sim[:, 5]
    sim_surge.append(surge)
    sim_sway.append(sway)
    sim_heave.append(heave)
    sim_pitch.append(pitch)
    sim_roll.append(roll)
    sim_yaw.append(yaw)
"""
c  = 200
Kd = 50*np.diag([1]*6)
Kx = 400
sim = odeint(f,zeta0,t)
print(sim)
"""
## Plot

# 3D
# fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
# ax.set_xlabel('x [m]')
# ax.set_ylabel('y [m]')
# ax.set_zlabel('z [m]')
# ax.set_xlim([0,10])
# ax.set_ylim([0,10])
# ax.set_zlim([0,10])
# ax.plot3D(surge, sway, heave)

# 2D
fig, axs = plt.subplots(2, 3, figsize=(20, 8))
axs[0,0].set_ylabel('x [m]')
axs[0,0].set_title('Surge')
axs[0,1].set_ylabel('y [m]')
axs[0,1].set_title('Sway')
axs[0,2].set_ylabel('z [m]')
axs[0,2].set_title('Heave')
axs[1,0].set_ylabel('Grader ?')
axs[1,0].set_title('Pitch')
axs[1,1].set_ylabel('Grader ?')
axs[1,1].set_title('Roll')
axs[1,2].set_ylabel('Grader ?')
axs[1,2].set_title('Yaw')

for i in range(len(Kp_values)):
    axs[0,0].plot(t, sim_surge[i])
    axs[0,1].plot(t, sim_sway[i])
    axs[0,2].plot(t, sim_heave[i])
    axs[1,0].plot(t, sim_pitch[i])
    axs[1,1].plot(t, sim_roll[i])
    axs[1,2].plot(t, sim_yaw[i])

#for ax in axs:
    #print(ax)
    #ax.grid()
    #ax.set_xlabel('t [s]')
    #ax.set_ylim([-5, 5])
plt.show()
