from locale import normalize
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.integrate import odeint
import os
import math

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

# os.remove('C:\Users\elias\Documents\Debugging\v.txt')
# os.remove('C:\Users\elias\Documents\Debugging\q.txt')
# os.remove('C:\Users\elias\Documents\Debugging\x.txt')
# Python program to implement Runge Kutta method
def f(zeta,t):
    global M
    x = zeta[:3]
    # file = open('C:\Users\elias\Documents\Debugging\x.txt','a')
    # file.write(repr(x) + "\n")
    # file.close
    q = zeta[3:7]/np.linalg.norm(zeta[3:7])
    
    # file = open('C:\Users\elias\Documents\Debugging\q.txt','a')
    # file.write(repr(q) + "\n")
    # file.close
    
    v = zeta[7:]
    
    # file = open('C:\Users\elias\Documents\Debugging\v.txt','a')
    # file.write(repr(v) + "\n")
    # file.close

    z1 = x - x_d
    q_tilde = quatMul(conjugate(q_d), q)
    q_tilde /= np.linalg.norm(q_tilde)
    z2 = sign(q[0])*np.array([q_tilde[1], q_tilde[2], q_tilde[3]])
    z = np.concatenate([z1, z2])
    zeta11 = np.concatenate([R.from_quat(q).as_matrix(), Zero3x3], axis=1)
    zeta22 = np.concatenate([Zero4x3, 1/2*U(q)], axis=1)
    zetaDot = np.concatenate([zeta11, zeta22])@v
    vDot = -np.linalg.inv(M)@(C(v)@v+D(v)@v+Kd@v+Kp(q)@z)
    xDot = np.concatenate([zetaDot, vDot])

    return xDot


def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in degree.
    :param pitch: The pitch (rotation around y-axis) angle in degree.
    :param yaw: The yaw (rotation around z-axis) angle in degree.
 
  Output
    :return eta, e1, e2, e3: The orientation in quaternion [eta, e1, e2, e3] format
  """
  roll = roll/180 * math.pi
  pitch = pitch/180 * math.pi
  yaw = yaw/180 * math.pi
  e1 = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  e2 = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  e3 = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  eta = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [eta, e1, e2, e3]


def euler_from_quaternion(eta, e1, e2, e3,):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (eta * e1 + e2 * e3)
        t1 = +1.0 - 2.0 * (e1 * e1 + e2 * e2)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (eta * e2 - e3 * e1)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (eta * e3 + e1 * e2)
        t4 = +1.0 - 2.0 * (e2 * e2 + e3 * e3)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x*180/math.pi, pitch_y*180/math.pi, yaw_z*180/math.pi # in degrees


# x = surge
# y = sway
# z = heave
# om x akse = pitch
# om y akse = roll
# om z akse = yaw

## Initial position, attitude and velocities
x_init = np.array([0, 0, 0])
q_init = np.array(get_quaternion_from_euler(0,0,0))
lin_speed = [0, 0, 0]
ang_speed = [0, 0, 0]
v_init = np.concatenate((lin_speed, ang_speed))
zeta0 = np.concatenate([x_init, q_init, v_init])


## Desired position and attitude
x_d = np.array([0, 0, 0])
q_d = np.array(get_quaternion_from_euler(0,0,90)) 
print(q_d)

################## Simulation ##################################################################################### 
start_time = 0
end_time = 2
stepsize = round(end_time/0.03) #5/x = 0.03 => end_time/0.03
t = np.linspace(start_time,end_time,stepsize)  # gjør om slik at det samples hver 30ms = 0.03 s

Kx = 1
c  = 40
Kd = 1*np.diag([1]*6)
sim = odeint(f,zeta0,t)
###################################################################################################################

# Values from simulation
surge = sim[:, 0]
sway = sim[:, 1]
heave = sim[:, 2]

# Convertion from Quaternions to Euler angles 
eta = np.array([sim[:,3]])
epsilon1 = np.array([sim[:,4]])
epsilon2 = np.array([sim[:,5]])
epsilon3 = np.array([sim[:,6]])
q1 = np.concatenate((np.transpose(eta),np.transpose(epsilon1)),axis = 1)
q2 = np.concatenate((np.transpose(epsilon2),np.transpose(epsilon3)),axis = 1)
q = np.concatenate((q1,q2),axis = 1)

euler = []
for i in range(len(q)):
    ye = euler_from_quaternion(q[i][0], q[i][1], q[i][2], q[i][3])
    euler.append(ye)
euler_roll = []
euler_pitch = []
euler_yaw = []
for i in range(len(euler)):
    euler_roll.append(euler[i][0])
    euler_pitch.append(euler[i][1])
    euler_yaw.append(euler[i][2])

# Linear and angular speeds
surge_speed = sim[:, 7]
sway_speed = sim[:, 8]
heave_speed = sim[:, 9]

roll_speed = sim[:, 10]
pitch_speed = sim[:, 11]
yaw_speed = sim[:, 12]
## Plot
#### PARAM TESTING ####
### Quaternions ###
fig, axs = plt.subplots(1, 4, figsize=(8, 5))
axs[0].plot(t, eta[0])
axs[1].plot(t, epsilon1[0])
axs[2].plot(t, epsilon2[0])
axs[3].plot(t, epsilon3[0])
axs[0].set_title('w')
axs[1].set_title('x')
axs[2].set_title('y')
axs[3].set_title('z')
for ax in axs:
    ax.grid()
    ax.set_xlabel('t [s]')
    ax.set_ylim([-1,1])

### YAW ###
# fig, axs = plt.subplots(1, 2, figsize=(8, 5))
# axs[0].plot(t, yaw_speed)
# axs[1].plot(t, euler_yaw)
# axs[0].set_ylabel('z [rad/s]')
# axs[0].set_title('Yawrate')
# axs[1].set_ylabel('z [degrees]')
# axs[1].set_title('Yaw')
# for ax in axs:
#     ax.grid()
#     ax.set_xlabel('t [s]')

### PITCH ###
# fig, axs = plt.subplots(1, 2, figsize=(8, 5))
# axs[0].plot(t, pitch_speed)
# axs[1].plot(t, euler_pitch)
# axs[0].set_ylabel('y [rad/s]')
# axs[0].set_title('Pitchrate')
# axs[1].set_ylabel('y [degrees]')
# axs[1].set_title('Pitch')
# for ax in axs:
#     ax.grid()
#     ax.set_xlabel('t [s]')

### ROLL ###
# fig, axs = plt.subplots(1, 2, figsize=(8, 5))
# axs[0].plot(t, roll_speed)
# axs[1].plot(t, euler_roll)
# axs[0].set_ylabel('x [rad/s]')
# axs[0].set_title('Rollrate')
# axs[1].set_ylabel('x [degrees]')
# axs[1].set_title('Roll')
# for ax in axs:
#     ax.grid()
#     ax.set_xlabel('t [s]')

# Linear and angular speeds
# fig, axs = plt.subplots(2, 3, figsize=(8, 5))
# axs[0,0].set_ylabel('x [m/s]')
# axs[0,0].set_title('Surge')
# axs[0,1].set_ylabel('y [m/s]')
# axs[0,1].set_title('Sway')
# axs[0,2].set_ylabel('z [m/s]')
# axs[0,2].set_title('Heave')
# axs[1,0].set_ylabel('x [rad/s]')
# axs[1,0].set_title('Roll')
# axs[1,1].set_ylabel('y [rad/s]')
# axs[1,1].set_title('Pitch')
# axs[1,2].set_ylabel('z [rad/s]')
# axs[1,2].set_title('Yaw')

# axs[0,0].plot(t, surge_speed)
# axs[0,1].plot(t, sway_speed)
# axs[0,2].plot(t, heave_speed)

# axs[1,0].plot(t, roll_speed)
# axs[1,1].plot(t, pitch_speed)
# axs[1,2].plot(t, yaw_speed)
# for i in range(3):
#     axs[0,i].grid()
#     axs[1,i].grid()
#     axs[0,i].set_xlabel('t [s]')
#     axs[1,i].set_xlabel('t [s]')



# 2D Attitude
# fig, axs = plt.subplots(1, 3, figsize=(8, 5))
# axs[0].set_ylabel('x [grader]')
# axs[0].set_title('Roll')
# axs[1].set_ylabel('y [grader]')
# axs[1].set_title('Pitch')
# axs[2].set_ylabel('z [grader]')
# axs[2].set_title('Yaw')
# axs[0].plot(t, euler_roll)
# axs[1].plot(t, euler_pitch)
# axs[2].plot(t, euler_yaw)

# for ax in axs:
#     ax.grid()
#     ax.set_xlabel('t [s]')

# 3D Position
# fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
# ax.set_xlabel('x [m]')
# ax.set_ylabel('y [m]')
# ax.set_zlabel('z [m]')
# ax.set_xlim([0,10])
# ax.set_ylim([0,10])
# ax.set_zlim([0,10])
# ax.plot3D(surge, sway, heave)

# 2D Position
# fig, axs = plt.subplots(1, 3, figsize=(20, 8))
# axs[0].set_ylabel('x [m]')
# axs[0].set_title('Surge')
# axs[1].set_ylabel('y [m]')
# axs[1].set_title('Sway')
# axs[2].set_ylabel('z [m]')
# axs[2].set_title('Heave')
# axs[0].plot(t, surge)
# axs[1].plot(t, sway)
# axs[2].plot(t, heave)

# for ax in axs:
#     ax.grid()
#     ax.set_xlabel('t [s]')
#     ax.set_ylim([-2, 2])
plt.show()