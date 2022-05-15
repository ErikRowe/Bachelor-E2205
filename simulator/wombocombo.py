import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


from locale import normalize
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.integrate import odeint
import os
import math

#########################################################################################
#   Dataleser
path = '/home/elias/Node_intervall_12.xlsx'
df = pd.read_csv(path,skiprows=1)
title = '60_degrees_pitch_and_yaw'

Columns = df.columns 

Time = df[Columns[0]].tolist()
T1 = df[Columns[1]].tolist()
T2 = df[Columns[2]].tolist()
T3 = df[Columns[3]].tolist()
T4 = df[Columns[4]].tolist()
T5 = df[Columns[5]].tolist()
T6 = df[Columns[6]].tolist()
Z1 = df[Columns[7]].tolist()
Z2 = df[Columns[8]].tolist()
Z3 = df[Columns[9]].tolist()
Z4 = df[Columns[10]].tolist()
Z5 = df[Columns[11]].tolist()
Z6 = df[Columns[12]].tolist()
Q1 = df[Columns[13]].tolist() #x
Q2 = df[Columns[14]].tolist() #y
Q3 = df[Columns[15]].tolist() #z
Q4 = df[Columns[16]].tolist() #w
Q_D1 = df[Columns[17]].tolist() #x
Q_D2 = df[Columns[18]].tolist() #y
Q_D3 = df[Columns[19]].tolist() #z
Q_D4 = df[Columns[20]].tolist() #w
X1 = df[Columns[21]].tolist()
X2 = df[Columns[22]].tolist()
X3 = df[Columns[23]].tolist()
X_D1 = df[Columns[24]].tolist()
X_D2 = df[Columns[25]].tolist()
X_D3 = df[Columns[26]].tolist()
V1 = df[Columns[27]].tolist()
V2 = df[Columns[28]].tolist()
V3 = df[Columns[29]].tolist()
V4 = df[Columns[30]].tolist()
V5 = df[Columns[31]].tolist()
V6 = df[Columns[32]].tolist()
A1 = df[Columns[33]].tolist()
A2 = df[Columns[34]].tolist()
A3 = df[Columns[35]].tolist()
A4 = df[Columns[36]].tolist()
A5 = df[Columns[37]].tolist()
A6 = df[Columns[38]].tolist()
A7 = df[Columns[39]].tolist()
A8 = df[Columns[40]].tolist()

##########################################################################################
# Simulator

# Constants
m = 11.5                    # Kg
W = m*9.81                  # Newton
B = 114.8                   # Newton
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
    return Combine4(np.transpose(R.from_quat(q).as_matrix())*Kx, Zero3x3, Zero3x3, c*np.diag([2,2,1]))


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
_M12rb = np.array([[0, m*r_g[2][0], 0], [-m*r_g[2][0], 0, 0], [0, 0, 0]])
_M21rb = np.array([[0, -m*r_g[2][0], 0], [m*r_g[2][0], 0, 0], [0, 0, 0]])
# _M12rb = Zero3x3
# _M21rb = Zero3x3
_M22rb = np.array([[I_x, 0, 0], [0, I_y, 0], [0, 0, I_z]])
_M11A = np.array([[X_udot, 0, 0], [0, Y_vdot, 0], [0, 0, Z_wdot]])
_M12A = Zero3x3
_M21A = Zero3x3
_M22A = np.array([[K_pdot, 0, 0], [0, M_qdot, 0], [0, 0, N_rdot]])
Mrb = Combine4(_M11rb, _M12rb, _M21rb, _M22rb)
MA = -Combine4(_M11A,_M12A,_M21A,_M22A)
M = Mrb + MA
_M11, _M12, _M21, _M22 = split(M,3,3)


# os.remove('C:\Users\elias\Documents\Debugging\v.txt')
# os.remove('C:\Users\elias\Documents\Debugging\q.txt')
# os.remove('C:\Users\elias\Documents\Debugging\x.txt')

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
    vDot = np.linalg.inv(M)@(-C(v)@v-D(v)@v-Kd@v-Kp(q)@z)
    xDot = np.concatenate([zetaDot, vDot])
    if t > 1:
        return xDot
    else:
        return np.array([0]*13)


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
x_d = np.array([0, 0,0])
q_d = np.array(get_quaternion_from_euler(0,0,90))
################## Simulation and plotting ##################################################################################### 
start = 125
stop = -1
t = np.linspace(0, (len(Time[:stop]) - len(Time[:start])) * 0.025, len(Time[start:stop]))
Kx = 0
c = 20

 
Kd = 10*np.diag([1]*6)
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



# print(len(Time[:start]) * 0.025)
# print(len(Time[:stop]) * 0.025)
print(len(Time))

fig, axs = plt.subplots(2, 2)
axs[0,0].plot(t, Q_D4[start:stop],linestyle='dashed',color = 'r')
axs[0,1].plot(t, Q_D1[start:stop],linestyle='dashed',color = 'r')
axs[1,0].plot(t, Q_D2[start:stop],linestyle='dashed',color = 'r')
axs[1,1].plot(t, Q_D3[start:stop],linestyle='dashed',color = 'r')
axs[0,0].plot(t, eta[0],linestyle='dotted',color = 'g')
axs[0,1].plot(t, epsilon1[0],linestyle='dotted',color = 'g')
axs[1,0].plot(t, epsilon2[0],linestyle='dotted',color = 'g')
axs[1,1].plot(t, epsilon3[0],linestyle='dotted',color = 'g')
axs[0,0].plot(t, Q4[start:stop])
axs[0,1].plot(t, Q1[start:stop])
axs[1,0].plot(t, Q2[start:stop])
axs[1,1].plot(t, Q3[start:stop])
axs[0,0].set_title('η')
axs[0,1].set_title(r'$ε_{1}$')
axs[1,0].set_title(r'$ε_{2}$')
axs[1,1].set_title(r'$ε_{3}$')
for i in range(2):
    axs[0,i].grid()
    axs[1,i].grid()
    axs[0,i].set_xlabel('t [s]')
    axs[1,i].set_xlabel('t [s]')
    axs[0,i].set_ylim([-1, 1])
    axs[1,i].set_ylim([-1, 1])
#plt.subplots_adjust(left = 0.033, bottom = 0.05, right = 0.99, top = 0.933, wspace = 0.2, hspace = 0.24) #for fullscreen
plt.subplots_adjust(left = 0.069, bottom = 0.057, right = 0.985, top = 0.933, wspace = 0.2, hspace = 0.24) #for set_size 8,8
fig = plt.gcf()  # get current figure
fig.set_size_inches(8, 8)
fig.legend(['Reference','Simulated', 'Measured'])
plt.show()
#plt.savefig('C:/Users/Elias/OneDrive - NTNU/Studier/3. år/2. semester/IELET2920/Simulering/{}.pdf'.format(save_title))

