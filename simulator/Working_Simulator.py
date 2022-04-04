import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

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

def D(v):
    D1 = np.diag([70, 100, 100, 30, 50, 50])
    D2 = np.diag([100*abs(v[0]), 200*abs(v[1]), 200*abs(v[2]),
                  50*abs(v[3]), 100*abs(v[4]), 100*abs(v[5])])
    return D1 + D2

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

_M11 = np.array([[215, 0, 0], [0, 265, 0], [0, 0, 265]])
_M12 = Zero3x3
_M21 = Zero3x3
_M22 = np.array([[40, 0, 0], [0, 80, 0], [0, 0, 80]])
M = Combine4(_M11, _M12, _M21, _M22)



# Python program to implement Runge Kutta method
def f(zeta):
    global M
    x = zeta[:3]
    q = zeta[3:7]/np.linalg.norm(zeta[3:7])
    v = zeta[7:]
    z1 = x - x_d
    q_tilde = quatMul(conjugate(q_d), q)
    q_tilde /= np.linalg.norm(q_tilde)
    z2 = sign(q[0])*np.array([q_tilde[1], q_tilde[2], q_tilde[3]])
    z = np.concatenate([z1, z2])
    zeta11 = np.concatenate([R.from_quat(q).as_matrix(), Zero3x3], axis=1)
    zeta22 = np.concatenate([Zero4x3, 1/2*U(q)], axis=1)
    zetaDot = np.concatenate([zeta11, zeta22])@v
    vDot = np.linalg.inv(M)@(-C(v)@v-D(v)@v-Kd@v-Kp(q)@z)
    return np.concatenate([zetaDot, vDot])

# Finds value of y for a given x using step size h
# and initial value y0 at x0.

def rungeKutta(x0, y0, x, h):
    global Y, X, Z, T
    # Count number of iterations using step size or
    # step height h
    n = (int)((x - x0)/h)
    # Iterate for number of iterations
    y = y0
    Y = np.append(Y, y[0])
    X = np.append(X, y[1])
    Z = np.append(Z, y[2])
    T = np.append(T, x0)
    
    for i in range(1, n + 1):
        "Apply Runge Kutta Formulas to find next value of y"
        k1 = h * f(y)
        k2 = h * f(y + 0.5 * k1)
        k3 = h * f(y + 0.5 * k2)
        k4 = h * f(y + k3)

        # Update next value of y
        y = y + (1.0 / 6.0)*(k1 + 2 * k2 + 2 * k3 + k4)

        # Update next value of x
        x0 = x0 + h
        Y = np.append(Y, y[0])
        X = np.append(X, y[1])
        Z = np.append(Z, y[2])
        T = np.append(T, x0)
    return y


X = np.array([])
Y = np.array([])
Z = np.array([])
T = np.array([])

c = 200
Kd = np.diag([1]*6)
Kx = 30

x_init = np.array([10, 10, 10])
q_init = np.array([0.5]*4)
v_init = np.array([0]*6)
zeta0 = np.concatenate([x_init, q_init, v_init])

x_d = np.array([0, 0, 0])
q_d = np.array([1] + [0]*3)

# Driver method
x0 = 0
y = zeta0
x = 30
h = 0.25
rungeKutta(x0, y, x, h)

# fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
# ax.set_xlabel('x [m]')
# ax.set_ylabel('y [m]')
# ax.set_zlabel('z [m]')
# ax.plot3D(X, Y, Z)

fig, axs = plt.subplots(1, 3, figsize=(12, 3))
axs[0].set_ylabel('x [m]')
axs[1].set_ylabel('y [m]')
axs[2].set_ylabel('z [m]')
axs[0].plot(T, X)
axs[1].plot(T, Y)
axs[2].plot(T, Z)

for ax in axs: 
    ax.grid()
    ax.set_xlabel('t [s]')
    ax.set_ylim([-5, 10])
plt.show()
