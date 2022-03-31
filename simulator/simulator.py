import numpy as np
from scipy.integrate import odeint

# Eq: Mṽ + C(v)v +D(v)v + g(mu) + g_0 = Tau + Tau_wind + Tau_wave # might be wrong


def  inertiaMatrix(): # System inertia matrix
    _m = np.array([[0],
                  [0],
                  [0],
                  [0],
                  [0],
                  [0]])
    return _m

def coriolisMatrix(): # Matrix of coriolis en sentrifugal terms
    _c = np.array([])
    return _c

def dampingMatrix(): # Matrix of dissipated damping terms
    _d = np.array([])
    return _d

def disturbances():
    _tau = np.array([[],
                    [],
                    [],
                    [],
                    [],
                    []])
    return _tau

def movementVector(x_n, y_n, z_n, phi_N, theta_n, psi_n):
    _v = np.array([[x_n],
                  [y_n],
                  [z_n],
                  [phi_N],
                  [theta_n],
                  [psi_n]])
    return _v

def blueROV2Heavy():
    M_rigid_body = 0
    v_dot = 0
    C_rigid_body = 0
    v = 0

    M_a = 0
    v_world = 0
    C_a = 0
    D_world = 0
    g_nu = 0
    tau = 0
    dydt = M_rigid_body * v_dot + C_rigid_body * v + M_a * v_world + C_a * v_world + D_world * v_world * g_nu - tau

if __name__ == "__main__":
    blueROV2Heavy()