import numpy as np

# Eq: Mṽ + C(v)v +D(v)v + g(mu) + g_0 = Tau + Tau_wind + Tau_wave # might be wrong

tau = 0
tau_wind = 0
tau_wave = 0

def  inertiaMatrix(): # System inertia matrix
    M = np.array([[0],
                  [0],
                  [0],
                  [0],
                  [0],
                  [0]])
    return M

def coriolisMatrix(): # Matrix of coriolis en sentrifugal terms
    C = np.array([])
    return C

def dampingMatrix(): # Matrix of dissipated damping terms
    D = np.array([])
    return D

def movementVector(x_n, y_n, z_n, phi_N, theta_n, psi_n):
    v = np.array([x_n, y_n, z_n, phi_N, theta_n, psi_n]) # Might need another []
    return v


if __name__ == "__main__":
    v = movementVector(1,1,1,1,1,1)
    print(v)