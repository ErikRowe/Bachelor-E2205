import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
from scipy.integrate import odeint

# Eq: Mṽ + C(v)v +D(v)v + g(mu) + g_0 = Tau + Tau_wind + Tau_wave # might be wrong

class Simulator():
    Mat6x6 = np.zeros((6,6))
    Mat3x3 = np.zeros((3,3))
    Mat4x3 = np.zeros((4,3))

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
    I_0 = np.array([[I_x],[I_y],[I_z]]) * np.identity(3)

    X_udot = -5.5               # Kg
    Y_vdot = -12.7              # Kg
    Z_wdot = -14.57             # Kg
    K_pdot = -0.12              # Kg m**2/rad
    M_qdot = -0.12              # Kg m**2/rad
    N_rdot = -0.12              # Kg m**2/rad
    added_mass_matrix = np.transpose(np.array([[X_udot, Y_vdot, Z_wdot, K_pdot, M_qdot, N_rdot]]))

    X_u = -4.03                 # Ns/m
    N_r = -0.07                 # Ns/m
    Y_v = -6.22                 # Ns/m
    K_p = -0.07                 # Ns/m
    M_q = -0.07                 # Ns/m
    Z_w = -5.18                 # Ns/m

    X_u_abs = -18.18            # Ns**2/m**2
    Y_v_abs = -21.66            # Ns**2/m**2
    Z_w_abs = -36.99            # Ns**2/m**2
    K_p_abs = -1.55             # Ns**2/m**2
    M_q_abs = -1.55             # Ns**2/m**2
    N_r_abs = -1.55             # Ns**2/m**2



    def __init__(self):
        pass

    def create_s_mat(self, vector): # Returns a scew symmetric matrix from 3d vector
        s = np.array([[    0     , -vector[2],  vector[1]],
                      [ vector[2],    0      , -vector[0]],
                      [-vector[1], vector[0] ,    0     ]])
        return s

    def create_m_mat(self):
        mat_m_rb = np.array([[self.m * np.identity(3), - self.m * self.create_s_mat(self.r_g)], # 2x3
                             [self.m * self.create_s_mat(self.r_g), self.I_0]])
        test = np.array(np.concatenate((self.m * np.identity(3), - self.m * self.create_s_mat(self.r_g)), axis=1))
        print(test)
        mat_m_a = self.added_mass_matrix * np.identity(6)
        mat_m = mat_m_rb + mat_m_a
        return mat_m

    def create_c_mat(self):
        pass

    def create_d_mat(self):
        pass

    def create_r_mat(self):
        pass
    
        

if __name__ == "__main__":
    sim = Simulator()
    t = sim.create_m_mat()