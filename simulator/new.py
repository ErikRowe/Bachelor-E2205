import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
from scipy.integrate import odeint

####################### Hvordan definere vektorer #######################
#   Alle funksjoner og matriser er hentet fra 
#   Quaternion feedback regulation of underwater vehicles av Fossen and Fjelstad
#   Dersom vektoren er definert som en 4x1 kolonnevektor, defineres den her som 4x1     
#   Feks: 
#   q = np.array([[n], [e1], [e2], [e3]])
#   gir en vektor på formen
#   q = [n
#        e1
#        e2
#        e3]
#   Det samme gjelder da for en 1x4 radvektor
#





class Simulator():
    # Constants
    m = 11.5                    # Kg
    W = 112.8                   # Newton
    B = 114.8                   # Newtons
    r_b = np.array([[0], [0], [0]])
    r_g = np.array([[0], [0], [0.02]])
    
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
    linear_dampening_matrix = np.transpose(np.array([[X_u, Y_v, Z_w, K_p, M_q, N_r]]))

    X_u_abs = -18.18            # Ns**2/m**2
    Y_v_abs = -21.66            # Ns**2/m**2
    Z_w_abs = -36.99            # Ns**2/m**2
    K_p_abs = -1.55             # Ns**2/m**2
    M_q_abs = -1.55             # Ns**2/m**2
    N_r_abs = -1.55             # Ns**2/m**2
    quadratic_dampening_matrix_1 = np.transpose(np.array([[X_u_abs,Y_v_abs,Z_w_abs,K_p_abs,M_q_abs,N_r_abs]]))

    #   Testing variables
    q = np.array([[1], [0], [1], [0]])
    
    #   Inital conditions
    x_init = np.array([[0], [0], [0]])
    q_init = np.array([[1], [0.5], [0.5], [0.75]])
    nu_init = np.array([[1], [1], [1], [1], [1], [1]])
    #zeta0 = np.array([[x_init], [q_init], [nu_init]])
    #TODO: FIKS zeta0
    



    #   Functions for making the required matrices
    def split(self, array, nrows, ncols):
        """Split a matrix into sub-matrices."""

        r, h = array.shape
        return (array.reshape(h//nrows, nrows, -1, ncols)
                    .swapaxes(1, 2)
                    .reshape(-1, nrows, ncols))
    
    def create_u_mat(self,q):
        q_real = q[0]
        q_imag = np.array([[q[1][0],q[2][0],q[3][0]]])
        u_bottom = q_real*np.identity(3) + self.create_s_mat(q[1:])
        u = np.concatenate((-q_imag, u_bottom))
        return u
        
    def create_m_mat(self):
        mat_m_rb = np.array(np.concatenate((np.concatenate((self.m * np.identity(3), - self.m * self.create_s_mat(self.r_g)), axis=1),
                                            np.concatenate((self.m * self.create_s_mat(self.r_g), self.I_0), axis=1)))) # 6x6
        mat_m_a = self.added_mass_matrix * np.identity(6) # 6x6
        mat_m = mat_m_rb + mat_m_a
        return mat_m

    def create_c_mat(self, mat_m, nu):
        m11, m12, m21, m22 = self.split(mat_m, 3, 3)
        mat_c_rb = 0
        mat_c_a = np.array(np.concatenate((np.concatenate((np.zeros((3,3)), - self.create_s_mat(m11 * nu[1] + m12 * nu[2])), axis=1),
                                            np.concatenate((- self.create_s_mat(m11 * nu[1] + m12 * nu[2]), self.create_s_mat(m22 * nu[1] + m21 * nu[2])), axis=1))))
        mat_c = mat_c_rb + mat_c_a
        return mat_c

    #   Functions used for quaternion math
    def create_s_mat(self, vector): # Returns a scew symmetric matrix from 3d vector(3x1)
        s = np.array([[    0     , -vector[2][0],  vector[1][0]],
                      [ vector[2][0],    0      , -vector[0][0]],
                      [-vector[1][0], vector[0][0] ,    0     ]])
        return s
    
    def conjugate(self,q):
        return np.array([q[0], -q[1], -q[2], -q[3]])

    def signum(self,a):
        return 1 if a >=0 else -1

    def quaternion_multiplication(self, q1, q2):
        M11 = np.array([q1[0][0]])
        M12 = np.array([-q1[1][0], -q1[2][0], -q1[3][0]])
        M21 = np.array([q1[1], q1[2], q1[3]])
        e = np.array([q1[1], q1[2], q1[3]])
        M22 = q1[0]*np.identity(3) + self.create_s_mat(q1[1:])
        M1 = np.concatenate((M11, M12))
        M2 = np.concatenate((M21, M22),axis = 1)
        M = np.array([M1, M2[0], M2[1], M2[2]])
        return M@q2


    def main(self):
        m_mat = self.create_m_mat()
        c = self.create_c_mat(m_mat,self.nu_init)
        print(c)
        #TODO FIX C MATRISEN
        return 0
        
        


if __name__ == "__main__":
    sim = Simulator()
    sim.main()