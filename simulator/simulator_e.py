import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
from scipy.integrate import odeint

# Eq: Mṽ + C(v)v +D(v)v + g(mu) + g_0 = Tau + Tau_wind + Tau_wave # might be wrong

class Simulator():
    Mat6x6 = np.zeros((6,6))
    Mat3x3 = np.zeros((3,3))
    Mat4x3 = np.zeros((4,3))

    Kxp = 30
    cp = 200
    Kd = np.identity(6)

    x_init = np.array([0, 0, 0])
    q_init = np.array([1] + [0]*3)
    nu_init = np.array([0]*6)
    zeta0 = np.concatenate([x_init, q_init, nu_init])
    t = np.linspace(0,5,100)
    x_d = np.array([10,10,10])
    q_d = np.array([1] + [0]*3)

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
    linear_dampening_matrix = np.transpose(np.array([[X_u, Y_v, Z_w, K_p, M_q, N_r]]))

    X_u_abs = -18.18            # Ns**2/m**2
    Y_v_abs = -21.66            # Ns**2/m**2
    Z_w_abs = -36.99            # Ns**2/m**2
    K_p_abs = -1.55             # Ns**2/m**2
    M_q_abs = -1.55             # Ns**2/m**2
    N_r_abs = -1.55             # Ns**2/m**2
    quadratic_dampening_matrix_1 = np.transpose(np.array([[X_u_abs,Y_v_abs,Z_w_abs,K_p_abs,M_q_abs,N_r_abs]]))

    def __init__(self):
        pass

    def create_s_mat(self, vector): # Returns a scew symmetric matrix from 3d vector(4x1)
        s = np.array([[    0     , -vector[2][0],  vector[1][0]],
                      [ vector[2][0],    0      , -vector[0][0]],
                      [-vector[1][0], vector[0][0] ,    0     ]])
        return s

    def create_m_mat(self):
        mat_m_rb = np.array(np.concatenate((np.concatenate((self.m * np.identity(3), - self.m * self.create_s_mat(self.r_g)), axis=1),
                                            np.concatenate((self.m * self.create_s_mat(self.r_g), self.I_0), axis=1)))) # 6x6
        mat_m_a = self.added_mass_matrix * np.identity(6) # 6x6
        mat_m = mat_m_rb + mat_m_a
        return mat_m

    def create_c_mat(self, mat_m, nu):
        m11, m12, m21, m22 = self.split(mat_m, 3, 3)
        mat_c_rb = 0
        mat_c_a = np.array(np.concatenate((np.concatenate((np.zeros((3,3)), - self.create_s_mat(m11 * nu[0][1] + m12 * nu[0][2])), axis=1),
                                            np.concatenate((- self.create_s_mat(m11 * nu[0][1] + m12 * nu[0][2]), self.create_s_mat(m22 * nu[0][1] + m21 * nu[0][2])), axis=1))))
        mat_c = mat_c_rb + mat_c_a
        return mat_c

    def create_d_mat(self,nu):
        mat_d_l = -self.linear_dampening_matrix * np.identity(6)
        mat_d_q = -(self.quadratic_dampening_matrix_1 * np.identity(6))@(abs(np.transpose(np.array([[nu[0][0], nu[0][1], nu[0][2], nu[0][3], nu[0][4], nu[0][5]]]))*np.identity(6)))
        mat_d = mat_d_l + mat_d_q
        return mat_d

    def create_r_mat(self):
        pass
    
    def create_u_mat(self,q):
        q_real = q[0]
        q_imag = np.array([[q[1]],[q[2]],[q[3]]])
        u_bottom = q_real*np.identity(3) + self.create_s_mat(q)
        u = np.concatenate((np.transpose(q_imag), u_bottom))
        return u

    def quaternion_multiplication(self, q1, q2):
        M11 = np.array([q1[0][0]])
        M12 = np.array([-q1[0][1], -q1[0][2], -q1[0][3]])
        M21 = np.array([[q1[0][1]], [q1[0][2]], [q1[0][3]]])
        M22 = q1[0][0]*np.identity(3) + self.create_s_mat(np.transpose(q1))
        M1 = np.concatenate((M11, M12))
        M2 = np.concatenate((M21, M22),axis = 1)
        M = np.array([M1, M2[0], M2[1], M2[2]])
        return M@np.transpose(q2)
    
    def signum(self,a):
        return 1 if a >=0 else -1

    def conjugate(self,q):
        return np.array([[q[0], -q[1], -q[2], -q[3]]])

    def Kp(self,q):
        kp11 = np.transpose(Rot.from_quat(q[0]).as_matrix()*self.Kxp)
        kp12 = self.Mat3x3
        kp21 = self.Mat3x3
        kp22 = self.cp*np.identity(3)      
        return np.concatenate((np.concatenate((kp11,kp12),axis = 1),np.concatenate((kp21,kp22),axis = 1)))
    
    def diff_equations(self, state,t):
        x = state[:3]
        q = state[3:7]
        nu = state[7:]
        z_x = x - self.x_d
        q_tilde = self.quaternion_multiplication(self.conjugate(self.q_d),q)
        q_tilde /= np.linalg.norm(q_tilde)
        z_q = self.signum(q_tilde[0])*np.array([q_tilde[1], q_tilde[2], q_tilde[3]])
        z = np.concatenate([z_x,z_q])
        J11 = np.concatenate([Rot.from_quat(q).as_matrix(), self.Mat3x3], axis=1)
        J21 = np.concatenate([self.Mat4x3,1/2*self.create_u_mat(q)], axis = 1)
        J = np.concatenate([J11,J21])
        zeta_dot = J@nu
        M = self.create_m_mat()
        C = self.create_c_mat(self.m,nu)
        D = self.create_d_mat(nu)
        Kp = self.Kp(q)
        nu_dot = np.linalg.inv(M)@(-C(nu)@nu - D(nu)@nu - self.Kd@nu - Kp(q)@z)
        return np.concatenate([zeta_dot,nu_dot])
    

    def main(self):
        print(self.diff_equations(self.zeta0,self.t))        
        


    def split(self, array, nrows, ncols):
        """Split a matrix into sub-matrices."""

        r, h = array.shape
        return (array.reshape(h//nrows, nrows, -1, ncols)
                    .swapaxes(1, 2)
                    .reshape(-1, nrows, ncols))



if __name__ == "__main__":
    sim = Simulator()
    sim.main()