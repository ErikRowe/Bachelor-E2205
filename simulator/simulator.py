import warnings
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
from scipy.integrate import odeint

class Simulator():        
    Kxp = 30   # Scaling of linear proportional gain
    cp = 200    # Scaling constant for angular proportional gain
    Kd = np.identity(6)*1

    x_init = np.array([10, 10, 10])    # Position
    q_init = np.array([0.5]*4)  # Quaternion representing orientation
    nu_init = np.array([0]*6)   # [v, w] linear and angular velocities
    zeta0 = np.concatenate([x_init, q_init, nu_init])
    t = np.linspace(0,5,100)
    x_d = np.array([0,0,0])
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
    quadratic_dampening_matrix = np.transpose(np.array([[X_u_abs,Y_v_abs,Z_w_abs,K_p_abs,M_q_abs,N_r_abs]]))



    def __init__(self):
        self.M_matrix = self.create_m_mat() # Inertia matrix

    def create_jacobi_mat(self, q):
        u = self.create_u_mat(q) # Coordinate transformation matrix
        m11 = Rot.from_quat(q).as_matrix()
        m12 = np.zeros((3,3))
        m21 = np.zeros((4,3))
        m22 = 0.5 * u
        j = self.concatenate_4_mat(m11,m12,m21,m22)
        return j

    def create_s_mat(self, vector): # Returns a scew symmetric matrix from 3d vector(3x1)
        s = np.array([[    0        , -vector[2][0],  vector[1][0]],
                      [ vector[2][0],    0         , -vector[0][0]],
                      [-vector[1][0], vector[0][0] ,    0     ]])
        return s

    def create_m_mat(self):
        m_rb11 = self.m * np.identity(3)
        m_rb12 = - self.m * self.create_s_mat(self.r_g)
        m_rb21 = self.m * self.create_s_mat(self.r_g)
        m_rb22 = self.I_0
        mat_m_rb = self.concatenate_4_mat(m_rb11, m_rb12, m_rb21, m_rb22)
        mat_m_a = self.added_mass_matrix * np.identity(6) # 6x6
        mat_m = mat_m_rb + mat_m_a
        return mat_m

    def create_c_mat(self, mat_m, nu):
        v = np.array(np.transpose([nu[:3]]))
        w = np.array(np.transpose([nu[3:]]))
        
        m_rb11 = np.zeros((3,3))
        m_rb12 = - self.m * self.create_s_mat(v) - self.m * self.create_s_mat(self.create_s_mat(w)*self.r_g)
        m_rb21 = m_rb12
        m_rb22 = - self.create_s_mat(self.I_0 * w) + self.m * self.create_s_mat(self.create_s_mat(v)*self.r_g)
        mat_c_rb = self.concatenate_4_mat(m_rb11, m_rb12, m_rb21, m_rb22)
        
        
        m11, m12, m21, m22 = self.split(mat_m, 3, 3)
        m_a11 = np.zeros((3,3))
        m_a12 = - self.create_s_mat(m11 * v + m12 * w)
        m_a21 = - self.create_s_mat(m11 * v + m12 * w)
        m_a22 = self.create_s_mat(m22 * v + m21 * w)
        mat_c_a = self.concatenate_4_mat(m_a11, m_a12, m_a21, m_a22)
        mat_c = mat_c_rb + mat_c_a
        return mat_c

    def create_d_mat(self, nu):
        mat_d_l = -self.linear_dampening_matrix * np.identity(6)
        mat_d_q = -(self.quadratic_dampening_matrix * np.identity(6))@(abs(np.transpose(np.array([[nu[0], nu[1], nu[2], nu[3], nu[4], nu[5]]]))*np.identity(6)))
        mat_d = mat_d_l + mat_d_q
        return mat_d

    def create_u_mat(self, q):
        real, imag = self.split_real_imag(q)
        imag = np.transpose([imag])
        u_bottom = real*np.identity(3) + self.create_s_mat(np.transpose([q]))
        self.t_q = u_bottom
        u = np.concatenate((- np.transpose(imag), u_bottom))
        return u

    def create_z_mat(self, x, q):
        q_T = (np.transpose(q))
        real, imag = self.split_real_imag(q_T[0])
        top = np.transpose([x])
        bot = self.signum(real[0]) * np.transpose([imag])
        return np.concatenate((top, bot))

    def quaternion_multiplication(self, q1, q2):
        # q1
        q1_real, q1_imag = self.split_real_imag(q1)
        m11 = q1_real
        m12 = - q1_imag
        m21 = np.transpose([q1_imag])
        m22 = q1[0] * np.identity(3) + self.create_s_mat(np.transpose([q1_imag]))
        top = np.array([np.concatenate((m11,m12),axis=0)])
        bot = np.concatenate((m21,m22),axis=1)
        q_1 = np.concatenate((top, bot))
        
        # q2
        q2_real, q2_imag = self.split_real_imag(q2)
        v1 = np.array([q2_real])
        v2 = np.transpose([q2_imag])
        q_2 = np.concatenate((v1,v2))
        return q_1@q_2

    def split(self, array, nrows, ncols):
        """Split a matrix into sub-matrices."""
        r, h = array.shape
        return (array.reshape(h//nrows, nrows, -1, ncols)
                    .swapaxes(1, 2)
                    .reshape(-1, nrows, ncols))

    def conjugate(self,q):
        return np.array([q[0], -q[1], -q[2], -q[3]])

    def signum(self, x):
        if x < 0: x == -1
        if x >= 0: x == 1
        return x

    def concatenate_4_mat(self, a11, a12, a21, a22):
        return np.array(np.concatenate((np.concatenate((a11, a12), axis=1),
                                        np.concatenate((a21, a22), axis=1))))

    def split_real_imag(self, quaternion):
        real = np.array([quaternion[0]])
        imag = np.array([quaternion[1], quaternion[2], quaternion[3]])
        return real, imag

    def Kp(self, q):
        m11 = Rot.from_quat(q).as_matrix() * self.Kxp
        m12 = np.zeros((3,3))
        m21 = m12
        m22 = self.cp* np.identity(3)
        return self.concatenate_4_mat(m11,m12,m21,m22)

    def simulate(self, state, t):
        x = state[:3]
        q = state[3:7]/np.linalg.norm(state[3:7])
        nu = state[7:]

        q_tilde = self.quaternion_multiplication(self.conjugate(self.q_d),q)
        q_tilde /= np.linalg.norm(q_tilde)  # = q_tilde/norm(q_tilde) self.normalize(q_tilde)

        x_tilde = x - self.x_d      # x_tilde = current_x - desired_x
        z = self.create_z_mat(x_tilde, q_tilde)    # Error vector
        c = self.create_c_mat(self.M_matrix, nu)    # Coriolis and Centrifugal matrix
        d = self.create_d_mat(nu)   # Dampening matrix
        # dd = self.D(nu)

        nu = np.transpose([nu])

        # epsilon_dot
        jacobi = self.create_jacobi_mat(q) # 
        epsilon_dot = jacobi@nu # @np.transpose([nu])

        # nu_dot
        Kp = self.Kp(q)        
        nu_dot = np.linalg.inv(self.M_matrix)@(-self.Kd@nu - Kp@z - c@nu - d@nu)
        
        return np.transpose(np.concatenate((epsilon_dot, nu_dot)))[0] #np.concatenate([epsilon_dot, nu_dot])

    def main(self):
        initial_conditions = self.zeta0
        t = np.linspace(0,15,1000)
        # sim = self.simulate(initial_conditions, t)
        simulation = odeint(self.simulate,initial_conditions, t)

        x = simulation[:, 0]
        y = simulation[:, 1]
        z = simulation[:, 2]

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
            ax.set_ylim([-5, 15])
        plt.show()
        

sim2 = Simulator() 
initial = sim2.zeta0
t = np.linspace(0,15,1000)
ode = odeint(sim2.simulate, initial, t)

x = ode[:,0]
y = ode[:,1]
z = ode[:,2]

fig, axs = plt.subplots(1,3, figsize = (12,3))
axs[0].set_ylabel('x [m]')
axs[1].set_ylabel('y [m]')
axs[2].set_ylabel('z [m]')
axs[0].plot(t, x)
axs[1].plot(t, y)
axs[2].plot(t, z)

for ax in axs:
    ax.grid()
    ax.set_xlabel('t [s]')
    ax.set_ylim([-5, 15])
plt.show()

# if __name__ == "__main__":
#     sim = Simulator()
#     #sim.main()