import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.integrate import odeint

# Eq: Mṽ + C(v)v +D(v)v + g(mu) + g_0 = Tau + Tau_wind + Tau_wave # might be wrong

class Simulator():
    Mat6x6 = np.zeros((6,6))
    Mat3x3 = np.zeros((3,3))

    def __init__(self):
        pass

    def create_S_MAT(self, matrix): # Returns a scew symmetric matrix from 3d vector
        S = np.array([[    0     , -matrix[2],  matrix[1]],
                      [ matrix[2],    0      , -matrix[0]],
                      [-matrix[1], matrix[0] ,    0     ]])
        return S

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
    sim = Simulator()
    print(sim.create_S_MAT(sim.Mat3x3))