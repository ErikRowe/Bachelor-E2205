import csv
from time import time
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

### README
# To change dataset to plot, change path in df

df = pd.read_csv('/home/elias/bs_ws/Data Logging/vanilla_ROV/new/90yaw.xlsx',skiprows=1)

start = 0
stop = -1

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

t = np.linspace(0, len(Time[start:stop]) * 0.025, len(Time[start:stop]))

fig, axs = plt.subplots(2, 4, figsize=(8, 5))
axs[0,0].plot(t, Q4[start:stop])
axs[0,1].plot(t, Q1[start:stop])
axs[0,2].plot(t, Q2[start:stop])
axs[0,3].plot(t, Q3[start:stop])
axs[1,0].plot(t, Q_D4[start:stop])
axs[1,1].plot(t, Q_D1[start:stop])
axs[1,2].plot(t, Q_D2[start:stop])
axs[1,3].plot(t, Q_D3[start:stop])
axs[0,0].set_title('w')
axs[0,1].set_title('x')
axs[0,2].set_title('y')
axs[0,3].set_title('z')
axs[1,0].set_title('w_d')
axs[1,1].set_title('x_d')
axs[1,2].set_title('y_d')
axs[1,3].set_title('z_d')
for i in range(4):
    axs[0,i].grid()
    axs[1,i].grid()
    axs[0,i].set_xlabel('t [s]')
    axs[1,i].set_xlabel('t [s]')
    axs[0,i].set_ylim([-1, 1])
    axs[1,i].set_ylim([-1, 1])
plt.show()