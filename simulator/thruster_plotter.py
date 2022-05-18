from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt


lengths = np.array([
              [0.156, 0.111, 0.085],
              [0.156, -0.111, 0.085],
              [-0.156, 0.111, 0.085],
              [-0.156, -0.111, 0.085],
              [0.12, 0.218, 0],
              [0.12, -0.218, 0],
              [-0.12, 0.218, 0],
              [-0.12, -0.218, 0]])

names = np.array(["T1","T2","T3","T4","T5","T6","T7","T8"])
angles = np.array([-45, 45, -135, 135, 0, 0, 0, 0, 0])
directions = np.array([1, 1, -1, -1, 1, -1, -1, 1])


fig = plt.figure(figsize=(5,5))
ax = fig.add_subplot(111, projection='3d')

for i in range(4):
    point = lengths[i]
    x, y, z = point[0],-point[1],-point[2]
    ax.text(x,y,z, names[i], color='black')
    #ax.scatter(x, y, z, label = names[i])
    if angles[i] == -45:
        u = 1
        v = 1
    if angles[i] == 45:
        u = 1
        v = -1
    if angles[i] == -135:
        u = 1
        v = -1
    if angles[i] == 135:
        u = 1
        v = 1
    w = 0
    ax.quiver(x, y, z, u, v, w, length = 0.1, normalize = True)
for i in range(4,8):
    point = lengths[i]
    x, y, z = point[0],-point[1],-point[2]
    ax.text(x,y,z, names[i], color='black')
    #ax.scatter(x, y, z, label = names[i])
    u = 0
    v = 0
    w = directions[i]
    ax.quiver(x, y, z, u, v, w, length = 0.1, normalize = True)


ax.set_xlim([-0.2,0.2])
ax.set_ylim([-0.2,0.2])
ax.set_zlim([-0.2,0.2])
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_zticklabels([])

ax.quiver(0,0,0,1,0,0, length = .2, linewidth=2, color = "r")
ax.quiver(0,0,0,0,-1,0, length = .2, linewidth=2, color = "b")
ax.quiver(0,0,0,0,0,-1, length = .2, linewidth=2, color = "g")
ax.text(.25,0,0, 'x', color='r')
ax.text(0,-.25,0, 'y', color='b')
ax.text(0,0,-.25, 'z', color='g')
plt.show()


