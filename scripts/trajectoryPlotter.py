import os
import matplotlib.pyplot as plt

file_path = os.path.join(os.pardir,'Project_WorkCell','trajectories','trajectory.dat')

t = []
x = []
y = []
z = []




with open(file_path) as f:
    for l in f.readlines():
        data_l = l.strip().split(' ')
        ts = float(data_l[0])
        xs = float(data_l[1])
        ys = float(data_l[2])
        zs = float(data_l[3])

        t.append(ts)
        x.append(xs)
        y.append(ys)
        z.append(zs)

fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')
ax.scatter(x,y,z)

plt.show()
