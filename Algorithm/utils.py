import math
import matplotlib.pyplot as plt

POSX = 0
POSY = 1
YAW = 2
SPD = 3
YAWSPD = 4

def plot_arrow(x, y, yaw, length=5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


def plot(ltraj, state, goal, ob):
    plt.cla()
    ax = plt.gca()
    if ltraj:
        data_x = [s[0] for s in ltraj]
        data_y = [s[1] for s in ltraj]
        plt.plot(data_x, data_y, color="red", linewidth=1)
    plt.plot(state[POSX], state[POSY], "xr")
    plt.plot(goal[0], goal[1], "xb", label='target')
    plt.plot(ob[:, 0], ob[:, 1], "ok", label='obstacle')
    ax.legend(loc='best')

    for i in range(len(ob)):
        circle = plt.Circle((ob[i, 0], ob[i, 1]), 3, color='blue', Fill=False)
        plt.text(ob[i, 0], ob[i, 1], 'No. %s' % i, family='serif', style='italic', ha='right', wrap=True)
        ax.add_patch(circle)

    plot_arrow(state[POSX], state[POSY], state[YAW])
    plt.axis("equal")
    plt.grid(True)
    plt.pause(0.0001)


def plot_traj(traj):
    plt.scatter([s[0] for s in traj], [s[1] for s in traj], c='r', s=1)
    plt.axis("equal")
    plt.grid(True)
    plt.pause(0.0001)
    plt.show()
