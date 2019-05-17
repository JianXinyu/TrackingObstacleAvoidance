"""
Mobile robot motion planning sample with Dynamic Window Approach
author: Atsushi Sakai (@Atsushi_twi)
"""
from PID import PID
import math
import numpy as np
import matplotlib.pyplot as plt

import sys
sys.path.append("../../")


show_animation = True


class Config():
    # simulation parameters

    def __init__(self):
        # robot parameter
        self.max_speed = 1.0  # [m/s]
        self.min_speed = 0  # [m/s]
        self.max_yawrate = 40.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_dyawrate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_reso = 0.05  # [m/s]
        self.yawrate_reso = 5 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s]
        self.dT = 1.45  # [s]
        self.predict_time = 6.0  # [s]
        self.to_goal_cost_gain = 0.01 # 1.0
        self.speed_cost_gain = 0.5
        self.robot_radius = 3.0  # [m]


def motion(x, u, dt):
    # motion model

    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def calc_dynamic_window(x, config):

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dT,
          x[3] + config.max_accel * config.dT,
          x[4] - config.max_dyawrate * config.dT,
          x[4] + config.max_dyawrate * config.dT]

    #  [vmin,vmax, yawrate min, yawrate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def calc_trajectory(xinit, v, y, config):

    x = np.array(xinit)
    traj = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        traj = np.vstack((traj, x))
        time += config.dt

    return traj


def calc_final_input(x, u, dw, config, goal, ob):

    xinit = x[:]
    min_cost = 10000.0
    min_u = u
    min_u[0] = 0.0
    best_traj = np.array([x])
    i = 0
    traj_all = {}
    # evalucate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_reso):
        for y in np.arange(dw[2], dw[3], config.yawrate_reso):
            traj = calc_trajectory(xinit, v, y, config)
            traj_all[i] = np.array(traj)
            i += 1
            # calc cost
            to_goal_cost = calc_to_goal_cost(traj, goal, config)
            speed_cost = config.speed_cost_gain * \
                (config.max_speed - traj[-1, 3])
            ob_cost = calc_obstacle_cost(traj, ob, config)
            # print(ob_cost)

            final_cost = to_goal_cost + speed_cost + ob_cost

            #print (final_cost)

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                min_u = [v, y]
                best_traj = traj
    print('goal_cost', to_goal_cost, 'speed_cost', speed_cost, 'obstacle_cost', ob_cost)
    print(dw)
    return min_u, best_traj, traj_all


def calc_obstacle_cost(traj, ob, config):
    # calc obstacle cost inf: collistion, 0:free

    skip_n = 2
    minr = float("inf")

    for ii in range(0, len(traj[:, 1]), skip_n):
        for i in range(len(ob[:, 0])):
            ox = ob[i, 0]
            oy = ob[i, 1]
            dx = traj[ii, 0] - ox
            dy = traj[ii, 1] - oy

            r = math.sqrt(dx**2 + dy**2)
            if r <= config.robot_radius:
                return float("Inf")  # collision

            if minr >= r:
                minr = r

    return 1.0 / minr  # OK


def calc_to_goal_cost(traj, goal, config):
    # calc to goal cost. It is 2D norm.
    # cnt = len(traj) // 5
    # goal_magnitude = math.sqrt(goal[0]**2 + goal[1]**2)
    # # traj_magnitude = math.sqrt(traj[-1, 0]**2 + traj[-1, 1]**2)
    # # dot_product = (goal[0] * traj[-1, 0]) + (goal[1] * traj[-1, 1])
    # traj_magnitude = math.sqrt(traj[cnt, 0]**2 + traj[cnt, 1]**2)
    # dot_product = (goal[0] * traj[cnt, 0]) + (goal[1] * traj[cnt, 1])
    # error = dot_product / (goal_magnitude * traj_magnitude + 1e-6)
    # error_angle = math.acos(error)
    # cost = config.to_goal_cost_gain * error_angle
    dist = math.sqrt((goal[0]-traj[-1, 0])**2 + (goal[1]-traj[-1, 1])**2)
    cost = config.to_goal_cost_gain * dist
    return cost


def dwa_control(x, u, config, goal, ob):
    # Dynamic Window control

    dw = calc_dynamic_window(x, config)

    u, traj, traj_all = calc_final_input(x, u, dw, config, goal, ob)

    return u, traj, traj_all


def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


from math import sin, cos, pi, atan2
POSX = 0
POSY = 1
YAW = 2
SPD = 3
YAWSPD = 4


# 注意适用范围: left, right 0~1000 rpm
# left, right都为正表示前进
def trimaran_model(state, left, right, dt):
    # X0 = state['x']  # 北东系x坐标 [m]
    # Y0 = state['y']  # 北东系y坐标 [m]
    # U0 = state['U']  # x方向速度(大地坐标系) [m/s] TODO 输入应该是速度和速度方向, 既然漂角很小, 那船横向速度v0是不是就是0
    # V0 = state['V']  # y方向速度(大地坐标系) [m/s]
    # phi0 = state['yaw']  # 艏向角，即船头与正北的夹角，范围为0~2PI [rad]
    # r0 = state['yaw_spd']  # 艏向角速度 [rad/s]
    X0 = state[POSX]
    Y0 = state[POSY]
    U0 = state[SPD] * cos(state[YAW])
    V0 = state[SPD] * sin(state[YAW])
    phi0 = state[YAW]
    r0 = state[YAWSPD]

    left = left / 60  # 船舶左桨转速 [rps]
    right = right / 60  # 船舶右桨转速 [rps]

    u0 = V0 * sin(phi0) + U0 * cos(phi0)  # 转为随船坐标系, i.e. 船纵向速度
    v0 = V0 * cos(phi0) - U0 * sin(phi0)  # 船横向速度

    #  根据当前状态, 螺旋桨转速1500, 1500 / 0, 1500, 求du, dv, dr极值
    du = (-6.7 * u0 ** 2 + 15.9 * r0 ** 2 + 0.01205 * (left ** 2 + right ** 2) - 0.0644 * (
        u0 * (left + right) + 0.45 * r0 * (left - right)) + 58 * r0 * v0) / 33.3
    dv = (-29.5 * v0 + 11.8 * r0 - 33.3 * r0 * u0) / 58
    dr = (-0.17 * v0 - 2.74 * r0 - 4.78 * r0 * abs(r0) + 0.45 * (
        0.01205 * (left ** 2 - right ** 2) - 0.0644 * (
            u0 * (left - right) + 0.45 * r0 * (left + right)))) / 6.1

    u = u0 + du * dt
    v = v0 + dv * dt

    r = r0 + dr * dt    # 更新后的转艏角速度
    phi = phi0 + (r + r0) * dt / 2  # 更新后的艏向角
    phi = phi % (2 * pi)
    U = u * cos(phi) - v * sin(phi)  # 更新后的速度, 转为大地坐标系
    V = u * sin(phi) + v * cos(phi)
    state[POSX] = X0 + (U0 + U) * dt / 2   # 更新后的坐标
    state[POSY] = Y0 + (V0 + V) * dt / 2
    state[YAW] = phi
    state[SPD] = math.sqrt(U ** 2 + V ** 2)
    state[YAWSPD] = r

    # return {
    #     'x': X,
    #     'y': Y,
    #     'U': U,
    #     'V': V,
    #     'yaw': phi,
    #     'yaw_spd': r
    # }
    return state


def pure_pursuit(self, target):
    dx = target[POSX] - self[POSX]
    dy = target[POSY] - self[POSY]
    target_angle = atan2(dy, dx)
    dist = (dx ** 2 + dy ** 2) ** .5
    return target_angle, dist


def main(gx=40, gy=40):
    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = np.array([gx, gy])
    # obstacles [x(m) y(m), ....]
    ob = np.array([[10.0, 10.0],
                   [10.0, 20.0],
                   [20.0, 30.0],
                   [30.0, 30.0],
                   [30.0, 25.0],
                   [40.0, 15.0]
                   ])

    u = np.array([0.0, 0.0])
    config = Config()
    traj = np.array(x)

    pid_yaw = PID(kp=800, ki=3, kd=10, minout=-1200, maxout=1200, sampleTime=0.1)
    pid_spd = PID(kp=3000.0, ki=100.0, kd=0, minout=0, maxout=2000, sampleTime=0.1)
    pid_yawspd = PID(kp=5000, ki=10.0, kd=0, minout=-1200, maxout=1200, sampleTime=0.1)
    for i in range(1000):
        ob[0, 0] -= 0.05
        ob[0, 1] -= 0.05
        ob[1, 0] -= 0.025
        ob[1, 1] += 0.025
        ob[3, 0] -= 0.2
        ob[3, 1] -= 0.2
        ob[4, 0] += 0.1
        ob[4, 1] += 0.1
        ob[5, 0] -= 0.1
        ob[5, 1] += 0.1
        goal[0] += 0.1
        goal[1] += 0.1
        u, ltraj, traj_all = dwa_control(x, u, config, goal, ob)

        # x = motion(x, u, config.dt)
        # target_point = ltraj[-1]  # ltraj[len(ltraj) // 3]
        # # target_point = goal
        # real_point = x
        # target_angle, dist = pure_pursuit(real_point, target_point)
        # if target_angle < 0:
        #     target_angle += 2 * pi
        # if dist <= 3:
        #     average = 600
        # elif dist <= 10:
        #     average = 800
        # else:
        #     average = 1000
        # output = pid_yaw.compute(x[YAW], target_angle)
        # print('output', output, 'ideal_angle', target_angle, 'real_angle', x[YAW])
        # output = 0 if abs(output) < 5 else output
        # left, right = average + output / 2, average - output / 2
        average_forward = 0  # 4000 * (0.2825 * u[0]**2 + 0.3648 * u[0])
        diff_forward = 0  # 2400 * (1.8704 * u[1]**2 + 0.5533 * u[1])
        average = pid_spd.compute(x[SPD], u[0])
        diff = pid_yawspd.compute(x[YAWSPD], u[1])
        # print('output', output, 'ideal_angle', target_angle, 'real_angle', state[YAW])
        print('real_spd', round(x[SPD],2), 'target_spd', round(u[0],2), 'output', round(average,2), 'forward', round(average_forward,2))
        print('real_yawspd', round(x[YAWSPD],2), 'target_yawspd', round(u[1], 2), 'output', round(diff,2), 'forward', round(diff_forward,2))
        average = 0 if abs(average) < 5 else average
        diff = 0 if abs(diff) < 5 else diff
        left, right = average + average_forward + (diff + diff_forward) / 2, average+average_forward - (diff + diff_forward) / 2
        left = max(min(left, 1500), 0)
        right = max(min(right, 1500), 0)
        print('left', left, 'right', right)
        x = trimaran_model(x, left, right, 0.1)

        traj = np.vstack((traj, x))  # store state history

        # print(traj)

        if show_animation:
            plt.cla()
            plt.plot(ltraj[:, 0], ltraj[:, 1], color="red", linewidth=1)
            # for point in range(len(traj_all)):
            #     plt.plot(traj_all[point][:, 0], traj_all[point][:, 1], color="green", linewidth=0.1)
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            plt.plot(ob[:, 0], ob[:, 1], "ok")
            plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)

        # check goal
        if math.sqrt((x[0] - goal[0])**2 + (x[1] - goal[1])**2) <= config.robot_radius:
            print("Goal!!")
            break

    print("Done")
    if show_animation:
        plt.plot(traj[:, 0], traj[:, 1], "-r")
        plt.pause(0.0001)

    plt.show()


if __name__ == '__main__':
    main()