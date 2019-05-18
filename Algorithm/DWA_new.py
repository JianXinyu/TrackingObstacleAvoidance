import math
import random
import time
from math import cos, sin, pi, atan2

import matplotlib.pyplot as plt
import numpy as np

from algorithm.pid import PID
from algorithm.simulator import trimaran_model, decode_state, encode_state
from algorithm.utils import plot_arrow

# 下标宏定义
# state [x(m), y(m), yaw(rad), v(m/s), yaw spd(rad/s)]
POSX = 0
POSY = 1
YAW = 2
SPD = 3
YAWSPD = 4


# simulation parameters
class Config:
    def __init__(self):
        # robot parameter
        self.max_speed = 1.0  # 最大航速[m/s]
        self.min_speed = 0  # 最大倒车速度[m/s]
        self.max_yawrate = 0.6  # 最大转艏角速度[rad/s]
        self.max_accel = 0.45  # [m/ss] TODO min_accel
        self.max_dyawrate = 0.5  # 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_reso = 0.1  # [m/s]
        self.yawrate_reso = 6 * pi / 180.0  # [rad/s]

        self.dT = 1  # 动态窗口时间[s]
        self.dt = 0.1  # 轨迹推演时间步长[s]
        self.predict_time = 10.0  # 轨迹推演总时间[s]

        self.to_goal_cost_gain = 0.01  # 0.01
        self.speed_cost_gain = 0.5
        self.obstacle_cost_gain = 1.5
        self.robot_radius = 3  # [m]

        self.left_max = 1500
        self.right_max = 1500


class DWA(object):
    def __init__(self, config):
        self.config = config

    def update(self, state, acc_list, goal, ob):
        # def dynamic_window(x, u, config, goal, ob):
        # Dynamic Window control

        # 根据当前速度u, 角速度r计算最大加速度
        updated_accel = self.update_accel(state[3], state[4])

        dw = self.calc_dynamic_window(state, updated_accel)

        u, traj = self.calc_final_input(state, acc_list, dw, goal, ob)

        return u, traj

    def calc_final_input(self, x, u, dw, goal, ob):
        xinit = x[:]
        min_cost = 10000.0
        min_u = u
        min_u[0] = 0.0
        best_traj = np.array([x])
        # traj_all = []
        i = 0
        # evaluate all trajectory with sampled input in dynamic window
        for v in np.arange(dw[0], dw[1], self.config.v_reso):
            for y in np.arange(dw[2], dw[3], self.config.yawrate_reso):
                traj = self.calc_trajectory(xinit, v, y)
                # traj_all[i] = np.array(traj)
                i += 1
                # traj = calc_trajectory(xinit, v, y, self.config)
                # calc cost
                final_cost = self._get_cost(traj, goal, ob)
                # search minimum trajectory
                if min_cost >= final_cost:
                    min_cost = final_cost
                    min_u = [v, y]
                    best_traj = traj
        print(dw)
        # print('min_u', min_u, 'best_traj', best_traj)
        return min_u, best_traj

    def uniform_accel(self, state, spd_accel, yawspd_accel, dt):
        # 认为船只有纵向速度
        # u0 = state[SPD]
        # phi0 = state[YAW]
        # r0 = state[YAWSPD]

        posx, posy, phi0, u0, r0 = decode_state(state)

        U0 = u0 * cos(phi0)
        V0 = u0 * sin(phi0)

        # u0 = u0 + spd_accel * dt
        # r0 += yawspd_accel * dt

        # state[SPD] += spd_accel * dt
        # state[YAWSPD] += yawspd_accel * dt
        new_u = u0 + spd_accel * dt
        new_r = r0 + yawspd_accel * dt

        new_phi = phi0 + (new_r + r0) * dt / 2  # 更新后的艏向角
        new_phi = new_phi % (2 * pi)
        # state[YAW] += (r + r0) * dt / 2  # 更新后的艏向角

        # state[YAW] = state[YAW] % (2 * pi)

        # phi = state[YAW]
        new_U = new_u * cos(new_phi)  # 更新后的速度, 转为大地坐标系
        new_V = new_u * sin(new_phi)

        new_posx = posx + (U0 + new_U) * dt / 2  # 更新后的坐标
        new_posy = posy + (V0 + new_V) * dt / 2

        # state[POSX] += (U0 + U) * dt / 2   # 更新后的坐标
        # state[POSY] += (V0 + V) * dt / 2
        state = encode_state(new_posx, new_posy, new_phi, new_u, new_r)
        return state

    # motion model TODO 改为圆弧模型
    # x(m), y(m), yaw(rad), v(m/s), yaw spd(rad/s)
    # u[0] v, u[1] yaw spd
    def uniform_spd(self, x, u, dt):
        x[2] += u[1] * dt
        x[0] += u[0] * cos(x[2]) * dt
        x[1] += u[0] * sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]

        return x

    # 轨迹推演函数
    # 动态窗口法假定运载器的线速度和角速度在给定的规划时域保持不变，
    # 因此规划时域内运载器的局部轨迹只能是直线（r=0）或是圆弧（r≠0），
    # 可以通过(u,r)值确定
    # 但为了考虑操纵性, 加入了达到目标状态的匀加速轨迹
    def calc_trajectory(self, state, v, y):
        state = np.array(state)
        trajectory = np.array(state)
        # trajectory = state
        predicted_time = 0
        spd_accel = (v - state[SPD]) / self.config.dT
        yawspd_accel = (y - state[YAWSPD]) / self.config.dT
        while predicted_time <= self.config.predict_time:
            if predicted_time <= self.config.dT:  # 匀加速段
                state = self.uniform_accel(state, spd_accel, yawspd_accel, self.config.dt)
            else:  # 匀速段
                state = self.uniform_spd(state, [v, y], self.config.dt)
            trajectory = np.vstack((trajectory, state))  # 记录当前及所有预测的点
            # trajectory.append(state)
            predicted_time += self.config.dt
        return trajectory

    def _get_cost(self, traj, goal, ob):
        to_goal_cost = self.calc_to_goal_cost(traj[-1], goal, self.config)
        speed_cost = self.config.speed_cost_gain * (self.config.max_speed - traj[-1][3])
        ob_cost = self.config.obstacle_cost_gain * self.calc_obstacle_cost(traj, ob)
        final_cost = to_goal_cost + speed_cost + ob_cost
        print('goal_cost', to_goal_cost, 'speed_cost', speed_cost, 'obstacle_cost', ob_cost)
        return final_cost

    def calc_dynamic_window(self, state, updated_accel):
        # Dynamic window from kinematic self.configure
        # 船舶有能力达到的速度范围

        Vs = [self.config.min_speed, self.config.max_speed,
              -self.config.max_yawrate, self.config.max_yawrate]

        # 环境障碍物约束， 要求能在碰到障碍物之前停下来
        # Va = (2*du_min*min_dist)**.5

        # Dynamic window from motion model
        # 根据当前速度以及加速度限制计算的动态窗口, 依次为：最小速度 最大速度 最小角速度 最大角速度
        Vd = [0,  # max(0, state[3] + updated_accel['du_min'] * self.config.dT),  # TODO du_min, du_max的符号
              state[3] + updated_accel['du_max'] * self.config.dT,
              state[4] - updated_accel['dr_max'] * self.config.dT,
              state[4] + updated_accel['dr_max'] * self.config.dT]

        #  [vmin, vmax, yawrate min, yawrate max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),  # Va),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw

    def update_accel(self, accel, r_accel):
        u0 = accel
        v0 = 0  # 假定船横向速度恒为0
        r0 = r_accel
        left = self.config.left_max / 60
        right = self.config.right_max / 60
        du_max = (-6.7 * u0 ** 2 + 15.9 * r0 ** 2 + 0.01205 * (left ** 2 + right ** 2) - 0.0644 * (
            u0 * (left + right) + 0.45 * r0 * (left - right)) + 58 * r0 * v0) / 33.3  # 认为螺旋桨转速最高时加速度最大
        du_min = -6.7 * u0 ** 2 + 15.9 * r0 ** 2 + 58 * r0 * v0  # 认为螺旋桨不转时减速度最大
        # dv = (-29.5 * v0 + 11.8 * r0 - 33.3 * r0 * u0) / 58
        left = 1000 / 60  # 认为只有一边螺旋桨转速最高时角加速度最大, 为贴近实际, 减小转速最大值
        right = 0
        dr_max = (-0.17 * v0 - 2.74 * r0 - 4.78 * r0 * abs(r0) + 0.45 * (
            0.01205 * (left ** 2 - right ** 2) - 0.0644 * (
                u0 * (left - right) + 0.45 * r0 * (left + right)))) / 6.1
        return {
            'du_max': du_max,
            'du_min': du_min,
            'dr_max': abs(dr_max)
        }

    def calc_obstacle_cost(self, traj, ob):
        # calc obstacle cost inf: collistion, 0:free

        skip_n = 2  # 每隔一个点计算一次, 加速
        minr = float("inf")

        for ii in range(0, len(traj[:, 1]), skip_n):
            for i in range(len(ob)):
                ox = ob[i][0] + ii * self.config.dt * ob[i][2] * cos(pi / 4)  # 按障碍物匀速直线假设
                oy = ob[i][1] + ii * self.config.dt * ob[i][2] * sin(pi / 4)
                dx = traj[ii][0] - ox
                dy = traj[ii][1] - oy

                r = math.sqrt(dx ** 2 + dy ** 2)
                if r <= self.config.robot_radius:
                    return float("Inf")  # collision

                if minr >= r:
                    minr = r

        return 1.0 / minr  # OK

    def calc_to_goal_cost(self, last_point, goal, config):
        # calc to goal cost. It is 2D norm.

        # 1. 本船, 目标与坐标系零点连线的夹角, 越大损失越大
        # goal_magnitude = math.sqrt(goal[0] ** 2 + goal[1] ** 2)
        # traj_magnitude = math.sqrt(traj[-1, 0] ** 2 + traj[-1, 1] ** 2)
        # dot_product = (goal[0] * traj[-1, 0]) + (goal[1] * traj[-1, 1])
        # error = dot_product / (goal_magnitude * traj_magnitude)
        # error_angle = math.acos(error)
        # cost = config.to_goal_cost_gain * error_angle

        # 2. 本船的艏向和本船与目标连线的夹角, 越大损失越大. 取预测轨迹的最后一个点
        # dist = math.sqrt((goal[0]-traj[-1, 0])**2 + (goal[1]-traj[-1, 1])**2)
        # angle = math.sin((goal[1] - traj[-1, 1]) / dist)
        # cost = config.to_goal_cost_gain * abs(angle - traj[-1, 2])

        # 3. 本船和目标的距离
        dist = math.sqrt((goal[0] - last_point[0]) ** 2 + (goal[1] - last_point[1]) ** 2)
        cost = config.to_goal_cost_gain * dist
        return cost


def pure_pursuit(self, target):
    dx = target[POSX] - self[POSX]
    dy = target[POSY] - self[POSY]
    target_angle = atan2(dy, dx)
    dist = (dx ** 2 + dy ** 2) ** .5
    return target_angle, dist


show_animation = True
baseline = [600, 900, 1200]


def main():
    print(__file__ + " start!!")
    fakedata = np.loadtxt('fakedata1.txt')
    fakedata2 = np.loadtxt('fakedata2.txt')

    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    init_state = np.array([-20.0, -30.0, 45 * pi / 180, 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = np.array([-40.0, -20.0])
    # obstacles [x(m) y(m), ....]
    # ob = np.array([[20.0, 20.0, 0.2],
    #                [10.0, 20.0, 0.1],
    #                [20.0, 30.0, 0],
    #                [30.0, 30.0, -0.8],
    #                [30.0, 25.0, 0.4],
    #                [40.0, 15.0, 0.6]
    #                ])
    ob = np.array([[-20.0, 0.0, 0.0, 0.0],
                   [-10.0, 0.0, 0.1, 0.0],
                   [-20.0, 0.0, 0, 0.0],
                   ])
    u = np.array([0.0, 0.0])
    config = Config()
    dynamic_window = DWA(config)
    # traj = np.array(init_state)
    traj = [init_state]
    pid_yaw = PID(kp=300, ki=3, kd=10, minout=-1200, maxout=1200, sampleTime=0.1)
    pid_spd = PID(kp=3000.0, ki=100.0, kd=0, minout=0, maxout=2000, sampleTime=0.1)
    pid_yawspd = PID(kp=5000, ki=10.0, kd=0, minout=-1200, maxout=1200, sampleTime=0.1)
    state = init_state
    init_time = time.perf_counter()

    # 对输入数据加入gauss噪声
    # 定义gauss噪声的均值和方差
    mu = 0
    sigma = 0.0036

    for i in range(1000):
        start = time.perf_counter()
        # 虚拟动态障碍物
        # ob[0, 0] -= 0.01
        # ob[0, 1] -= 0.01
        ob[0, :] = fakedata[i + 1000, :] + [random.gauss(mu, sigma), random.gauss(mu, sigma), 0, 0]
        ob[1, :] = fakedata2[i + 300, :]
        ob[2, :] = fakedata[i, :]
        # ob[1, 0] -= 0.005
        # ob[1, 1] += 0.005
        # ob[3, 0] -= 0.04
        # ob[3, 1] -= 0.04
        # ob[4, 0] += 0.02
        # ob[4, 1] += 0.02
        # ob[5, 0] -= 0.03
        # ob[5, 1] += 0.03
        goal[0] = fakedata[i + 2000, 0]
        goal[1] = fakedata[i + 2000, 1]

        # u, ltraj, traj_all = dynamic_window.update(state, u, goal, ob)
        u, ltraj = dynamic_window.update(state, u, goal, ob)
        traj = np.vstack((traj, state))  # store state history

        target_point = ltraj[len(ltraj) // 3]
        # target_point = goal
        real_point = state
        target_angle, dist = pure_pursuit(real_point, target_point)
        if target_angle < 0:
            target_angle += 2 * pi
        # if dist <= 2:
        #     average = baseline[0]
        # elif dist <= 5:
        #     average = baseline[1]
        # else:
        #     average = baseline[2]
        # output = pid_yaw.compute(state[YAW], target_angle)
        average_forward = 0  # 1000 * (0.2825 * u[0]**2 + 0.3648 * u[0])
        diff_forward = 0  # 2400 * (1.8704 * u[1]**2 + 0.5533 * u[1])
        average = pid_spd.compute(state[SPD], u[0])
        diff = pid_yawspd.compute(state[YAWSPD], u[1])
        # print('output', output, 'ideal_angle', target_angle, 'real_angle', state[YAW])
        print('real_spd', round(state[SPD], 2), 'target_spd', round(u[0], 2), 'output', round(average, 2), 'forward',
              round(average_forward, 2))
        print('real_yawspd', round(state[YAWSPD], 2), 'target_yawspd', round(u[1], 2), 'output', round(diff, 2),
              'forward', round(diff_forward, 2))
        average = 0 if abs(average) < 5 else average
        diff = 0 if abs(diff) < 5 else diff
        left, right = average + average_forward + (diff + diff_forward) / 2, average + average_forward - (
        diff + diff_forward) / 2
        left = max(min(left, 1500), 0)
        right = max(min(right, 1500), 0)
        print('left', left, 'right', right)

        state = trimaran_model(state, left, right, 0.5)
        # 加入过程噪声, 高斯分布, 均值为0
        state += [random.gauss(0, 0.01), random.gauss(0, 0.01),  # POSX, POSY
                  random.gauss(0, 0.01), random.gauss(0, 0.01),  # Yaw (rad), Speed (m/s)
                  random.gauss(0, 0.005)]  # yaw speed (rad/s)
        # state = uniform_spd(state, u, 0.1)
        # print(traj)

        elapsed = (time.perf_counter() - start)
        print("Time used:", elapsed)

        if show_animation:
            plt.cla()
            plt.plot(ltraj[:, 0], ltraj[:, 1], color="red", linewidth=1)
            # 显示所有候选轨迹
            # for point in range(len(traj_all)):
            #     plt.plot(traj_all[point][:, 0], traj_all[point][:, 1], color="green", linewidth=0.2)
            plt.plot(state[POSX], state[POSY], "xr")
            plt.plot(goal[0], goal[1], "xb")
            plt.plot(ob[:, 0], ob[:, 1], "ok")

            circle1 = plt.Circle((ob[2, 0], ob[2, 1]), 3, color='blue', Fill=False)
            circle2 = plt.Circle((ob[1, 0], ob[1, 1]), 3, color='blue', Fill=False)
            circle3 = plt.Circle((ob[0, 0], ob[0, 1]), 3, color='blue', Fill=False)
            ax = plt.gca()
            ax.add_artist(circle1)
            ax.add_artist(circle2)
            ax.add_artist(circle3)
            plot_arrow(state[POSX], state[POSY], state[YAW])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)
        elapsedd = (time.perf_counter() - start)
        print("Plot Time used:", elapsedd)
        print("overall time: ", time.perf_counter() - init_time)
        # check goal
        if math.sqrt((state[POSX] - goal[0]) ** 2 + (state[POSY] - goal[1]) ** 2) <= config.robot_radius:
            print("Goal!!")
            break

    print("Done")

    if show_animation:
        plt.plot(traj[:, 0], traj[:, 1], "-r")
        plt.pause(0.0001)

    plt.show()


if __name__ == '__main__':
    main()
