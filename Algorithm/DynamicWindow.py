import numpy as np
import math
import matplotlib.pyplot as plt
import time
import sys
import pysnooper

sys.path.append("../../")


show_animation = True

# record direction
RecordPath = 'C:/Users/jianx/Desktop'

# Obstacle Coordinate

# Obstacle Radius: m
obstacle_r = 1

# Target Ship Coordinate

# Time Period, unit: s
dt = 0.5

# Vehicle Kinematic Parameters
# max speed: m/s; max rotate speed: rad/s; max accelerated speed: m/s^2; max rotational acceleration: rad/s^2
# speed resolution: m/s; rotate speed resolution: rad/s
Kinematic = [1.0, math.radians(20.0), 0.2, math.radians(50.0), 0.01, math.radians(1)]

# Evaluation Function Parameters
# Weight of Course, Weight of Distance, Weight of Speed, Time to simulate the trajectory forward(s)
EvalParam = [0.05, 0.2, 0.1, 3.0]

# Simulation Area [x_min, x_max, y_min, y_max]
area = [0, 12, 0, 12]

# Save Simulation Result
result = []

# obstacles是一个字典：keyword是障碍物的index，从0开始的序列； value是Obstacle类
obstacles = {}

# target是一个list
target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

vehicle_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # , 0.0, 0.0]
# 下标宏定义
POSE_X      = 0
POSE_Y      = 1
SPD         = 2
SPD_DIR     = 3
YAW         = 4
YAW_SPEED   = 5
# LEFT_RPM    = 6
# RIGHT_RPM   = 7


class MyTimer(object):
    """用上下文管理器计时"""
    def __enter__(self):
        self.t0 = time.time()

    def __exit__(self, exc_type, exc_val, exc_tb):
        print('[finished, spent time: {time:.2f}s]'.format(time=time.time() - self.t0))


class Config:
    def __init__(self):
        # Vehicle Kinematic Parameters
        self.max_speed = 1.0  # [m/s]
        self.min_speed = 0  # [m/s]
        self.max_yawrate = math.radians(20.0)  # max rotate speed [rad/s]
        self.max_accel = 0.2  # max accelerated speed [m/ss]
        self.max_dyawrate = math.radians(50.0)  # max rotational acceleration [rad/ss]
        self.speed_reso = 0.01  # speed resolution[m/s]
        self.yawrate_reso = math.radians(1)  # rotate speed resolution [rad/s]
        self.dt = 0.1  # Time Period [s]
        self.predict_time = 3.0  # Time to simulate the trajectory forward [s]
        # Evaluation Function Parameters
        self.to_goal_cost_gain = 1.0
        self.speed_cost_gain = 1.0
        self.obstacle_radius = 2.0  # [m]


class VehicleState:
    def __init__(self, posx=0.0, posy=0.0, hspeed=0.0, track=0.0, yaw=0.0, yaw_speed=0.0, l_m=0.0, r_m=0.0):
        self.posx = posx
        self.posy = posy
        self.spd = hspeed
        self.spddir = track
        self.yaw = yaw
        self.yawspd = yaw_speed
        self.leftrpm = l_m
        self.rightrpm = r_m


# def draw_obstacles():
class Obstacle:
    def __init__(self, posx=0.0, posy=0.0, spd=0.0, spddir=0.0): # index=0, radius = 0.0, yaw=0.0, yawspd=0.0
        # self.index = index
        # self.radius = radius
        self.posx = posx
        self.posy = posy
        self.spd = spd
        self.spddir = spddir
        # self.yaw = yaw
        # self.yawspd = yawspd



# def motion_model(state, speed, yaw_speed, dt)：


# trimaran model
def motion_model(state, left, right, dt):
    x = state[POSE_X]  # 北东系x坐标
    y = state[POSE_Y]  # 北东系y坐标
    u0 = state[SPD] * math.cos(state[SPD_DIR]) # x方向速度(大地坐标系) [m/s]
    v0 = state[SPD] * math.sin(state[SPD_DIR]) # y方向速度(大地坐标系) [m/s]
    phi = state[YAW]  # 艏向角，即船头与正北的夹角，范围为0~2PI， [rad]
    r0 = state[YAW_SPEED] # 艏向角速度 [rad/s]

    left = left / 60    # 船舶左桨转速 [rps]
    right = right / 60  # 船舶右桨转速 [rps]

    u = v0 * math.sin(phi) + u0 * math.cos(phi)
    v = v0 * math.cos(phi) - u0 * math.sin(phi)
    du = (-6.7 * u ** 2 + 15.9 * r0 ** 2 + 0.01205 * (left ** 2 + right ** 2) - 0.0644 * (
        u * (left + right) + 0.45 * r0 * (left - right)) + 58 * r0 * v) / 33.3
    dv = (-29.5 * v + 11.8 * r0 - 33.3 * r0 * u) / 58
    dr = (-0.17 * v - 2.74 * r0 - 4.78 * r0 * abs(r0) + 0.45 * (
        0.01205 * (left ** 2 - right ** 2) - 0.0644 * (
            u * (left - right) + 0.45 * r0 * (left + right)))) / 6.1
    u1 = u + du * dt
    v1 = v + dv * dt
    r = r0 + dr * dt
    phi1 = phi + (r + r0) * dt / 2
    U = u1 * math.cos(phi) - v1 * math.sin(phi)
    V = u1 * math.sin(phi) + v1 * math.cos(phi)
    spd = np.linalg.norm([U, V])
    spddir = math.atan(V/U)
    X = x + (u0 + U) * dt / 2
    Y = y + (v0 + V) * dt / 2

    phi1 = phi1 % (2 * math.pi)

    return [X, Y, spd, spddir, phi1, r]

    # X = state[POSE_X]  # 北东系x坐标
    # Y = state[POSE_Y]  # 北东系y坐标
    # state[YAW] += yaw_speed * dt
    # X += speed * math.cos(state[YAW]) * dt
    # Y += speed * math.sin(state[YAW]) * dt
    # state[SPD] = speed
    # state[YAW_SPEED] = yaw_speed


    # return [X, Y, state[SPD], state[SPD_DIR], state[YAW],state[YAW_SPEED]]

def calc_dynamic_window(currentstate, kineticconfig): #, min_dist):
    # 速度的最大最小范围 依次为：最小速度 最大速度 最小角速度 最大角加速度
    Vs = [kineticconfig.min_speed, kineticconfig.max_speed,
          -kineticconfig.max_yawrate, kineticconfig.max_yawrate]

    # 障碍物约束， 要求能在碰到障碍物之前停下来 # TODO 改为圆弧模型
    # Va = (2*kineticconfig.max_accel*min_dist)**.5

    # 根据当前速度以及加速度限制计算的动态窗口, 依次为：最小速度 最大速度 最小角速度 最大角速度
    Vd = [currentstate[SPD] - kineticconfig.max_accel * kineticconfig.dt,
          currentstate[SPD] + kineticconfig.max_accel * kineticconfig.dt,
          currentstate[YAW_SPEED] - kineticconfig.max_dyawrate * kineticconfig.dt,
          currentstate[YAW_SPEED] + kineticconfig.max_dyawrate * kineticconfig.dt]

    #  [vmin, vmax, yawrate min, yawrate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),# Va),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def calc_braking_distance():
    pass


# brief: 单条轨迹生成、轨迹推演函数
# param: state 三体船当前位姿； left，right 左右螺旋桨转速；config 三体船动力学参数
# retval:

# @pysnooper.snoop()
def calc_trajectory(state, speed, yaw_speed, config):

    traj = np.array(state)
    time = 0
    while time <= config.predict_time:
        state_tmp = motion_model(state, speed, yaw_speed, config.dt)
        traj = np.vstack((traj, state_tmp))  # 记录当前及所有预测的点
        time += config.dt

    return traj


# TODO 确定输出类型
def calc_obstacle_eval(trajectory, obstacles, config):
    # brief: 障碍物距离评价函数(在当前轨迹上与最近的障碍物之间的距离，如果没有障碍物则设定一个常数),距离障碍物距离越近分数越低
    # param: state 三体船当前位姿， obstacles 所有障碍物位置，config 三体船动力学参数
    # retval: 当前预测的轨迹终点的位姿距离所有障碍物中最近的障碍物的距离,如果大于设定的最大值则等于最大值.
    # calc obstacle cost inf: collision, 0:free
    skip_n = 2  # 没必要计算每个轨迹点
    min_dist = float("inf")  # 无穷大
    for ii in range(0, len(trajectory[:, 1]), skip_n):
        for i in range(len(obstacles)):
            dist = np.linalg.norm([(trajectory[ii, POSE_X] - obstacles[i].posx), (trajectory[ii, POSE_Y] - obstacles[i].posy)])
            # math.sqrt((state[POSE_X] - obstacles[i].posx)**2 + (state[POSE_Y] - obstacles[i].posy)**2)

            if dist <= config.obstacle_radius:
                return float("Inf")  # collision

            if min_dist >= dist:
                min_dist = dist

    if min_dist >= 100:  # 给障碍物距离评价限定最大值，防止没有障碍物的轨迹占比过重
        min_dist = 100

    return min_dist  # 1.0 / min_dist  # OK


# TODO 相对于yaw的偏角
def calc_to_goal_eval(traj, goal, config):
    # calc to goal cost. It is 2D norm.

    goal_magnitude = math.sqrt(goal[POSE_X]**2 + goal[POSE_Y]**2)
    traj_magnitude = math.sqrt(traj[-1, POSE_X]**2 + traj[-1, POSE_Y]**2)
    dot_product = (goal[POSE_X] * traj[-1, POSE_X]) + (goal[POSE_Y] * traj[-1, POSE_Y])
    error = dot_product / (goal_magnitude * traj_magnitude)
    error_angle = math.acos(error)
    cost = error_angle * config.to_goal_cost_gain

    return cost  # error_angle  # cost


def calc_speed_cost(traj, config):
    cost = (config.max_speed - traj[-1, SPD]) * config.speed_cost_gain
    return cost  # config.max_speed - traj[-1, SPD]  # cost


def eval_func(state, dw, config, goal, obstacles, speed, yaw_speed):
    # xinit = state[:]
    min_cost = 10000.0
    # min_u = [0.0, 0.0]
    speed = 0.0
    yaw_speed = 0.0
    best_traj = np.array([state])

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.speed_reso):
        for y in np.arange(dw[2], dw[3], config.yawrate_reso):
            state[SPD] = v
            state[YAW] = y
            traj = calc_trajectory(state, speed, yaw_speed, config)

            # calc cost
            to_goal_cost = calc_to_goal_eval(traj, goal, config)  # 前项预测终点的航向得分, 偏差越小分数越高
            speed_cost = calc_speed_cost(traj, config)  # 速度得分 速度越快分越高
            ob_cost = 1.0 / calc_obstacle_eval(traj, obstacles, config)   # 前项预测终点 距离最近障碍物的间隙得分 距离越远分数越高
            # print(ob_cost)

            final_cost = to_goal_cost + speed_cost + ob_cost

            #print (final_cost)

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                # min_u = [v, y]
                speed = v
                yaw_speed = y
                best_traj = traj

    return speed, yaw_speed, best_traj


def dwa_control(state, config, goal, obstacles, speed, yaw_speed):
    # Dynamic Window control
   # min_dist = calc_obstacle_eval(traj, obstacles, config)
    dw = calc_dynamic_window(state, config) #, min_dist)

    speed, yaw_speed, traj = eval_func(state, dw, config, goal, obstacles, speed, yaw_speed)

    return speed, yaw_speed, traj


def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


def main():
    # Initialize
    config = Config()
    # left = 1000
    # right = 1000
    speed = 0.0
    yaw_speed = 0.0
    state = motion_model(vehicle_state, speed, yaw_speed, config.dt)
    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    # x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = np.array([10, 10])
    # obstacles [x(m) y(m), ....]
    # ob = np.array([[-1, -1],
    #                [0, 2],
    #                [4.0, 2.0],
    #                [5.0, 4.0],
    #                [5.0, 5.0],
    #                [5.0, 6.0],
    #                [5.0, 9.0],
    #                [8.0, 9.0],
    #                [7.0, 9.0],
    #                [12.0, 12.0]
    #                ])
    obstacles[0] = Obstacle(4.0, 2.0)
    obstacles[1] = Obstacle(5.0, 5.0)
    obstacles[2] = Obstacle(5.0, 6.0)
    obstacles[3] = Obstacle(5.0, 9.0)
    obstacles[4] = Obstacle(8.0, 9.0)
    obstacles[5] = Obstacle(7.0, 9.0)
    obstacles[6] = Obstacle(12.0, 12.0)
    obstacles[7] = Obstacle(0, 2)
    u = np.array([0.0, 0.0])
    traj = np.array(state)

    for i in range(1000):
        speed, yaw_speed, ltraj = dwa_control(state, config, goal, obstacles, speed, yaw_speed)

        state = motion_model(state, speed, yaw_speed, config.dt) # motion_model(state, left, right, config.dt)
        traj = np.vstack((traj, state))  # store state history

        # print(traj)

        if show_animation:
            plt.cla()
            plt.plot(ltraj[:, 0], ltraj[:, 1], "-g")
            plt.plot(state[POSE_X], state[POSE_Y], "xr")
            plt.plot(goal[0], goal[1], "xb")
            for i in range(len(obstacles)):
                plt.plot(obstacles[i].posx, obstacles[i].posy, "ok")
            plot_arrow(state[POSE_X], state[POSE_Y], state[YAW])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)

        # check goal
        if math.sqrt((state[0] - goal[0])**2 + (state[1] - goal[1])**2) <= config.obstacle_radius:
            print("Goal!!")
            break

    print("Done")
    if show_animation:
        plt.plot(traj[:, 0], traj[:, 1], "-r")
        plt.pause(0.0001)

    plt.show()

if __name__ == '__main__':
    main()
