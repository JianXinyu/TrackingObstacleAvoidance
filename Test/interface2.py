import csv
import math
import time
from collections import deque
from math import atan2, pi, cos, sin

import numpy as np
from PID import PID

from msgdev import MsgDevice, PeriodTimer


class Interface(object):
    def __init__(self, sub_addr, ahrs_port, gnss_port, motor_port):
        self.dev = MsgDevice()
        self.dev.open()
        self.dev.sub_connect(sub_addr + ':' + ahrs_port)
        self.dev.sub_add_url('ahrs.roll')
        self.dev.sub_add_url('ahrs.pitch')
        self.dev.sub_add_url('ahrs.yaw')
        self.dev.sub_add_url('ahrs.roll_speed')
        self.dev.sub_add_url('ahrs.pitch_speed')
        self.dev.sub_add_url('ahrs.yaw_speed')
        self.dev.sub_add_url('ahrs.acce_x')
        self.dev.sub_add_url('ahrs.acce_y')
        self.dev.sub_add_url('ahrs.acce_z')

        self.dev.sub_connect(sub_addr + ':' + gnss_port)
        self.dev.sub_add_url('gps.time')
        self.dev.sub_add_url('gps.posx')
        self.dev.sub_add_url('gps.posy')
        self.dev.sub_add_url('gps.posz')
        self.dev.sub_add_url('gps.stdx')
        self.dev.sub_add_url('gps.stdy')
        self.dev.sub_add_url('gps.stdz')
        self.dev.sub_add_url('gps.satn')
        self.dev.sub_add_url('gps.hspeed')
        self.dev.sub_add_url('gps.vspeed')
        self.dev.sub_add_url('gps.track')

        self.dev.pub_bind('tcp://0.0.0.0:' + motor_port)

    def receive(self, *args):
        data = []
        for i in args:
            data.append(self.dev.sub_get1(i))
        return data

    def Motor_send(self, left_motor, right_motor):
        self.dev.pub_set1('pro.left.speed', left_motor)
        self.dev.pub_set1('pro.right.speed', right_motor)


class PortPzh:
    def __init__(self, sub_addr, object_port):
        self.dev = MsgDevice()
        self.dev.open()
        self.dev.sub_connect(sub_addr + ':' + object_port)
        self.dev.sub_add_url("det.data", [-100] * 7)  # -100表示信号丢失，-99表示程序断了。

    def receive(self):
        data = self.dev.sub_get("det.data")
        assert isinstance(data, list), "data should be a list"
        assert len(data) == 7, "data should have 3*2 position and 1 target, totally 7 numbers"
        ret = {
            "Object 0": [data[0], data[1]] if data[0] not in [-100, -99] else None,
            "Object 1": [data[2], data[3]] if data[2] not in [-100, -99] else None,
            "Object 2": [data[4], data[5]] if data[4] not in [-100, -99] else None,
            "target": "Object {}".format(data[6]) if data[6] in [0, 1, 2] else None,
            "terminated": all([d == -99 for d in data])
        }
        return ret


def ship_initialize(USE_TLG001, USE_TLG002, USE_PZH):
    if USE_TLG001:
        sub_addr1 = 'tcp://192.168.1.150'#'tcp://127.0.0.1'
        ahrs_port1 = '55005'
        gnss_port1 = '55004'
        motor_port1 = '55002'
        interface001 = Interface(sub_addr1, ahrs_port1, gnss_port1, motor_port1)
    else:
        interface001 = None

    if USE_TLG002:
        sub_addr2 = 'tcp://127.0.0.1' # 'tcp://192.168.1.152'
        ahrs_port2 = '55205'
        gnss_port2 = '55204'
        motor_port2 = '55202'
        interface002 = Interface(sub_addr2, ahrs_port2, gnss_port2, motor_port2)
    else:
        interface002 = None

    if USE_PZH:
        sub_addr3 = 'tcp://192.168.1.222'
        object_port = '55019'
        pzhdata = PortPzh(sub_addr3, object_port)
    else:
        pzhdata = None
    return interface001, interface002, pzhdata


# 下标宏定义
POS_X = 0
POS_Y = 1
YAW = 2
YAW_SPEED = 3
SPD = 4
SPD_DIR = 5

# PID parameters
kp = 800
ki = 3
kd = 10

# Algorithm
# PP = False
# DWA = True
# USE_PZH = False

# baseline = 1000
baseline = [800, 1200, 1400]

# propeller parameter
rpm_normal_left = [800, 1200, 1400]
rpm_normal_right = [850, 1250, 1450]
rpm_max = 2000


def pure_pursuit(self, target):
    dx = target[POS_X] - self[POS_X]
    dy = target[POS_Y] - self[POS_Y]
    target_angle = atan2(dy, dx)
    dist = (dx ** 2 + dy ** 2) ** .5
    return target_angle, dist


def rpm_limit(rpm):
    if rpm > rpm_max:
        return rpm_max
    elif rpm < -rpm_max:
        return -rpm_max
    else:
        return rpm


# DWA
init_state = {
    'x': 0,
    'y': 0,
    'u': 0,
    'v': 0,
    'phi': 0,
    'alpha': 0
}


class Config():
    # simulation parameters

    def __init__(self):
        # robot parameter
        self.max_speed = 1.0  # [m/s]
        self.min_speed = 0  # [m/s]
        self.max_yawrate = 20.0 * pi / 180.0  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_dyawrate = 20.0 * pi / 180.0  # [rad/ss]
        self.v_reso = 0.01  # [m/s]
        self.yawrate_reso = 0.1 * pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s]
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 2.0
        self.speed_cost_gain = 1.0
        self.robot_radius = 0.5  # [m]


# x0-posx, x1-posy, x2-yaw, x3-u, x4-v
# u0-u, u1-v
def motion(x, u, dt):
    # motion model

    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def simulate(state, left, right, dt):
    x = state['x']
    y = state['y']
    u0 = state['u']
    v0 = state['v']
    phi = state['phi']  # 艏向角
    r0 = state['alpha']  # 角速度

    left = left / 60
    right = right / 60

    u = v0 * sin(phi) + u0 * cos(phi)
    v = v0 * cos(phi) - u0 * sin(phi)
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
    U = u1 * cos(phi) - v1 * sin(phi)
    V = u1 * sin(phi) + v1 * cos(phi)
    X = x + (u0 + U) * dt / 2
    Y = y + (v0 + V) * dt / 2

    phi1 = phi1 % (2 * pi)

    return {
        'x': X,
        'y': Y,
        'u': U,
        'v': V,
        'phi': phi1,
        'alpha': r
    }


def calc_dynamic_window(x, config):
    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]

    #  [vmin, vmax, yawrate min, yawrate max]
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

    # evalucate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_reso):
        for y in np.arange(dw[2], dw[3], config.yawrate_reso):
            traj = calc_trajectory(xinit, v, y, config)

            # calc cost
            to_goal_cost = calc_to_goal_cost(traj, goal, config)
            speed_cost = config.speed_cost_gain * \
                         (config.max_speed - traj[-1, 3])
            ob_cost = calc_obstacle_cost(traj, ob, config)
            final_cost = to_goal_cost + speed_cost + ob_cost
            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                min_u = [v, y]
                best_traj = traj

    return min_u, best_traj


def calc_obstacle_cost(traj, ob, config):
    # calc obstacle cost inf: collistion, 0:free

    skip_n = 2
    minr = float("inf")

    for ii in range(0, len(traj[:, 1]), skip_n):
        for i in range(len(ob)):
            ox = ob[i][0]
            oy = ob[i][1]
            dx = traj[ii][0] - ox
            dy = traj[ii][1] - oy

            r = math.sqrt(dx ** 2 + dy ** 2)
            if r <= config.robot_radius:
                return float("Inf")  # collision

            if minr >= r:
                minr = r

    return 1.0 / minr  # OK


def calc_to_goal_cost(traj, goal, config):
    # calc to goal cost. It is 2D norm.

    goal_magnitude = math.sqrt(goal[0] ** 2 + goal[1] ** 2)
    traj_magnitude = math.sqrt(traj[-1, 0] ** 2 + traj[-1, 1] ** 2)
    dot_product = (goal[0] * traj[-1, 0]) + (goal[1] * traj[-1, 1])
    error = dot_product / (goal_magnitude * traj_magnitude + 1e-6)
    error_angle = math.acos(error)
    cost = config.to_goal_cost_gain * error_angle

    return cost


def dwa_control(x, u, config, goal, ob):
    # Dynamic Window control

    dw = calc_dynamic_window(x, config)

    u, traj = calc_final_input(x, u, dw, config, goal, ob)

    return u, traj


class MovingAverage(object):
    def __init__(self, max_len=10):
        self.val = deque(maxlen=max_len)
        self.avg = 0
        self.maxlen = max_len

    def update(self, val):
        self.val.append(val)
        self.avg = sum(self.val) / len(self.val)


def trans_coord(self_coord, self_angle, object_coord):
    trans_matrix = np.mat([[cos(self_angle), -sin(self_angle)], [sin(self_angle), cos(self_angle)]])
    object_coord = np.mat(object_coord).T  # [2, num_obstacle]
    object_coord = trans_matrix * object_coord  # [2, 2] X [2, num_obstacle] = [2, num_obstacles]
    return object_coord.T + np.mat(self_coord)  # [num_obstacles, 2] + [1,2]
    #return np.mat(relative_coord) + np.mat(self_coord)


def main(args):
    USE_PZH = args.use_pzh
    PP = args.pp
    DWA = args.dwa

    # initialize
    interface001, interface002, pzhdata = ship_initialize(True, True, USE_PZH)
    pid = PID(kp=kp, ki=ki, kd=kd, minout=-1000, maxout=1000, sampleTime=0.1)

    u = np.array([0.0, 0.0])  # speed, yaw speed
    config = Config()

    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    # 需要根据实际出发点赋值
    x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])

    # goal position [x(m), y(m)]
    goal = np.array([-90, 3])
    predefiend_goal = goal.copy()

    # obstacles
    obstacles = []

    csv_file = csv.writer(
        open("log-{:s}.txt".format(time.strftime("%m-%d-%H-%M", time.localtime())), 'w'))

    csv_file.writerow(
        ["timestamp", 'gps.posx', 'gps.posy', 'ahrs.yaw', 'ahrs.yaw_speed', 'gps.hspeed',
         'gps.stdx',
         'gps.stdy', 'gps.track', 'target.posx', 'target.posy', 'target.yaw', 'target.yaw_speed',
         'target.hspeed', 'target.stdx', 'target.stdy', 'target.track', 'distance', 'left_motor',
         'right_motor'])
    interval = 0.5
    t = PeriodTimer(interval)
    t.start()
    try:
        while True:
            with t:
                self_state = interface001.receive('gps.posx', 'gps.posy', 'ahrs.yaw',
                                                  'ahrs.yaw_speed', 'gps.hspeed',
                                                  'gps.stdx', 'gps.stdy', 'gps.track')
                target_state = interface002.receive('gps.posx', 'gps.posy', 'ahrs.yaw',
                                                    'ahrs.yaw_speed', 'gps.hspeed',
                                                    'gps.stdx', 'gps.stdy', 'gps.track')

                if USE_PZH:
                    assert pzhdata is not None
                    lidar_data = pzhdata.receive()

                    if lidar_data["terminated"]:
                        print(
                            "Peng Zhenghao's program is terminated. For safety we close this program.")
                        break

                    target = lidar_data["target"]
                    if not target: #如果没有搜索到target, 则用初始化的goal作为目标点
                        # TODO 所以如果target没有指定会发生什么事情？
                        print("No Target Specified! Heading to [-90,3]")
                        goal = predefiend_goal
                    else:
                        goal = target  # goal = [x, y]
                        for key, centroid in lidar_data.items():
                            if key.startswith("Object") and centroid and key != target:
                                obstacles.append(centroid)
                        obstacles = trans_coord([self_state[POS_X], self_state[POS_Y]],
                                                self_state[YAW],
                                                np.array(obstacles))
                        assert obstacles.shape[1] == 2
                        obstacles = obstacles.tolist()
                else:
                    obstacles = [[target_state[POS_X], target_state[POS_Y]]]# [num_obstacles, 2]

                if PP:
                    if USE_PZH:
                        target_angle, dist = pure_pursuit(self_state, goal)
                    else:
                        target_angle, dist = pure_pursuit(self_state, target_state)
                    output = pid.compute(self_state[YAW], target_angle)  #yaw, target_angle unit: rad
                elif DWA:
                    x = [self_state[POS_X], self_state[POS_Y], self_state[YAW], self_state[SPD],
                         self_state[YAW_SPEED]]
                    u, ltraj = dwa_control(x, u, config, goal, obstacles)
                    ideal_angle = self_state[YAW] + u[1] * 0.1
                    output = pid.compute(self_state[YAW], ideal_angle)
                    # 本船距离目标的距离
                    dist = math.sqrt((self_state[POS_X] - goal[0]) ** 2 +
                                     (self_state[POS_Y] - goal[1]) ** 2)
                else:
                    output, dist = 0, 0

                ################## 此处往下没有问题 #######################

                # dead band
                output = 0 if abs(output) < 5 else output

                if dist <= 3:
                    average = baseline[0]
                elif dist <= 10:
                    average = baseline[1]
                else:
                    average = baseline[2]
                left_motor, right_motor = (average + output / 2), average - output / 2
                left_motor = -rpm_limit(left_motor)
                right_motor = rpm_limit(right_motor)

                print('self ship state: ',
                      'posx:{},posy:{},yaw:{},speed:{},left:{},right:{}'.format(self_state[POS_X],
                                                                                self_state[POS_Y],
                                                                                self_state[YAW],
                                                                                self_state[SPD],
                                                                                left_motor,
                                                                                right_motor))
                print('target ship state: ',
                      'posx:{}, posy: {}，distance:{}'.format(target_state[POS_X],
                                                             target_state[POS_Y], dist))

                print('goal', goal, 'obstacles', obstacles)
                interface001.Motor_send(left_motor, right_motor)

                csv_file.writerow(
                    [time.time()] + self_state + target_state + [dist, left_motor,
                                                                 right_motor])

    finally:
        interface001.Motor_send(0, 0)
        time.sleep(interval)
        interface001.dev.close()
        interface002.dev.close()
        pzhdata.dev.close()
        csv_file.close()
        print('everything closed')


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--use-pzh", "-p", action="store_true")
    parser.add_argument("--pp", "-pp", action="store_true")
    parser.add_argument("--dwa", "-dwa", action="store_true")
    args = parser.parse_args()

    main(args)
