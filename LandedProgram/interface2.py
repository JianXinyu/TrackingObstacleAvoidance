import csv
import math
import time
from collections import deque
from math import atan2, pi, cos, sin
import numpy as np
from msgdev import MsgDevice, PeriodTimer

from PID import PID
from Simulator import trimaran_model, decode_state, encode_state, apply_noise
from utils import plot, POSX, POSY, YAW, SPD, YAWSPD, plot_traj

# Algorithm
PurePursuit = False
DynamicWindow = False
COMBINED = True
# DynamicWindow = True
# COMBINED = False

# DWA控制选择
USE_Trajectory_Following = False
USE_UR_Control = True

# 数据来源选择
USE_TLG002 = True
USE_FAKE_TARGET = True
USE_FAKE_OB1 = True
USE_FAKE_OB2 = True
# 一定是两true两false
USE_Fake_Goal = True
USE_Real_Goal = False
USE_Fake_Obstacle = True
USE_Real_Obstacle = False

# 可视化
show_animation = True
show_result = True

info_format = \
"Step {i} Info:\n\
SPD:\t real {real_spd:.2f} | target {target_spd:.2f}\n\
YAW:\t real {real_yaw:.2f} | target {target_yaw:.2f}\n\
YAWSPD:\t real {real_yawspd:.2f} | target {target_yawspd:.2f}\n\
Average Output:\t\t{average_output:.2f}\n\
Diff Output:\t{output_diff:.2f}\n\
Motor:\tleft {left:.2f} | right {right:.2f}\n\
Calc Time:\t {calc_time:.2f}\n\
==================="

# propeller parameter
rpm_normal_left = [800, 1200, 1400]
rpm_normal_right = [850, 1250, 1450]
rpm_max = 1500

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


class Interface_rec(object):
    def __init__(self, sub_addr, gnss_port):
        self.dev = MsgDevice()
        self.dev.open()

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

    def receive(self, *args):
        data = []
        for i in args:
            data.append(self.dev.sub_get1(i))
        return data


class PortPzh:
    def __init__(self, sub_addr, object_port):
        self.dev = MsgDevice()
        self.dev.open()
        self.dev.sub_connect(sub_addr + ':' + object_port)
        self.dev.sub_add_url("det.data", [-100] * 7)  # -100表示信号丢失，-99表示程序断了。

    def receive(self):
        data = self.dev.sub_get("det.data")
        assert len(data) == 7, "data should have 3*2 position and 1 target, totally 7 numbers"
        target = "Object {}".format(int(data[6])) if int(data[6]) in [0, 1, 2] else None
        assert target in ["Object 0", "Object 1", "Object 2", None]
        ret = {
            "Object 0": [data[0], data[1]] if data[0] not in [-100, -99] else None,
            "Object 1": [data[2], data[3]] if data[2] not in [-100, -99] else None,
            "Object 2": [data[4], data[5]] if data[4] not in [-100, -99] else None,
            "target": target,
            "terminated": all([d == -99 for d in data])
        }
        return ret


def ship_initialize(USE_TLG001, USE_TLG002, USE_FAKE_TARGET, USE_FAKE_OB1, USE_FAKE_OB2):
    if USE_TLG001:
        sub_addr1 = 'tcp://192.168.1.150'  # 'tcp://127.0.0.1'
        ahrs_port1 = '55005'
        gnss_port1 = '55004'
        motor_port1 = '55002'
        interface001 = Interface(sub_addr1, ahrs_port1, gnss_port1, motor_port1)
    else:
        interface001 = None

    if USE_TLG002:
        sub_addr2 = 'tcp://127.0.0.1'  # 'tcp://192.168.1.152'  # 'tcp://127.0.0.1'
        ahrs_port2 = '55205'
        gnss_port2 = '55204'
        motor_port2 = '55202'
        interface002 = Interface(sub_addr2, ahrs_port2, gnss_port2, motor_port2)
    else:
        interface002 = None

    if USE_FAKE_TARGET:
        sub_addr3 = 'tcp://127.0.0.1'
        gnss_port3 = '55205'
        fake_target = Interface_rec(sub_addr3, gnss_port3)
    else:
        fake_target = None

    if USE_FAKE_OB1:
        sub_addr4 = 'tcp://127.0.0.1'
        gnss_port4 = '55305'
        fake_ob1 = Interface_rec(sub_addr4, gnss_port4)
    else:
        fake_ob1 = None

    if USE_FAKE_OB2:
        sub_addr5 = 'tcp://127.0.0.1'
        gnss_port5 = '55405'
        fake_ob2 = Interface_rec(sub_addr5, gnss_port5)
    else:
        fake_ob2 = None

    return interface001, interface002, fake_target, fake_ob1, fake_ob2


def rpm_limit(rpm):
    if rpm > rpm_max:
        return rpm_max
    elif rpm < -rpm_max:
        return -rpm_max
    else:
        return rpm


# simulation parameters
class Config:
    def __init__(self):
        # robot parameter
        self.max_speed = 1.0  # 最大航速[m/s]
        self.min_speed = 0  # 最大倒车速度[m/s]
        self.max_yawrate = 0.6  # 最大转艏角速度[rad/s]
        # self.max_accel = 0.45  # [m/ss]
        # self.max_dyawrate = 0.5  # 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_reso = 0.1  # [m/s]  #TODO
        self.yawrate_reso = 6 * pi / 180.0  # [rad/s]

        self.dT = 1.0  # 动态窗口时间[s]
        self.dt = 0.1  # 轨迹推演时间步长[s]
        self.predict_time = 10.0  # 轨迹推演总时间[s]

        self.to_goal_cost_gain = 0.01  # 0.01
        self.speed_cost_gain = 0.5
        self.obstacle_cost_gain = 1.5 #TODO
        self.robot_radius = 3  # [m]
        self.robot_radius_square = self.robot_radius ** 2  # [m]

        self.left_max = 1500
        self.right_max = 1500


class DWA(object):
    def __init__(self, config):
        self.config = config
        self._ob_trajectory = []

    def update(self, state, acc_list, goal, ob):
        # 根据当前速度u, 角速度r计算最大加速度
        du_max, dr_max = self.update_accel(state[3], state[4])
        dw = self.calc_dynamic_window(state, du_max, dr_max)
        u, traj = self.calc_final_input(state, acc_list, dw, goal, ob)
        # u, traj, traj_all = self.calc_final_input(state, acc_list, dw, goal, ob, du_max, dr_max)
        return u, traj

    def update_accel(self, accel, r_accel):
        u0 = accel
        v0 = 0  # 假定船横向速度恒为0
        r0 = r_accel
        left = self.config.left_max / 60
        right = self.config.right_max / 60
        du_max = (-6.7 * u0 ** 2 + 15.9 * r0 ** 2 + 0.01205 * (left ** 2 + right ** 2) - 0.0644 * (
            u0 * (left + right) + 0.45 * r0 * (left - right)) + 58 * r0 * v0) / 33.3  # 认为螺旋桨转速最高时加速度最大
        # du_min = -6.7 * u0 ** 2 + 15.9 * r0 ** 2 + 58 * r0 * v0  # 认为螺旋桨不转时减速度最大
        # dv = (-29.5 * v0 + 11.8 * r0 - 33.3 * r0 * u0) / 58
        left = 1000 / 60  # 认为只有一边螺旋桨转速最高时角加速度最大, 为贴近实际, 减小转速最大值
        right = 0
        dr_max = (-0.17 * v0 - 2.74 * r0 - 4.78 * r0 * abs(r0) + 0.45 * (
            0.01205 * (left ** 2 - right ** 2) - 0.0644 * (
                u0 * (left - right) + 0.45 * r0 * (left + right)))) / 6.1
        return du_max, abs(dr_max)

    def calc_final_input(self, x, u, dw, goal, ob):
        xinit = x[:]
        min_cost = 10000
        min_u = u
        min_u[0] = 0.0
        best_traj = [x]
        # traj_all = {}
        # i = 0
        self._ob_trajectory.clear()
        # evaluate all trajectory with sampled input in dynamic window
        v_step = (dw[1] - dw[0]) / 6
        r_step = (dw[3] - dw[2]) / 12  # 0.1-0.2s
        for v in np.arange(dw[0], dw[1], v_step): # self.config.v_reso):
            for y in np.arange(dw[2], dw[3], r_step): # self.config.yawrate_reso):
                traj = self.calc_trajectory(xinit, v, y)
                # traj_all[i] = np.array(traj)
                # i += 1
                self.update_ob_trajectory(ob, len(traj))
                final_cost = self._get_cost(traj, goal, ob)
                if min_cost >= final_cost:
                    min_cost = final_cost
                    min_u = [v, y]
                    best_traj = traj
        # print(dw)
        # print(best_traj)
        return min_u, best_traj,  # traj_all

    def uniform_accel(self, state, spd_accel, yawspd_accel, dt):
        posx, posy, phi0, u0, r0 = decode_state(state)

        U0 = u0 * cos(phi0)
        V0 = u0 * sin(phi0)

        new_u = u0 + spd_accel * dt
        new_r = r0 + yawspd_accel * dt

        new_phi = phi0 + (new_r + r0) * dt / 2  # 更新后的艏向角
        new_phi = new_phi % (2 * pi)

        new_U = new_u * cos(new_phi)  # 更新后的速度, 转为大地坐标系
        new_V = new_u * sin(new_phi)

        new_posx = posx + (U0 + new_U) * dt / 2  # 更新后的坐标
        new_posy = posy + (V0 + new_V) * dt / 2

        state = encode_state(new_posx, new_posy, new_phi, new_u, new_r)
        return state

    # motion model
    # x(m), y(m), yaw(rad), v(m/s), yaw spd(rad/s)
    # u[0] v, u[1] yaw spd
    def uniform_spd(self, x, u, dt):
        x[2] += u[1] * dt
        x[0] += u[0] * cos(x[2]) * dt
        x[1] += u[0] * sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]

        return x

    def calc_dynamic_window(self, state, du_max, dr_max):
        # Dynamic window from kinematic self.configure
        # 船舶有能力达到的速度范围

        Vs = [self.config.min_speed, self.config.max_speed,
              -self.config.max_yawrate, self.config.max_yawrate]

        # 环境障碍物约束， 要求能在碰到障碍物之前停下来
        # Va = (2*du_min*min_dist)**.5

        # Dynamic window from motion model
        # 根据当前速度以及加速度限制计算的动态窗口, 依次为：最小速度 最大速度 最小角速度 最大角速度
        Vd = [0,  # max(0, state[3] + updated_accel['du_min'] * self.config.dT),
              state[3] + du_max * self.config.dT,
              state[4] - dr_max * self.config.dT,
              state[4] + dr_max * self.config.dT]

        #  [vmin, vmax, yawrate min, yawrate max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),  # Va),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw

    # 轨迹推演函数
    # 动态窗口法假定运载器的线速度和角速度在给定的规划时域保持不变，
    # 因此规划时域内运载器的局部轨迹只能是直线（r=0）或是圆弧（r≠0），
    # 可以通过(u,r)值确定
    # 但为了考虑操纵性, 加入了达到目标状态的匀加速轨迹
    def calc_trajectory(self, state, v, y):
        trajectory = [state.copy()]
        predicted_time = 0
        spd_accel = (v - state[SPD]) / self.config.dT
        yawspd_accel = (y - state[YAWSPD]) / self.config.dT
        while predicted_time <= self.config.predict_time:
            if predicted_time <= self.config.dT:  # 匀加速段
                state = self.uniform_accel(state, spd_accel, yawspd_accel, self.config.dt)
            else:  # 匀速段
                state = self.uniform_spd(state, [v, y], self.config.dt)
            trajectory.append(state.copy())  # 记录当前及所有预测的点
            predicted_time += self.config.dt
        return trajectory

    def _get_cost(self, traj, goal, ob):
        to_goal_cost = self.calc_to_goal_cost(traj[-1], goal, self.config)
        speed_cost = self.config.speed_cost_gain * (self.config.max_speed - traj[-1][3])
        ob_cost = self.config.obstacle_cost_gain * self.calc_obstacle_cost(traj, ob)
        final_cost = to_goal_cost + speed_cost + ob_cost
        # print('goal_cost', to_goal_cost, 'speed_cost', speed_cost, 'obstacle_cost', ob_cost)
        return final_cost

    def calc_obstacle_cost(self, traj, ob):
        # calc obstacle cost inf: collistion, 0:free

        skip_n = 2  # 每隔一个点计算一次, 加速
        minr = float("inf")

        for step in range(0, len(traj), skip_n):
            for ob in self._ob_trajectory[step]:
                r = (traj[step][0] - ob[0]) ** 2 + (traj[step][1] - ob[1]) ** 2
                if r <= self.config.robot_radius_square:
                    return float("Inf")
                minr = min(minr, math.sqrt(r))  # TODO 这里不能改为平方
        # for ii in range(0, len(traj), skip_n):
        #     for i in range(len(ob)):
        #         ox = ob[i][0] + ii * self.config.dt * ob[i][2] * cos(ob[i][3])  # 按障碍物匀速直线假设
        #         oy = ob[i][1] + ii * self.config.dt * ob[i][2] * sin(ob[i][3])
        #         dx = traj[ii][0] - ox
        #         dy = traj[ii][1] - oy
        #
        #         r = math.sqrt(dx ** 2 + dy ** 2)
        #         if r <= self.config.robot_radius:
        #             return float("Inf")  # collision
        #
        #         if minr >= r:
        #             minr = r
        return 1.0 / minr  # OK

    def update_ob_trajectory(self, ob_list, length=0):
        if len(self._ob_trajectory) == 0:
            self._ob_trajectory.append([[ob[0], ob[1]] for ob in ob_list])
        if len(self._ob_trajectory) < length:
            for _ in range(len(self._ob_trajectory), length):
                last_data = self._ob_trajectory[-1].copy()
                new_data = []
                for ob_id, ob in enumerate(ob_list):
                    new_ob = []
                    new_ob.append(last_data[ob_id][0] + self.config.dt * ob[2] * cos(ob[3]))
                    new_ob.append(last_data[ob_id][1] + self.config.dt * ob[2] * sin(ob[3]))
                    new_data.append(new_ob)
                # self._ob_trajectory.append(last_data.copy())
                self._ob_trajectory.append(new_data)

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
    target_angle = atan2(dy, dx) % (2 * pi)
    dist = (dx ** 2 + dy ** 2) ** .5
    return target_angle, dist


class MovingAverage(object):
    def __init__(self, max_len=10):
        self.val = deque(maxlen=max_len)
        self.avg = 0
        self.maxlen = max_len

    def update(self, val):
        self.val.append(val)
        self.avg = sum(self.val) / len(self.val)


# 将所有辨识目标的坐标 从随船坐标系转为大地坐标系
def trans_coord(self_coord, self_angle, object_coord):
    trans_matrix = np.mat([[cos(self_angle), -sin(self_angle)], [sin(self_angle), cos(self_angle)]])
    object_coord = np.mat(object_coord).T  # [2, num_obstacle]
    object_coord = trans_matrix * object_coord  # [2, 2] X [2, num_obstacle] = [2, num_obstacles]
    return object_coord.T + np.mat(self_coord)  # [num_obstacles, 2] + [1,2]


def main():
    # initialize
    interface001, interface002, fake_target, fake_ob1, fake_ob2 = ship_initialize(
        True, USE_TLG002, USE_FAKE_TARGET, USE_FAKE_OB1, USE_FAKE_OB2)

    fakedata = np.loadtxt('fakedata1.txt')
    fakedata2 = np.loadtxt('fakedata2.txt')
    fakedata3 = np.loadtxt('fakedata3.txt')

    # PP initialize
    target_angle = 0
    dist = 0

    # DWA initialize
    config = Config()
    dynamic_window = DWA(config)
    # initial state [x(m), y(m), yaw(rad), speed(m/s), yaw_speed(rad/s)]
    init_state = np.array([-40.0, -30.0, 45 * pi / 180, 0.0, 0.0])
    # goal position [x(m), y(m)]
    predefined_goal = np.array([-90.0, 3.0])
    goal = predefined_goal
    # obstacles [x(m) y(m), spd(m/s), yaw spd(rad/s)]
    ob = np.array([[-20.0, 0.0, 0.0, 0.0],
                   [-10.0, 0.0, 0.1, 0.0],
                   [-20.0, 0.0, 0.0, 0.0],
                   [-30.0, 0.0, 0.0, 0.0]
                   ])
    u = np.array([0.0, 0.0])

    traj = [init_state]
    best_traj = None
    state = init_state

    csv_source = open("log-{:s}.txt".format(time.strftime("%m-%d-%H-%M", time.localtime())), 'w')
    csv_file = csv.writer(csv_source)
    csv_file.writerow(
        ["timestamp", 'gps.posx', 'gps.posy', 'ahrs.yaw', 'ahrs.yaw_speed', 'gps.hspeed',
         'gps.stdx',
         'gps.stdy', 'gps.track', 'target.posx', 'target.posy', 'target.yaw', 'target.yaw_speed',
         'target.hspeed', 'target.stdx', 'target.stdy', 'target.track', 'left_motor',
         'right_motor'])

    # 计算频率
    interval = 0.1 # TODO
    pid_spd = PID(kp=3000.0, ki=100.0, kd=0, minout=0, maxout=2000, sampleTime=0.1)  # TODO
    pid_yawspd = PID(kp=5000, ki=10.0, kd=0, minout=-1200, maxout=1200, sampleTime=interval)
    pid_yaw = PID(kp=300, ki=3, kd=10, minout=-1200, maxout=1200, sampleTime=interval)
    t = PeriodTimer(interval)
    t.start()
    i = 0

    try:
        while True:
            with t:
                start_time = time.perf_counter()
                self_state = interface001.receive('gps.posx', 'gps.posy', 'ahrs.yaw',
                                                  'ahrs.yaw_speed', 'gps.hspeed',
                                                  'gps.stdx', 'gps.stdy', 'gps.track')
                if USE_TLG002:
                    target_state = interface002.receive('gps.posx', 'gps.posy', 'ahrs.yaw',
                                                        'ahrs.yaw_speed', 'gps.hspeed',
                                                        'gps.stdx', 'gps.stdy', 'gps.track')
                if USE_FAKE_TARGET:
                    fake_target_state = fake_target.receive('gps.posx','gps.posy')
                if USE_FAKE_OB1:
                    fake_ob1_state = fake_ob1.receive('gps.posx', 'gps.posy', 'gps.hspeed', 'gps.track')
                if USE_FAKE_OB2:
                    fake_ob2_state = fake_ob2.receive('gps.posx', 'gps.posy', 'gps.hspeed', 'gps.track')
                # 本船状态 POSX, POSY, YAW, SPEED, YAW SPEED
                # state = [self_state[0], self_state[1], self_state[2], self_state[4], self_state[3]]

                if i < 1000:
                    i += 1
                else:
                    i = 0

                # 追踪目标
                if USE_Fake_Goal:
                    goal = fake_target_state
                elif USE_Real_Goal:
                    goal[0] = target_state[0]
                    goal[1] = target_state[1]
                else:
                    goal = predefined_goal
                # 障碍物
                if USE_Fake_Obstacle:
                    ob[0, :] = fakedata[i + 1000, :]
                    ob[1, :] = fakedata[i, :]
                    ob[2, :] = fake_ob1_state
                    ob[3, :] = fake_ob2_state
                    obstacle = ob
                elif USE_Real_Obstacle:  # POSX, POSY, SPEED, Track
                    obstacle = [[target_state[0], target_state[1], target_state[4], target_state[7]]]
                else:
                    obstacle = [[]]

                if PurePursuit:
                    target_angle, dist = pure_pursuit(self_state, target_state)
                    if target_angle < 0:
                        target_angle += 2 * pi
                    output = pid_yaw.compute(self_state[YAW], target_angle)  # yaw, target_angle unit: rad
                    # dead band
                    diff = 0 if abs(output) < 5 else output
                    # 根据距离给速度分级
                    if dist <= 3:
                        average = 800
                    elif dist <= 10:
                        average = 1000
                    else:
                        average = 1200

                elif DynamicWindow:
                    # TODO 假目标, 假障碍
                    u, best_traj = dynamic_window.update(state, u, goal, obstacle)
                    if USE_UR_Control:
                        # 前馈
                        average_forward = 0  # 1000 * (0.2825 * u[0]**2 + 0.3648 * u[0])
                        diff_forward = 0  # 2400 * (1.8704 * u[1]**2 + 0.5533 * u[1])
                        # 速度控制器
                        average = pid_spd.compute(state[SPD], u[0])
                        average = 0 if abs(average) < 5 else average
                        # 转艏角速度控制器
                        diff = pid_yawspd.compute(state[YAWSPD], u[1])
                        diff = 0 if abs(diff) < 5 else diff

                    elif USE_Trajectory_Following:  # 朝着预测轨迹的某一点航行
                        target_point = best_traj[len(best_traj) // 3]
                        real_point = state
                        target_angle, dist = pure_pursuit(real_point, target_point)
                        if target_angle < 0:
                            target_angle += 2 * pi
                        if dist <= 2:
                            average = 600
                        elif dist <= 5:
                            average = 800
                        else:
                            average = 1000
                        diff = pid_yaw.compute(state[2], target_angle)
                        diff = 0 if abs(diff) < 5 else diff
                        # left, right = average + (diff + diff_forward) / 2, \
                        #               average - (diff + diff_forward) / 2
                    # print(info_format.format(
                    #     i=i,
                    #     real_spd=state[SPD],
                    #     target_spd=u[0],
                    #     real_yawspd=state[YAWSPD],
                    #     target_yawspd=u[1],
                    #     average_output=average,
                    #     output_diff=diff,
                    #     left=left,
                    #     right=right,
                    #     calc_time=time.perf_counter() - start_time
                    # ))
                    # if show_animation:
                    #     plot(best_traj, state, goal, ob)
                    # if show_result or show_animation:
                    #     traj.append(state.copy())
                    # to_goal_dist_square = (state[POSX] - goal[0]) ** 2 + (state[POSY] - goal[1]) ** 2
                    # if to_goal_dist_square <= config.robot_radius_square:
                    #     print("Goal!!")
                    #     break

                elif COMBINED:
                    min_dist = 100 ** 2
                    for ob_single in obstacle:
                        dist = (state[0] - ob_single[0]) ** 2 + (state[1] - ob_single[1]) ** 2
                        if dist < min_dist:
                            min_dist = dist

                    if min_dist > 10 ** 2:  # TODO
                        target_angle, dist = pure_pursuit(state, target_state)
                        if target_angle < 0:
                            target_angle += 2 * pi
                        output = pid_yaw.compute(state[2], target_angle)  # yaw, target_angle unit: rad
                        # dead band
                        diff = 0 if abs(output) < 5 else output
                        # 根据距离给速度分级
                        if dist <= 3:
                            average = 800
                        elif dist <= 10:
                            average = 1000
                        else:
                            average = 1200
                            # 前馈
                        average_forward = 0  # 1000 * (0.2825 * u[0]**2 + 0.3648 * u[0])
                        diff_forward = 0  # 2400 * (1.8704 * u[1]**2 + 0.5533 * u[1])
                        print('PP')
                    else:
                        u, best_traj = dynamic_window.update(state, u, goal, obstacle)
                        if USE_UR_Control:
                            # 前馈
                            average_forward = 0  # 1000 * (0.2825 * u[0]**2 + 0.3648 * u[0])
                            diff_forward = 0  # 2400 * (1.8704 * u[1]**2 + 0.5533 * u[1])
                            # 速度控制器
                            average = pid_spd.compute(state[SPD], u[0])
                            average = 0 if abs(average) < 5 else average
                            # 转艏角速度控制器
                            diff = pid_yawspd.compute(state[YAWSPD], u[1])
                            diff = 0 if abs(diff) < 5 else diff

                        elif USE_Trajectory_Following:  # 朝着预测轨迹的某一点航行
                            target_point = best_traj[len(best_traj) // 3]
                            real_point = state
                            target_angle, dist = pure_pursuit(real_point, target_point)
                            if target_angle < 0:
                                target_angle += 2 * pi
                            if dist <= 2:
                                average = 600
                            elif dist <= 5:
                                average = 800
                            else:
                                average = 1000
                            diff = pid_yaw.compute(state[2], target_angle)
                            diff = 0 if abs(diff) < 5 else diff

                else:
                    left, right = 0, 0

                left, right = average + average_forward + (
                        diff + diff_forward) / 2, average + average_forward - (
                                      diff + diff_forward) / 2

                left_motor = max(min(left, 1500), 0)
                right_motor = max(min(right, 1500), 0)

                print(info_format.format(
                    i=i,
                    real_spd=state[3],
                    target_spd=u[0],
                    real_yaw=state[2],
                    target_yaw=target_angle,
                    real_yawspd=state[4],
                    target_yawspd=u[1],
                    average_output=average,
                    output_diff=diff,
                    left=left,
                    right=right,
                    calc_time=time.perf_counter() - start_time
                ))

                if show_animation:
                    plot(best_traj, state, goal, ob)
                if show_result or show_animation:
                    traj.append(state.copy())
                to_goal_dist_square = (state[POSX] - goal[0]) ** 2 + (state[POSY] - goal[1]) ** 2
                if to_goal_dist_square <= config.robot_radius_square:
                    print("Goal!!")
                    break

                interface001.Motor_send(left_motor, right_motor)

                # Simulation
                state = trimaran_model(state, left, right, interval)
                state = apply_noise(state)

                # 本船距离目标的距离
                # dist = math.sqrt((self_state[POSX] - goal[0]) ** 2 +
                #                  (self_state[POSY] - goal[1]) ** 2)
                csv_file.writerow([time.time()] + self_state + target_state + [left_motor, right_motor])

                print("Overall time: ", time.perf_counter() - start_time)

    finally:
        interface001.Motor_send(0, 0)
        time.sleep(interval)
        interface001.dev.close()
        interface002.dev.close()
        csv_source.close()
        print('everything closed')


if __name__ == "__main__":

    print(__file__ + " start!!")
    main()

    # # 根据模型计算最大加速度, 速度
    # init_state = [0,0,0,0,0]
    # x = init_state
    # left = 1500
    # right = 1500
    # cnt = 0
    # flag = 0
    # spd_rec = deque(maxlen=2)
    # yawspd_rec = deque(maxlen=2)
    # spd_rec.append(0)
    # yawspd_rec.append(0)
    # interval = 0.001
    # calc_interval = 0.1  # (s)
    # t = PeriodTimer(interval)
    # t.start()
    # while True:
    #     with t:
    #         x = trimaran_model(x, left, right, interval)
    #         cnt += 1
    #         if cnt == calc_interval / interval:
    #             cnt = 0
    #             speed = x[SPD]
    #             spd_rec.append(speed)
    #             yawspd_rec.append(x[YAWSPD])
    #             acc = (spd_rec[1] - spd_rec[0]) / calc_interval
    #             yaw_acc = (yawspd_rec[1] - yawspd_rec[0]) / calc_interval
    #             print('speed', speed, 'accelerate', acc, 'yaw speed', x[YAWSPD], 'yaw_accelerate', yaw_acc)
