import numpy as np
import matplotlib.pyplot as plt
from PID import PID
from math import cos, sin, pi
import time


# simulation parameters
class Config:

    def __init__(self):
        # robot parameter
        self.max_speed = 1.0  # 最大航速[m/s]
        self.min_speed = 0  # 最大倒车速度[m/s]
        self.max_yawrate = 0.65  # 最大转艏角速度[rad/s]
        self.max_accel = 0.45  # [m/ss] TODO min_accel
        self.max_dyawrate = 0.5  # 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_reso = 0.1  # [m/s]
        self.yawrate_reso = 0.1 * pi / 180.0  # [rad/s]

        self.dT = 2  # 动态窗口时间[s]
        self.dt = 0.1  # 轨迹推演时间步长[s]
        self.predict_time = 16.0  # 轨迹推演总时间[s]

        self.to_goal_cost_gain = 2.0
        self.speed_cost_gain = 1.0
        self.robot_radius = 1  # [m]


# 注意适用范围: left, right 0~1000 rpm
# left, right都为正表示前进
def trimaran_model(state, left, right, dt):
    X0 = state['x']  # 北东系x坐标 [m]
    Y0 = state['y']  # 北东系y坐标 [m]
    U0 = state['U']  # x方向速度(大地坐标系) [m/s] TODO 输入应该是速度和速度方向, 既然漂角很小, 那船横向速度v0是不是就是0
    V0 = state['V']  # y方向速度(大地坐标系) [m/s]
    phi0 = state['yaw']  # 艏向角，即船头与正北的夹角，范围为0~2PI [rad]
    r0 = state['yaw_spd']  # 艏向角速度 [rad/s]

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
    X = X0 + (U0 + U) * dt / 2   # 更新后的坐标
    Y = Y0 + (V0 + V) * dt / 2

    return {
        'x': X,
        'y': Y,
        'U': U,
        'V': V,
        'yaw': phi,
        'yaw_spd': r
    }


# x(m), y(m), yaw(rad), v(m/s), yaw spd(rad/s)
# u[0] v, u[1] yaw spd
def motion(x, u, dt):
    # motion model

    x[2] += u[1] * dt
    x[0] += u[0] * cos(x[2]) * dt
    x[1] += u[0] * sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x
