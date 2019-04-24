import numpy as np
import math
import matplotlib.pyplot as plt
from PID import PID
from math import cos, sin, pi
from numpy.random import normal

k = 0.1  # 前视距离系数
Lfc = 2.0  # 前视距离
Kp = 1.0  # 速度P控制器系数
dt = 0.1  # 时间间隔，单位：s
L = 2.9  # 车辆轴距，单位：m


def simulate(state, left, right, dt):
    x = state['x']
    y = state['y']
    u0 = state['u']
    v0 = state['v']
    phi = state['phi']
    r0 = state['alpha']

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


kp = 800
ki = 3
kd = 10
baseline = 1000
data = []
# 这里随机生成初始状态
old_state = {
    'x': 0,
    'y': 0,
    'u': normal(0, 0.2),
    'v': normal(0, 0.2),
    # 'phi': np.random.rand() * 2 * pi,
    'phi': 3.14159 / 4,
    'alpha': normal(0, 0.01)
}

class VehicleState:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):

        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update(state, a, delta):

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.v = state.v + a * dt

    return state

def PControl(target, current):
    a = Kp * (target - current)

    return a


def pure_pursuit_control(state, cx, cy, pind):

    ind = calc_target_index(state, cx, cy)

    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    # alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw
    #
    # if state.v < 0:  # back
    #     alpha = math.pi - alpha
    #
    # Lf = k * state.v + Lfc

    # delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

    # heading_goal = math.atan2(ty - state['y'], tx - state['x'])
    # diff = diff_adjust(heading_goal - - state['yaw'])
    ideal_angle = math.atan2(ty - state['y'], tx - state['x'])
    # if state.v < 0:  # back
    #     alpha = math.pi - alpha
    #
    # Lf = k * math.sqrt(state['u'] ** 2 + state['v'] ** 2) + Lfc
    # delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)
    return ideal_angle, ind#delta, ind


def diff_adjust(diff):
    while diff>pi:
        diff -= 2*pi
    while diff<-pi:
        diff += 2*pi
    return diff


def calc_target_index(state, cx, cy):
    # 搜索最临近的路点
    # dx = [state.x - icx for icx in cx]
    # dy = [state.y - icy for icy in cy]
    # d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    # ind = d.index(min(d))
    # L = 0.0
    #
    # Lf = k * state.v + Lfc
    #
    # while Lf > L and (ind + 1) < len(cx):
    #     dx = cx[ind + 1] - cx[ind]
    #     dy = cy[ind + 1] - cy[ind]
    #     L += math.sqrt(dx ** 2 + dy ** 2)
    #     ind += 1
    #
    # return ind
    dx = [state['x'] - icx for icx in cx]
    dy = [state['y'] - icy for icy in cy]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    L = 0.0

    Lf = k * math.sqrt(state['u'] ** 2 + state['v'] ** 2) + Lfc

    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cy[ind + 1] - cy[ind]
        L += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1

    return ind

def main():
    pid = PID(kp=kp, ki=ki, kd=kd, minout=-2000, maxout=2000, sampleTime=0.1)
    state = old_state

    #  设置目标路点
    cx = np.arange(0, 50, 1)
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]

    target_speed = 10.0 / 3.6  # [m/s]

    T = 100.0  # 最大模拟时间

    # 设置初始状态
    # state = VehicleState(x=-0.0, y=-3.0, yaw=0.0, v=0.0)

    lastIndex = len(cx) - 1
    time = 0.0
    # x = [state.x]
    # y = [state.y]
    # yaw = [state.yaw]
    # v = [state.v]
    x = [state['x']]
    y = [state['y']]
    yaw = [state['phi']]
    v = [math.sqrt(state['u'] ** 2 + state['v'] ** 2)]
    t = [0.0]
    target_ind = calc_target_index(state, cx, cy)

    while T >= time and lastIndex > target_ind:
        # ai = PControl(target_speed, state.v)
        # di, target_ind = pure_pursuit_control(state, cx, cy, target_ind)
        # state = update(state, ai, di)
        ideal_angle, target_ind = pure_pursuit_control(state, cx, cy, target_ind)
        output = pid.compute(state['phi'], ideal_angle)
        output = 0 if abs(output) < 5 else output
        left, right = baseline + output / 2, baseline - output / 2
        left = max(min(left, 2000), -2000)
        right = max(min(right, 2000), -2000)
        adata = [state['x'], state['y'], state['u'], state['v'], state['phi'], state['alpha'], left, right]
        data.append(adata)
        state = simulate(state, left, right, 0.1)

        time = time + dt

        x.append(state['x'])
        y.append(state['y'])
        yaw.append(state['phi'])
        v.append(math.sqrt(state['u'] ** 2 + state['v'] ** 2))
        t.append(time)

        plt.cla()
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.plot(cx[target_ind], cy[target_ind], "go", label="target")
        plt.axis("equal")
        plt.grid(True)
        plt.title("Speed[km/h]:" + str(math.sqrt(state['u'] ** 2 + state['v'] ** 2) * 3.6)[:4])
        plt.pause(0.001)

if __name__ == '__main__':
    main()