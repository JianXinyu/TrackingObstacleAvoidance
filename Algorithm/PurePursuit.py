import numpy as np
import math
import matplotlib.pyplot as plt

from PID import PID
from math import cos, sin, pi
from numpy.random import normal
import time
k = 0.1  # look forward gain
Lfc = 2.0  # look-ahead distance
dt = 0.1  # 时间间隔，单位：s


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


kp = 1000
ki = 3
kd = 10

# baseline = 1000
baseline = [800, 1200, 1400]

# propeller parameter
rpm_normal_left = [800, 1200, 1400]
rpm_normal_right = [850, 1250, 1450]
rpm_max = 2000

data = []

# 这里随机生成初始状态
old_state = {
    'x': 0,
    'y': 0,
    'u': normal(0, 0.2),
    'v': normal(0, 0.2),
    'phi': np.random.rand() * 2 * pi,
    'alpha': normal(0, 0.01)
}


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

    ideal_angle = math.atan2(ty - state['y'], tx - state['x'])
    dist = ((state['x']-tx)**2+(state['y']-ty)**2)**.5

    return ideal_angle, ind, dist


def diff_adjust(diff):
    while diff>pi:
        diff -= 2*pi
    while diff<-pi:
        diff += 2*pi
    return diff


def rpm_limit(rpm):
    if rpm>rpm_max:
        return rpm_max
    elif rpm<-rpm_max:
        return -rpm_max
    else:
        return rpm


def point_generate(point):
    # if np.random.rand() >= 0.5:
    #     coefficient = 1
    # else:
    #     coefficient = -1;
    # if np.random.rand() >= 0.5:
    #     coefficient2 = 1
    # else:
    #     coefficient2 = -1;
    point['x'] += np.random.rand() * 0.1 # * coefficient
    point['y'] += np.random.rand() * 0.1 # * coefficient2
    return point


def calc_target_index(state, cx, cy):
    # 搜索最临近的路点
    dx = [state['x'] - icx for icx in cx]
    dy = [state['y'] - icy for icy in cy]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    L = 0.0

    Lf = k * math.sqrt(state['u'] ** 2 + state['v'] ** 2) + Lfc
    # 根据最邻近的路点找前视点
    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cy[ind + 1] - cy[ind]
        L += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1

    return ind


def pure_pursuit(self, target):
    dx = target[0] - self[0]
    dy = target[1] - self[1]
    target_angle = math.atan2(dy, dx)
    dist = (dx ** 2 + dy ** 2) ** .5
    return target_angle, dist


def main():
    pid = PID(kp=kp, ki=ki, kd=kd, minout=-2000, maxout=2000, sampleTime=0.1)
    # 设置初始状态
    state = old_state

    #  设置目标路点
    cx = np.arange(0, 50, 1)
    cy = [math.cos(ix / 5.0) * ix / 2.0 for ix in cx]

    T = 100.0  # 最大模拟时间

    lastIndex = len(cx) - 1
    time = 0.0
    x = [state['x']]
    y = [state['y']]
    yaw = [state['phi']]
    v = [math.sqrt(state['u'] ** 2 + state['v'] ** 2)]
    t = [0.0]
    target_ind = calc_target_index(state, cx, cy)

    point = {'x': 0, 'y': 0}

    target_point = [0, 0]
    while T >= time and lastIndex > target_ind:

        # ideal_angle, target_ind, dist = pure_pursuit_control(state, cx, cy, target_ind
        self_point = [state['x'], state['y']]
        target_point[0] += 0.1
        target_point[1] += 0.1
        ideal_angle, dist = pure_pursuit(self_point, target_point)
        output = pid.compute(state['phi'], ideal_angle)
        output = 0 if abs(output) < 5 else output
        if dist <= 3:
            average = baseline[0]
        elif dist <= 10:
            average = baseline[1]
        else:
            average = baseline[2]
        left, right = average + output / 2, average - output / 2
        left = rpm_limit(left)
        right = rpm_limit(right)
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
        plt.plot(target_point[0], target_point[1], "go", label="point")
        plt.axis("equal")
        plt.grid(True)
        plt.title("Speed[km/h]:" + str(math.sqrt(state['u'] ** 2 + state['v'] ** 2) * 3.6)[:4])
        plt.pause(0.001)

if __name__ == '__main__':
    main()