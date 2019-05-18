
from math import cos, sin, pi, sqrt

# 下标宏定义
# state [x(m), y(m), yaw(rad), v(m/s), yaw spd(rad/s)]
POSX = 0
POSY = 1
YAW = 2
SPD = 3
YAWSPD = 4

def encode_state(*state):
    assert len(state)==5
    return list(state)

def decode_state(state):
    return state[POSX], state[POSY], state[YAW], state[SPD], state[YAWSPD]


# 注意适用范围: left, right 0~1000 rpm
# left, right都为正表示前进
def trimaran_model(state, left, right, dt):
    # X0 = state['x']  # 北东系x坐标 [m]
    # Y0 = state['y']  # 北东系y坐标 [m]
    # U0 = state['U']  # x方向速度(大地坐标系) [m/s]
    # V0 = state['V']  # y方向速度(大地坐标系) [m/s]
    # phi0 = state['yaw']  # 艏向角，即船头与正北的夹角，范围为0~2PI [rad]
    # r0 = state['yaw_spd']  # 艏向角速度 [rad/s]
    X0 = state[POSX]
    Y0 = state[POSY]
    U0 = state[SPD] * cos(state[YAW]) # 认为艏向就是速度方向, 船的横向速度为0
    V0 = state[SPD] * sin(state[YAW])
    phi0 = state[YAW]
    r0 = state[YAWSPD]

    left = left / 60  # 船舶左桨转速 [rps]
    right = right / 60  # 船舶右桨转速 [rps]

    u0 = V0 * sin(phi0) + U0 * cos(phi0)  # 转为随船坐标系, i.e. 船纵向速度
    v0 = V0 * cos(phi0) - U0 * sin(phi0)  # 船横向速度

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
    state[SPD] = sqrt(U ** 2 + V ** 2)
    state[YAWSPD] = r
    return state
