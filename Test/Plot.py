import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# 0 "timestamp",
# 1'gps.posx', 2'gps.posy', 3'ahrs.yaw', 4'ahrs.yaw_speed', 5'gps.hspeed',6'gps.stdx', 7'gps.stdy', 8'gps.track',
# 9'target.posx', 10'target.posy', 11'target.yaw', 12'target.yaw_speed',
#          13'target.hspeed', 14'target.stdx', 15'target.stdy', 16'target.track',
# 17'distance', 18'left_motor', 19 'right_motor'


if __name__ =="__main__":

    data = np.loadtxt('log-05-05-16-44.txt', skiprows=1, usecols=(1, 2, 9, 10), delimiter=',')
    plt.cla()
    ax = plt.gca()
    x_self = data[:, 0]
    y_self = data[:, 1]
    x_target = data[:, 2]
    y_target = data[:, 3]
    # df = pd.DataFrame({'y': y_self, 'x': x_self})
    # df.plot(x='y')
    ax.xaxis.set_ticks_position('top')  # 将x轴的位置设置在上边
    ax.invert_xaxis()  # x轴反向
    ax.yaxis.set_ticks_position('right')  # 将y轴的位置设置在右边
    ax.invert_yaxis()
#    plt.plot(y_self, x_self, color="red")
#    plt.plot(y_target, x_target, color="green")
    plt.scatter(y_self, x_self, s=1, color="red")
    plt.scatter(y_target, x_target, s=1, color="green")
    # plt.axis("equal")
    # pl.xticks(rotation=90) 标签
    plt.grid(True)
    plt.show()
