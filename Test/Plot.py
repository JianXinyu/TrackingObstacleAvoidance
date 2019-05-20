import matplotlib.pyplot as plt
import numpy as np
import os
import random
import pandas as pd

# 0 "timestamp",
# 1'gps.posx', 2'gps.posy', 3'ahrs.yaw', 4'ahrs.yaw_speed', 5'gps.hspeed',6'gps.stdx', 7'gps.stdy', 8'gps.track',
# 9'target.posx', 10'target.posy', 11'target.yaw', 12'target.yaw_speed',
#          13'target.hspeed', 14'target.stdx', 15'target.stdy', 16'target.track',
# 17'distance', 18'left_motor', 19 'right_motor'


def find_txt(path, ret):
    """Finding the *.txt file in specify path"""
    filelist = os.listdir(path)
    for filename in filelist:
        de_path = os.path.join(path, filename)
        if os.path.isfile(de_path):
            if de_path.endswith(".txt"):  # Specify to find the txt file.
                ret.append(de_path)
        else:
            find_txt(de_path, ret)


def load_data(file_name):
    data = np.loadtxt(file_name, skiprows=1, usecols=(1, 2, 5, 8, 9, 10, 13, 16), delimiter=',')
    return data


def painter(data, file_name):  # 绘图并保存至指定路径的函数
    ax = plt.gca()

    x_self = data[:, 0]
    y_self = data[:, 1]
    x_target = data[:, 4]
    y_target = data[:, 5]

    # df = pd.DataFrame({'y': y_self, 'x': x_self})
    # df.plot(x='y')
    ax.xaxis.set_ticks_position('top')  # 将x轴的位置设置在上边
    ax.invert_xaxis()  # x轴反向
    ax.yaxis.set_ticks_position('right')  # 将y轴的位置设置在右边
    ax.invert_yaxis()
    title = file_name.split('data\\')[1]
    ax.set_title(title)
    plt.plot(y_self, x_self, color="red", label='self_ship')
    plt.plot(y_target, x_target, color="green", label='target_ship')
    ax.legend(loc='best')
    # plt.scatter(y_self, x_self, s=1, color="red")
    # for i in range(len(data[:, 0])):
    #     plt.text(y_self[i], x_self[i], i, fontsize=4)
    # plt.scatter(y_target, x_target, s=1, color="green")
    # for i in range(len(data[:, 0])):
    #     plt.text(y_target[i], x_target[i], i, fontsize=4)
    plt.grid(True)
    path = file_name.replace('txt', 'jpg')
    plt.savefig(path, dpi=600)
    plt.clf()


if __name__ =="__main__":

    root = "D:\\GraduationProject\\data"

    # ret = []
    # find_txt(root, ret)
    # for file_name in ret:
    #     data = load_data(file_name)
    #     painter(data, file_name)

    # data = np.loadtxt('log-05-05-16-44.txt', skiprows=1, usecols=(1, 2, 5, 8, 9, 10, 13, 16), delimiter=',')  # 380 - 1050 跟踪部分
    # data = np.loadtxt('log-05-05-16-59.txt', skiprows=1, usecols=(1, 2, 5, 8, 9, 10, 13, 16), delimiter=',')  # 不好
    # data = np.loadtxt('%s\\log-05-12-15-51.txt' % root, skiprows=1, usecols=(1, 2, 5, 8, 9, 10, 13, 16), delimiter=',')  # 0.5s的间隔
    # data = np.loadtxt('%s\\log-05-05-17-21.txt' % root, skiprows=1, usecols=(1, 2, 5, 8, 9, 10, 13, 16), delimiter=',') # 0.5s的间隔
    data = np.loadtxt('%s\\log-05-12-16-47.txt' % root, skiprows=1, usecols=(1, 2, 5, 8, 9, 10, 13, 16), delimiter=',')
    # 除去零值
    data_de0 = [data[i, :] for i in range(len(data)) if (data[i, 0] != 0) & (data[i, 1] != 0)]
    data_de0 = np.array(data_de0)
    print(np.shape(data_de0))

    # 数据填充
    # data_fill = np.zeros((5*(len(data_de0)-1), 8))
    # for i in range(len(data_de0)-1):
    #     cnt = 5 * i
    #     diff = (data_de0[i+1] - data_de0[i]) / 5
    #     for ii in range(5):
    #         data_fill[cnt+ii] = data_de0[i] + diff * ii

    def interpolate(inp, multiplier=5):
        expanded_shape = [multiplier * (inp.shape[0] - 1) + 1, *inp.shape[1:]]
        out = np.empty(expanded_shape)
        diff = (inp[1:] - inp[:-1]) / multiplier
        for i in range(multiplier):
            out[i:-1:multiplier] = i * diff + inp[:-1]
        out[-1] = inp[-1]
        return out
    data_fill = interpolate(data_de0)

    np.savetxt("fakedata11.txt", data_fill[:, 0:4])
    print(np.shape(data_fill))
    plt.scatter(data_fill[:, 0], data_fill[:, 1], color="red")
    plt.show()

    # np.savetxt("fakedata6.txt", data_de0[:, 4:8])
    # print(np.shape(data_de0))
    # plt.scatter(data_de0[:, 4], data_de0[:, 5], color="red")
    # plt.show()

    # np.savetxt("fakedata1.txt", data[:, 4:8])
    # np.savetxt("fakedata2.txt", data[:, 0:4])
    # np.savetxt("fakedata3.txt", data_de0[:, 4:8])
