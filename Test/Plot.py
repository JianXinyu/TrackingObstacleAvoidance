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
#
# def data_reader(file_path):  # 读取数据的函数
#     time = []
#     volt = []
#     with open(file_path, 'r') as f:
#         data = f.readlines()  # 自存放数据的.txt文件中读入数据
#         data.pop()  # 文件读入后在末尾总有个'\n'，后面使用split()函数时会出错，将其去掉
#         for i in data:
#             t, v = i.split(' ')  # 使用split()函数将时间和数据分隔开
#             h, m, s = t.strip('[]').split(':')  # 先处理时间，将时分秒分隔开
#             time.append(3600 * int(h) + 60 * int(m) + float(s))  # 将时间数据转换成秒
#             volt.append(float(v))  # 处理电压数据，将各式转为浮点型
#     time = np.array(time) - time[0]  # 获取时间间隔
#     volt = np.array(volt)
#     return np.array([time, volt])  # 将易于处理的格式的数据打包输出
#
#
# def painter(time, volt, aim_path):  # 绘图并保存至指定路径的函数
#     plt.plot(time, volt, 'g-')
#     plt.savefig(aim_path, format='png', dpi=300)
#
#
# if __name__ == '__main__':
#
#     cwd = os.getcwd()
#     aim = cwd + '\picture'  # 创建用于存放图片的文件夹路径
#     os.makedirs(aim)  # 按路径创建文件夹
#
#     source = 'D:\\Users\\zhm\\Desktop\\水开关实验5~35\\'  # .txt文件所在文件夹
#
#     '''下面的两个列表用于建立文件名变量，便于自动读入数据和写入图片。组合方式举例，
#        例如：浓度5-第1次、浓度5-第2次、浓度15-第3次... 浓度后的数字取自列表nongdu
#        第几次中的几取自列表cishu。
#     '''
#     nongdu = [5, 10, 15, 20, 25, 27, 29, 31, 33, 35]
#     cishu = [1, 2, 3]
#
#     for no in nongdu:
#         for ci in cishu:  # 使用循环遍历上述组合，依次读取所有.txt文件中的内容
#             f_name = '浓度' + str(no) + '-' + '第' + str(ci) + '次' + '.txt'
#             f_path = source + f_name
#             img_path = aim + '\\' + str(no) + '-' + str(ci) + '.png'
#             data = data_reader(f_path)  # 每读取一个文件就处理数据、绘图、保存
#             painter(data[0], data[1], img_path)


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
    data = np.loadtxt('%s\\log-05-05-16-36.txt' % root, skiprows=1, usecols=(1, 2, 5, 8, 9, 10, 13, 16), delimiter=',')
    data_de0 = [data[i, :] for i in range(len(data)) if data[i, 4] != 0]
    data_de0 = np.array(data_de0)
    print(np.shape(data_de0))
    # np.savetxt("fakedata1.txt", data[:, 4:8])
    # np.savetxt("fakedata2.txt", data[:, 0:4])
    np.savetxt("fakedata3.txt", data_de0[:, 4:8])
