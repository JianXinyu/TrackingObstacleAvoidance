# 三体船使用流程

1. 船载平台开机: 
   - 接好两船的电源. 
   - 将箱子外部开关朝向自己, 打开箱子内部右边三个开关, 再打开外部开关即可上电. 
   - 当听到悦耳的声音时表明主板开机成功.
2. 布置WIFI天线: 
   - 天线采用POE供电，一个网口接岸基台式机，一个网口接天线，
   - 如接多个电脑，则加一个交换机。接好后ping `192.168.1.19`
3. 检查是否连上船载电脑WiFi:
   岸基台式机上ping `192.168.1.150`，若不通，则ping `192.168.1.21`，说明路由器能接通，路由器和主板的通讯有问题
4. 布置GPS基站:
   - 在WiFi架子上放置好GPS天线, 连接至接收机
   - 给接收机供电, 并将其COM2接到岸基电脑，运行TLG001/Landed Program/GPS_BS_Configuration 读约一分钟的位置，取均值，认为是精确值，运行完自动停掉
   - 打开GPS_Tools, 选串口，打开。 input基站读到的位置数据，output是把差分信号往船上发
5. 运行船载平台程序:
   - putty连船上主板，第一艘名称是usv150, 对应GPS名称是gps100, 同理第二艘usv152&gps102，账号密码均为sjtu。 
   - `sudo python3 文件名 &`, 运行AHRS.py, GNSS.py, motor_tlg.py, voltage.py. 注: &是放到后台运行, 多开几个putty终端则无需在后台运行
   - 如何修改船载程序? 
     - 岸基电脑的桌面TLG001/002文件夹，直接在sublime修改相应文件, 保存，右键SFTP/Upload file，密码sjtu。 
   - 如何杀死后台程序?
     - "ps aux|grep py"展示所有后台程序 
     - "sudo kill -9 "编号""
6. 运行岸基电脑端程序:
   - 连上遥控杆
   - 打开landed_porgram/joystick_pub.py: 右键-打开方式-python运行
7. 检查船载GPS情况: 
   - putty登陆gps100 or gps102
   - 终端输入`log bestposa ontime 2`, 表示每两秒显示一次GPS数据. 其中single表示没有差分， narrowfolat/narrowint表示有差分
8. 关机sudo poweroff
9. 第二艘船USB口损坏了一个. 若插hub, 则需要改端口名字
   cd dev
   cd serial/by-id
   ls
   并在相应船载文件修改URL 
# Appendix

1. 接线图：![circuit.PNG](https://i.loli.net/2019/05/03/5ccb19a6acf69.png)
2. 数据流：![flow.PNG](https://i.loli.net/2019/05/03/5ccb19fb394b9.png)