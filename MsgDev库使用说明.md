# MsgDev库使用说明（彭正皓, 2018.04）

在你的程序中调用MsgDev库，则可以通过很简单的方式与三体船互动。

首先，在你的程序中定义收发器：


        #Connect to control program
        dev_pro = MsgDevice()		
        dev_pro.open()
        dev_pro.sub_connect('tcp://127.0.0.1:55002')
        dev_pro.pub_bind('tcp://0.0.0.0:55003')

第一行创造实例

第二行开启收发器

第三行设置【接收通道】，也就是船上的主机（下位机）的IP地址。

第四行设置【发送通道】，也就是上位机的IP地址。之所以是四个零，是因为你要把输出的数据（也就是控制信号）发送到路由器的一个空间，让需要的程序自己来取就行了。


        # Connect to joystick
        dev_joy = MsgDevice()		#1
        dev_joy.open()
        dev_joy.sub_connect('tcp://127.0.0.1:55001')
        dev_joy.sub_add_url('js.autoctrl')

这里是设置手柄数据的收发。由于手柄不需要获取船的信息（因为是人类来控制的嘛）。第三行设置【接收通道】，第四行把那个url地址起个名字【autoctrl】。取了这个名字的目的是为了方便**你的电脑上的别的程序**获取来自这个收发器的数据（也就是你的手柄的控制信号），因为可能你需要对手柄的信号做一些处理再发送给船。


        left_motor = Motor(dev_pro, dev_joy,'left', 0x01, master)
        right_motor = Motor(dev_pro,dev_joy,'right', 0x02, master)

这里是两个张磊学长写的Motor类，第一个参数为发送器（用于向船发送信息），第二个参数为接收器（用于从手柄那里接受信息），第三个参数是名字。


        while True:
            with t:
                autoctrl = dev_joy.sub_get1('js.autoctrl')	#1
                dev_pro.pub_set1('autoctrl', autoctrl)
                left_motor.update(autoctrl)
                right_motor.update(autoctrl)

这里程序主循环，第一行把`接收器dev_joy`收到的数据存在`autoctrl`。
第二行用`发送器dev_pro`把数据发射到之前存储过的`autoctrl`地址处，程序会自动查找名字叫`autoctrl`的一条ip地址，然后发过去。
第三四行是学长写的motor的一些功能。
