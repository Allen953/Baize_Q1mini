# Baize_QuadrupedRobot

白泽四足机器人是作者在玩过Opencat之后，设xiu计gai的一款四足机器人。

这款四足机器人同样有3D打印版本和CAD激光切割版本，3D打印版本和CAD激光切割版本图纸也都开源，想要3D打印的小伙伴可以打印出来，想要激光切割的小伙伴可以下载cad图纸用激光切割机，任意切割木板、亚克力或碳纤维材质的零件，没有激光切割机的也可以拿图纸去淘宝代加工，挺方便的。

板子默认用的Arduino nano，和自己设计的一块扩展板，主要是没在淘宝上找到合适的舵机扩展板。

arduino nano shield v3这块板子设计有问题，他的多级供电经过压降芯片，但是压降芯片功率比较小，一旦驱动多个舵机，电流过大这个芯片就会烧掉。


我这里自己切割的木板零件，具体图纸在gitee链接可以看到。这款机器人可以自己编程玩耍，同时也兼容了Opencat的程序（只需要拆下来换一种安装方式即可），这里可以通过该机器人学习编程和ROS机器人操作系统，一起交流玩耍吧！

下图为测试现场：

![实地测试](https://github.com/Allen953/Baize_QuadrupedRobot/blob/main/7.Photos%20%26%20Videos/Baize_QuadrupedRobot_Arduino.gif)

实物图片：

![实物图](https://github.com/Allen953/Baize_QuadrupedRobot/blob/main/7.Photos%20%26%20Videos/IMG_20210719_104603.jpg)

木板零件：

![实物图](https://github.com/Allen953/Baize_QuadrupedRobot/blob/main/7.Photos%20%26%20Videos/IMG_20210702_191204.jpg)

专用驱动板(可选)：

![实物图](https://github.com/Allen953/Baize_QuadrupedRobot/blob/main/7.Photos%20%26%20Videos/IMG_20220804_152804.jpg)





