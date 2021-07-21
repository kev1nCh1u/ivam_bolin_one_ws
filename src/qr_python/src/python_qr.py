#! /usr/bin/env python
# coding=UTF-8
import rospy
import serial
from std_msgs.msg import String
from qr_python.msg import send
import sys
import math


def change_fun(i):
    a = int(s[i*2], 16)*16+int(s[i*2+1], 16)
    return a


def receive_data():
    High_X = (change_fun(2) & 0x04) >> 2
    X = int((change_fun(5) & 0x7f) | ((change_fun(4) & 0x7f) << 7) | ((change_fun(3) & 0x7f) << 14) |
            ((change_fun(2) & 0x07) <<
             21) | High_X << 24 | High_X << 25 | High_X << 26 | High_X << 27 | High_X << 28
            | High_X << 29 | High_X << 30 | High_X << 31)

    High_Y = (change_fun(6) & 0x40) >> 6
    Y = int((change_fun(7) & 0x7f) | ((change_fun(6) & 0x7f) << 7) |
            High_Y << 12 | High_Y << 13 | High_Y << 14 | High_Y << 15)

    High_A = (change_fun(10) & 0x40) >> 6
    A = int((change_fun(11) & 0x7f) | ((change_fun(10) & 0x7f) << 7) |
            High_A << 12 | High_A << 13 | High_A << 14 | High_A << 15)

    Tag1 = (change_fun(14) & 0x40) >> 6
    Tag2 = int((change_fun(15) & 0x7f) | ((change_fun(14) & 0x7f) << 7)
               | Tag1 << 12 | Tag1 << 13 | Tag1 << 14 | Tag1 << 15)

    Tag3 = (change_fun(16) & 0x40) >> 6
    Tag4 = int((change_fun(17) & 0x7f) | ((change_fun(16) & 0x7f) << 7)
               | Tag3 << 12 | Tag3 << 13 | Tag3 << 14 | Tag3 << 15)

    _Tag = int((Tag2 << 16) | Tag4)

    if(X > 65536*65536/2):
        X = X - 65536*65536
    if(Y > 32768):
        Y = Y - 65536
    if(A > 32768):
        A = A - 65536
    if(_Tag > 32768):
        _Tag = _Tag - 65536

    print("X = ", X)
    print("Y = ", Y)
    print("A = ", A)
    print("Tag = ", _Tag)
    print("dis_err = ", (X**2 + Y**2)**0.5)
    msg = send()
    msg.x = X
    msg.y = Y
    msg.A = A
    msg.Tag = _Tag
    pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('python_qr', anonymous=True)

    try:
        print('====== input setting ======')
        input_argv = sys.argv
        input_port = input_argv[1]
        input_baudrate = input_argv[2]
    except:
        print('====== defalt setting ======')
        input_port = "/dev/ttyUSB0"
        input_baudrate = "115200"
    print("port: " + input_port)
    print("baudrate: " + input_baudrate)

    # 串口設定
    ser = serial.Serial(input_port, baudrate=input_baudrate, bytesize=8,
                        parity=serial.PARITY_EVEN, stopbits=1, timeout=0.01)

    pub = rospy.Publisher('qrcode', send, queue_size=10)

    # 白光初始設定
    packet1 = bytearray()
    packet1.append(0xE8)
    packet1.append(0x17)
    # print("send > " + str(len(packet1)))
    print("===== white light setting =======")
    ser.write(packet1)

    ser.read(21)  # 清乾淨

    try:
        rate = rospy.Rate(10)  # 10hz
        s = []
        while not rospy.is_shutdown():
            packet = bytearray()
            packet.append(0xC8)
            packet.append(0x37)
            # print("send > " + str(len(packet)))
            # print("===== command binary =======")
            ser.write(packet)

            # 方法一：單純讀封包
            #s = ser.read(21).encode('hex')

            # 方法二：位移處理
            #p = ser.read(21).encode('hex')
            # for i in range(21*2):
            #    if(i > 33):
            #        s.append(p[i-34])
            #    else:
            #        s.append(p[i+6])
            #print p

            # 方法三：讀n個封包後清除垃圾
            # s = ''
            # for i in range(30):
            #     s += ser.read(1).encode('hex')
            # while 1:
            #     if(s[0:2] == '73' or s[0:2] == 'f3'):
            #         s = s[2:]
            #     else:
            #         break

            # 方法四：讀一段時間的封包後清除垃圾
            s = ser.read(30).encode('hex')
            s = s[-42:]

            # print s



            if(len(s) > 20):
                print('==================')
                receive_data()

            s = []
            # ser.read(21)  # 清乾淨
            rate.sleep()

    except rospy.ROSInterruptException:
        ser.close()
        print('error!')
        pass
