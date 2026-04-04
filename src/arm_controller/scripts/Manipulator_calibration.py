#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (C) 2023 reinovo, Inc. All Rights Reserved 
#
# @Time    : 2023/12/28 下午5:42
# @Author  : hmm
# @Email   : liuyuhang0531@foxmail.com
# @File    : Manipulator_calibration.py
import threading
import serial
import serial.tools.list_ports

class Arm_alignment(object):
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        self.stop_thread = False

    def send_data(self):
        while True:
            data = input("\n输入:")
            self.ser.write(f'{data} + \n'.encode('utf-8'))

    def alignment(self):
        print("\n准备机械臂校准定位图，将机械臂底座于定位图对齐")
        data = input("\n按下回车以继续")
        print('*' * 50)
        self.ser.write('M2019\n'.encode('utf-8'))
        # print(self.data)
        print("\n检查末端工具是否拆卸\n移动机械臂到校准定位图中的B点\n")
        data = input("\n按下回车以继续")
        print('*' * 50)
        self.ser.write('M2401 B\n'.encode('utf-8'))
        print(self.data)
        print("!!!!!!!!!!!!!!!!!标定结束")
        self.stop_thread = True


    def monitor_data(self):
        while not self.stop_thread:
            self.data = self.ser.readline().decode('utf-8').strip()
            # print("\nReceived data:",self.data)

if __name__ == "__main__":
    try:
        print('*' * 50)
        arm = Arm_alignment()
        print("!!!!串口打开成功")
        print('*' * 50)
    except serial.SerialException:
        ports_list = list(serial.tools.list_ports.comports())
        if len(ports_list) <= 0:
            print("串口未连接，请检查USB是否连接。")

        for comport in ports_list:
            comport = list(comport)
            if comport[0] == '/dev/ttyACM0':
                print("!!!!串口无权限，可通过在终端运行'sudo chmod 666 /dev/ttyACM0'。")
            else:
                print("串口未连接，请检查USB是否连接。")
        print('*' * 50)
    monitor_thread = threading.Thread(target=arm.monitor_data)
    monitor_thread.start()
    arm.alignment()
    monitor_thread.join()









