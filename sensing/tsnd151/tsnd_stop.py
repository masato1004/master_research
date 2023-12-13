# -*- coding: utf-8 -*-
#
# TSDN sample program for Python3
#
# required:
#     PySerial
#
import os
import sys
import serial
import subprocess
import time
import struct
import binascii
import ctypes

if __name__ == '__main__':

    # subprocess.run(('sudo', '-S',
    #                 'chmod', '666', '/dev/ttyACM0'),
    #                input='masatontoI104\n'.encode(), check=True)
    # Serial port 設定
    ser = serial.Serial()
    # ser.port = "COM5"  # ポート
    ser.port = "/dev/ttyACM0"  # ポート
    ser.timeout=1.0                                # タイムアウト
    ser.baudrate = 115200                          # ボーレート

    # Serial port Open
    ser.open() 
    
    # 計測終了
    header = 0x9A
    cmd = 0x15
    option = 0x00
    # smode   = 0x00
    # syear  = 0x00
    # smonth = 0x01
    # sday   = 0x01
    # shour  = 0x00
    # smin   = 0x00
    # ssec   = 0x00
    # emode  = 0x00
    # eyear  = 0x00
    # emonth = 0x01
    # eday   = 0x01
    # ehour  = 0x00
    # emin   = 0x00
    # esec   = 0x00
    check = header ^ cmd
    check = check ^ option
    # check = check ^ smode
    # check = check ^ syear
    # check = check ^ smonth
    # check = check ^ sday
    # check = check ^ shour
    # check = check ^ smin
    # check = check ^ ssec
    # check = check ^ emode
    # check = check ^ eyear
    # check = check ^ emonth
    # check = check ^ eday
    # check = check ^ ehour
    # check = check ^ emin
    # check = check ^ esec

    list = bytearray([header, cmd, option, check])
    # list = bytearray([header,  cmd, smode, syear, smonth, sday, shour, smin, ssec, emode, eyear, emonth, eday, ehour, emin, esec, check])
    
    # バッファクリア
    ser.read(1000)
    ser.write(list)

    str = ser.readline()

    # 計測終了通知
    str =ser.read(1)
    
    # ヘッダ検索
    while ord(str) != 0x9A:
        str = ser.read(1)
    
    # コマンド取得
    str = ser.read(1)
    
    print('計測終了')

    ser.close();

