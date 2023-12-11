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
import time
import struct
import binascii
import ctypes

if __name__ == '__main__':

        # Serial port 設定
        ser = serial.Serial()
        ser.port = "COM5"  # ポート
        ser.timeout=1.0                                # タイムアウト
        ser.baudrate = 115200                          # ボーレート

        # Serial port Open
        ser.open() 
        
        # 加速度・角速度パラメータ設定
        header = 0x9A
        cmd = 0x16
        data = 0x01
        data1 = 0x0A
        data2 = 0x00
        
        check = header ^ cmd
        check = check ^ data
        check = check ^ data1
        check = check ^ data2

        print(ser)
	
        list = bytearray([header,  cmd,  data,  data1, data2 , check])

        #バッファクリア
        ser.read(1000)
        ser.write(list)

        str = ser.readline()

        # print('CmdRes:' + repr(str))
	
        # 計測開始
        header = 0x9A
        cmd    = 0x13
        smode   = 0x00
        syear  = 0x00
        smonth = 0x01
        sday   = 0x01
        shour  = 0x00
        smin   = 0x00
        ssec   = 0x00
        emode  = 0x00
        eyear  = 0x00
        emonth = 0x01
        eday   = 0x01
        ehour  = 0x00
        emin   = 0x00
        esec   = 0x00
        check = header ^ cmd
        check = check ^ smode
        check = check ^ syear
        check = check ^ smonth
        check = check ^ sday
        check = check ^ shour
        check = check ^ smin
        check = check ^ ssec
        check = check ^ emode
        check = check ^ eyear
        check = check ^ emonth
        check = check ^ eday
        check = check ^ ehour
        check = check ^ emin
        check = check ^ esec
	
        list = bytearray([header,  cmd, smode, syear, smonth, sday, shour, smin, ssec, emode, eyear, emonth, eday, ehour, emin, esec, check])
  
        # バッファクリア
        ser.read(100)
        ser.write(list)
	
        str = ser.readline()
	
        # 計測開始通知
        str =ser.read(1)
	
        # ヘッダ検索
        while ord(str) != 0x9A:
                str = ser.read(1)

        # コマンド取得
        str = ser.read(1)
	
        # 加速度角速度計測データ通知のみ処理する
        if ord(str) == 0x80:
                
                # タイムスタンプ
                str = ser.read(4)
                
                # 加速度X
                data1 = ser.read(1)
                data2 = ser.read(1)
                data3 = ser.read(1)

                # 3byteの値を4byteのint型としてマイナスのハンドリング
                if ord(data3) & 0x80:
                        data4 = b'\xFF'
                else:
                        data4 = b'\x00'
		
                print(binascii.b2a_hex(data1))
                print(binascii.b2a_hex(data2))
                print(binascii.b2a_hex(data3))
                print(binascii.b2a_hex(data4))
		
                # エンディアン変換
                accx = ord(data1)
                accx += ord(data2)<<8
                accx += ord(data3)<<16
                accx += ord(data4)<<24

                print("accx = %d" % (ctypes.c_int(accx).value))
        
	
        ser.close();

