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


class Tsnd:
    """TSND151-control class
    ---
    arg: portname (default -> "COM5")
        Define the path or pathname of the port for serial communication with TSND151.
    """
    def __init__(self,portname:str ="COM5"):          
        # Serial port 設定
        self.ser = serial.Serial()
        self.ser.port = portname  # ポート
        self.ser.timeout=1.0                                # タイムアウト
        self.ser.baudrate = 115200                          # ボーレート

        # Serial port Open
        self.ser.open()

    def setup_all(self):
        self.accparam_setup()
        self.airpressureparam_setup()
        self.adparam_setup()
        self.batteryparam_setup()
        self.ad_setup()

    def accparam_setup(self):
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

        print(self.ser)

        list = bytearray([header,  cmd,  data,  data1, data2 , check])

        #バッファクリア
        self.ser.read(1000)
        self.ser.write(list)

        str = self.ser.readline()

        # print('CmdRes:' + repr(str))

    def airpressureparam_setup(self):
        # 加速度・角速度パラメータ設定
        header = 0x9A
        cmd = 0x1A
        rate = 0x04
        ave = 0x01
        rec = 0x01
        
        check = header ^ cmd
        check = check ^ rate
        check = check ^ ave
        check = check ^ rec

        list = bytearray([header,  cmd,  rate,  ave, rec , check])

        #バッファクリア
        self.ser.read(1000)
        self.ser.write(list)

        str = self.ser.readline()

        # print('CmdRes:' + repr(str))

    def ad_setup(self):
        # 送信コマンド
        header = 0x9A
        cmd = 0x30
        term1 = 0x00
        term2 = 0x00
        term3 = 0x0A
        term4 = 0x0A
        check = header ^ cmd
        check = check ^ term1
        check = check ^ term2
        check = check ^ term3
        check = check ^ term4

        self.list = bytearray([header, cmd, term1, term2, term3, term4, check])

        # バッファクリア
        self.ser.read(100)
        self.ser.write(self.list)


        # # 計測開始通知
        # str =self.ser.read(1)

        # # ヘッダ検索
        # while ord(str) != 0x9A:
        #         str = self.ser.read(1)
        # print("header",ord(str))
    
    def batteryparam_setup(self):
        # 加速度・角速度パラメータ設定
        header = 0x9A
        cmd  = 0x1C
        send = 0x01
        rec  = 0x00
        
        check = header ^ cmd
        check = check ^ send
        check = check ^ rec

        list = bytearray([header,  cmd,  send, rec, check])

        #バッファクリア
        self.ser.read(1000)
        self.ser.write(list)

        str = self.ser.readline()

    def adparam_setup(self):
        # 加速度・角速度パラメータ設定
        header = 0x9A
        cmd  = 0x1E
        rate = 0x02
        ave  = 0x01
        rec1 = 0x01
        edge = 0x01
        rec2 = 0x00
        
        check = header ^ cmd
        check = check ^ rate
        check = check ^ ave
        check = check ^ rec1
        check = check ^ edge
        check = check ^ rec2

        list = bytearray([header,  cmd,  rate,  ave, rec1, edge, rec2, check])

        #バッファクリア
        self.ser.read(1000)
        self.ser.write(list)

        str = self.ser.readline()
        
    def start(self):

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
        self.ser.read(100000)
        self.ser.write(list)
        
        # ヘッダ検索
        # str = self.ser.read(1)
        # while ord(str) != 0x84:
        #     str = self.ser.read(1)

    def get_datas(self):
        # self.ser.read(100000)
        str = self.ser.readline()
        # print('CmdRes:' + repr(str))

        # 計測開始通知
        str =self.ser.read(1)

        # ヘッダ検索
        while ord(str) != 0x9A:
            str = self.ser.read(1)

        # self.__get_acc()
        # self.__get_airpressure()
        self.__get_ad()
        # 加速度角速度計測データ通知のみ処理する
        # for i in range(1000):
        # コマンド取得
        
    def __get_acc(self):
        str = self.ser.read(1)
        
        # イベントコマンド検索
        while ord(str) != 0x80:
            str = self.ser.read(1)

        if ord(str) == 0x80:
                
            # タイムスタンプ
            str = self.ser.read(4)
            
            # 加速度X
            data1 = self.ser.read(1)
            data2 = self.ser.read(1)
            data3 = self.ser.read(1)

            # 3byteの値を4byteのint型としてマイナスのハンドリング
            if ord(data3) & 0x80:
                    data4 = b'\xFF'
            else:
                    data4 = b'\x00'
            
            # print(binascii.b2a_hex(data1))
            # print(binascii.b2a_hex(data2))
            # print(binascii.b2a_hex(data3))
            # print(binascii.b2a_hex(data4))
            
            # エンディアン変換
            accx = ord(data1)
            accx += ord(data2)<<8
            accx += ord(data3)<<16
            accx += ord(data4)<<24

            print("accx = %d" % (ctypes.c_int(accx).value))

    def __get_ad(self):
        # コマンド取得
        str = self.ser.read(1)
        
        # イベントコマンド検索
        while ord(str) != 0x84:
            str = self.ser.read(1)
            # if ord(str) == 0x9a:
            #     return

        if ord(str) == 0x84:
            # 不要端子情報
            str = self.ser.read(5)
            
            # 端子3
            term3_1 = self.ser.read(1)
            term3_2 = self.ser.read(1)
            
            # 端子4
            term4_1 = self.ser.read(1)
            term4_2 = self.ser.read(1)
            
            print(binascii.b2a_hex(term3_1))
            print(binascii.b2a_hex(term3_2))
            print(binascii.b2a_hex(term4_1))
            print(binascii.b2a_hex(term4_2))
            
            # エンディアン変換
            term3_in = ord(term3_1)
            term3_in += ord(term3_2)<<8
            term4_in = ord(term4_1)
            term4_in += ord(term4_2)<<8

            print("term3_in = %d" % (ctypes.c_int(term3_in).value))
            print("term4_in = %d" % (ctypes.c_int(term4_in).value*3000/4095))
        
    def __get_airpressure(self):
        str = self.ser.read(1)

        # イベントコマンド検索
        while ord(str) != 0x82:
            str = self.ser.read(1)

        if ord(str) == 0x82:
            # 不要端子情報
            str = self.ser.read(4)
            
            # 気圧データ
            data1 = self.ser.read(1)
            data2 = self.ser.read(1)
            data3 = self.ser.read(1)
            
            # 3byteの値を4byteのint型としてマイナスのハンドリング
            # if ord(data3) & 0x82:
            #         data4 = b'\xFF'
            # else:
            #         data4 = b'\x00'
            # print(binascii.b2a_hex(data1))
            # print(binascii.b2a_hex(data2))
            # print(binascii.b2a_hex(data3))
            
            # エンディアン変換
            data = ord(data1)
            data += ord(data2)<<8
            data += ord(data3)<<16
            # data += ord(data4)<<24

            print("AirPressure = %d [hPa]" % (ctypes.c_int(data).value/100))

    def __get_ad_setup(self):
        # 送信コマンド
        header = 0x9A
        cmd = 0x31
        option = 0x00
        check = header ^ cmd
        check = check ^ option
        
        self.list = bytearray([header, cmd, option, check])
        
        # バッファクリア
        self.ser.read(1000)
        self.ser.write(self.list)

        str = self.ser.readline()

        # 計測開始通知
        str =self.ser.read(1)

        # コマンド検索
        k=0
        while True:
            k+=1
            while ord(str) != 0x9A:
                str = self.ser.read(1)
            # print("__get_ad_setup:",binascii.b2a_hex(str))

            str = self.ser.read(1)
            if ord(str) == 0xB1:
                # 端子情報取得
                term1 = self.ser.read(1)
                term2 = self.ser.read(1)
                term3 = self.ser.read(1)
                term4 = self.ser.read(1)
                
                print(ord(term1))
                print(ord(term2))
                print(ord(term3))
                print(ord(term4))
                # print(binascii.b2a_hex(term1))
                # print(binascii.b2a_hex(term2))
                # print(binascii.b2a_hex(term3))
                # print(binascii.b2a_hex(term4))

                print("term3_mode = %d" % (ctypes.c_int(ord(term3)).value))
                print("term4_mode = %d" % (ctypes.c_int(ord(term4)).value))
                break
            if k>1000:
                break

    def __del__(self):
        self.ser.close();


if __name__ == '__main__':

    tsnd = Tsnd('COM5')
    tsnd.setup_all()
    tsnd.start()
    for i in range(100):
        tsnd.get_datas()
    del tsnd