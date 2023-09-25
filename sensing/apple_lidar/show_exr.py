from cmath import nan
import numpy as np
import cv2
import numpy as np
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from statistics import mean
import matplotlib.animation as anim

Inter_Param = [797.81427001953125, 0, 0, 0, 797.81427001953125, 0, 360.17623901367188, 476.64434814453125, 1]
f  = Inter_Param[0]
cx = Inter_Param[6]
cy = Inter_Param[7]

# openpose = np.load(r"data\npy\openpose.npy", allow_pickle=True)
exr = np.load(r"exr.npy", allow_pickle=True)
print(np.shape(exr))

pose_3d = np.full((len(exr),2,25,4),nan)
# for f_num, frame in enumerate(exr): # フレームの数だけループ
    # print(np.shape(frame))
    # print(f_num)
    # for p_num, pose in enumerate(frame): # 各フレーム内で検出した人の数だけループ
    # for i in range(25): # 関節の数(25)だけループ
    #     # print(pose)
    #     x = pose[i,0]
    #     if x > 959:
    #         x = 959
    #     y = pose[i,1]
    #     confidence = pose[i,2]
    #     # print(x, y, confidence)
    #     exr_x = int(x*255/959) # exr:256x192, jpg:720x960なのでサイズ調整
    #     exr_y = int(191 - (y*191/719))
        
    #     d = exr[exr_x,exr_y,f_num] # 各画素におけるexrファイルの値（深度変換前）

    #     if not np.isnan(d):
    #         pose_3d[f_num][p_num][i]=[(x-cx)/f*d,-(y-cy)/f*d,d,confidence] # 各画素のx,y,z