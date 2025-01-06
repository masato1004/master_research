import cv2
import os
import sys
import glob
import time
import numpy as np
import datetime as DT
from tqdm import tqdm
from line import sendLINE
from slack_notify import SendToSlackMessage

cropwidth = 1496
cropheight = 552
start_h = 1271
start_w = 1172
# input image size: 3840x2160
def cropper(img):
    # cropwidth = 1740
    # cropheight = 630
    # start_h = 1205
    # start_w = 1050
    cropped_img = img[start_h:start_h+cropheight,start_w:start_w+cropwidth]

    return cropped_img

def dataset_transfer(current_dir,destination_dir):
    nowdatasetdir = os.path.join(os.getcwd(), current_dir)
    new_dirname = os.path.join(os.getcwd(), f'{destination_dir}')
    # new_dirname = os.path.join(os.getcwd(), f'{destination_dir}_{cropwidth}x{cropheight}')

    if not os.path.exists(new_dirname):
        os.mkdir(new_dirname)

    # print(f"'{current_dir}' start")
    # for pos in os.listdir(nowdatasetdir):
        # for root, dirs, files in os.walk(os.path.join(nowdatasetdir,pos)):
    for root, dirs, files in os.walk(os.path.join(nowdatasetdir)):
        new_root = root.replace(nowdatasetdir,new_dirname)
        if not os.path.exists(new_root):
            os.mkdir(new_root)
        if len(files) != 0:
            # print(f"    {new_root}")
            if 'Thumbs.db' in files:
                files.remove('Thumbs.db')
            found = False
            for i,file in enumerate(files):
                # height, width = img.shape[:2]
                if not os.path.exists(os.path.join(new_root,file)):
                    found = True
                    # crop
                    # e = 0
                    while True:
                        try:
                            img = cv2.imread(os.path.join(root,file), cv2.IMREAD_UNCHANGED)
                            crop_img = cropper(img)
                            # save
                            wrote = cv2.imwrite(os.path.join(new_root,file),crop_img)
                            if not wrote:
                                print('Failed to write.')
                            else:
                                # if 'image' in root:
                                #     cv2.imshow('progress',img)
                                #     cv2.waitKey(100)
                                print(f"\r    {i+1}/{len(files)}: '{current_dir}' -> '{new_root}'",end="")
                                break
                        except Exception as e:
                            pass
                            # e += 1
                    else:
                        print('    Read error')
                        print(f'    {os.path.join(root,file)}')
                        pass
                        # sys.exit()
            if found:
                print("\n")

    # print(f"\n'{current_dir}' Finished\n")

username = 'DatasetMaker'
icon_emoji = ':divp:'

if __name__ == "__main__":
    total_file_num = 20*45
    new_dir = 'dataset/20250106_val'
    if not os.path.exists(new_dir):
        os.mkdir(new_dir)
    dirs = {
        'rgb':{
            'current':'image_front',
            'destination':f'{new_dir}/rgb'
        },
        'lidar':{
            'current':'sparse_depth',
            'destination':f'{new_dir}/lidar'
        },
        'gt':{
            'current':'gt_depth',
            'destination':f'{new_dir}/gt'
        },
        'label':{
            'current':'gt_label',
            'destination':f'{new_dir}/label'
        }
    }

    print("\n\n======Start======\n")
    t = time.time()
    k = -1
    k_time = 0
    last_k_time = 0
    k_time_sum = 0
    notification = False
    # cv2.namedWindow('progress', cv2.WINDOW_NORMAL)
    latest_file_num = len(glob.glob(f"{dirs['lidar']['destination']}/*.png"))
    pbar = tqdm(total=total_file_num,initial=latest_file_num)
    while True:
        files = glob.glob("sparse_depth/*.png")
        if 'Thumbs.db' in files:
            files.remove('Thumbs.db')
        dest_files = glob.glob(f"{dirs['lidar']['destination']}/*.png")
        if 'Thumbs.db' in dest_files:
            dest_files.remove('Thumbs.db')
        if len(files) > len(dest_files):
            time.sleep(2)
            print("\n")
            k += 1
            for key in dirs.keys():
                dataset_transfer(current_dir=dirs[key]['current'],destination_dir=dirs[key]['destination'])
            current_progress = len(dest_files) - latest_file_num
            latest_file_num = len(dest_files)
            # pbar.update(current_progress)
            # print("\n")
            k_time = time.time() - last_k_time
            last_k_time = time.time()
            notification = True
            if k >= 1:
                k_time_sum += k_time

        if len(dest_files) == total_file_num:
            current_progress = len(dest_files) - latest_file_num
            latest_file_num = len(dest_files)
            pbar.update(current_progress)
            break
        else:
            now = time.time()
            ss = round(now-t)
            h = int(ss/3600)
            m = int((ss-h*3600)/60)
            s = int(ss-h*3600-m*60)

            ETA = DT.datetime.now()
            if k >= 1:
                ETA_ss = round((k_time_sum)*(total_file_num-len(files)))/k
                ETA+=DT.timedelta(seconds=ETA_ss)
            else:
                ETA_ss = round((k_time)*(total_file_num-len(files)))
            ETA_h = int(ETA_ss/3600)
            ETA_m = int((ETA_ss-ETA_h*3600)/60)
            ETA_s = int(ETA_ss-ETA_h*3600-ETA_m*60)
            current_progress = len(dest_files) - latest_file_num
            latest_file_num = len(dest_files)
            pbar.update(current_progress)
            # print(f"\r{len(files)} files:: RunTime {h}:{'0'*(2-len(str(m)))}{m}:{'0'*(2-len(str(s)))}{s} :: ETA {ETA.month}/{ETA.day}-{ETA.hour}:{ETA.minute}==={ETA_h}:{'0'*(2-len(str(ETA_m)))}{ETA_m}:{'0'*(2-len(str(ETA_s)))}{ETA_s} left",end="")
            if int(np.mod(len(files),25)) == 0 and notification:
                notification = False
                once_ss = round(k_time_sum/k)
                once_m = int((once_ss)/60)
                once_s = int(once_ss-once_m*60)
                message = f"\n\n===Dataset making progress===\n\nSimulation outs {len(files)}/{total_file_num} files\nDIVPsim speed: {once_m}m {once_s}s\nETA: {ETA.month}/{ETA.day} {ETA.hour}:{ETA.minute}"
                # sendLINE(f"\n\n===Dataset making progress===\n\nSimulation outs {len(files)}/{total_file_num} files\nDIVPsim speed: {once_m}m {once_s}s\nETA: {ETA.month}/{ETA.day} {ETA.hour}:{ETA.minute}")
                SendToSlackMessage(message, username, icon_emoji)
            time.sleep(0.2)
    pbar.close()
    print("\n\n======Finish======\n\n")
    # sendLINE("\n\n= Dataset making has finished =")
    SendToSlackMessage("\n\n= Dataset making has finished =", username, icon_emoji)
    # cv2.destroyAllWindow()