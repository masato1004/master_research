import cv2
import numpy as np
import os
import sys
import glob
import time


cropwidth = 1512
cropheight = 592
start_h = 1282
start_w = 1164
# input image size: 3840x2160
def cropper(img):
    # cropwidth = 1740
    # cropheight = 630
    # start_h = 1205
    # start_w = 1050
    cropped_img = img[start_h:start_h+cropheight,start_w:start_w+cropwidth]

def dataset_transfer(current_dir,path_dict,destination_dir,cropwidth=cropwidth,cropheight=cropheight,unique_name=""):
    # nowdatasetdir = os.path.join(os.getcwd(), current_dir)
    temp_dirname = os.path.join(os.getcwd(), f'dataset/divp_dataset')
    # temp_dirname = os.path.join(os.getcwd(), f'{destination_dir}')
    new_dirname = os.path.join(os.getcwd(), f'{destination_dir}_{cropwidth}x{cropheight}')

    if not os.path.exists(new_dirname):
        os.mkdir(new_dirname)
    # print(f"'{current_dir}' start")
    for pos in os.listdir(temp_dirname):
        for root, dirs, files in os.walk(os.path.join(temp_dirname,pos)):
    # for root, dirs, files in os.walk(os.path.join(nowdatasetdir)):
            new_root = root.replace(temp_dirname,new_dirname)
            if not os.path.exists(new_root):
                os.mkdir(new_root)
            # if len(files) != 0:

            dir_id = 0
            if "test_depth_completion_anonymous\\" in root:
                dir_id = 4
            elif "val_selection_cropped\\" in root:
                dir_id = 3
            elif "val\\2024_12_11_drive_0001_sync\\" in root:
                dir_id = 2
            elif "train\\2024_12_11_drive_0001_sync\\" in root:
                dir_id = 1

            current_path_list = []
            data_type = ""
            if "groundtruth" in root:
                data_type = "groundtruth"
            elif "uneven_label" in root:
                data_type = "uneven_label"
            elif "velodyne_raw" in root:
                data_type = "velodyne_raw"
            elif "image" in root:
                data_type = "image"

            if data_type != "":
                current_path_list = path_dict[dir_id][data_type]
                print("")
                # print(f"    {new_root}")

            if 'Thumbs.db' in current_path_list:
                current_path_list.remove('Thumbs.db')

                # for i,file in enumerate(files):
                    
            for k, path in enumerate(current_path_list):
                # height, width = img.shape[:2]
                file = path.replace(current_dir,'')
                idx = file.find('/')
                file = file[idx+1:]

                if not os.path.exists(os.path.join(new_root,unique_name+file)):
                    # crop
                    e = 0
                    while e<10:
                        try:
                            # img = cv2.imread(os.path.join(root,file), cv2.IMREAD_UNCHANGED)
                            img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
                            # save
                            wrote = cv2.imwrite(os.path.join(new_root,unique_name+file),img)
                            if not wrote:
                                print('Failed to write.')
                            else:
                                print(f"\r    {k+1}/{len(current_path_list)}: '{path}' -> '{new_root}'",end="")
                                break
                        except TypeError:
                            e += 1
                    else:
                        print('    Read error')
                        print(f'    {os.path.join(root,file)}')
                        pass
                        # sys.exit()

    # print(f"\n'{current_dir}' Finished\n")

def path_loader(current_dir):

    for pos in os.listdir(current_dir):
        path = os.path.join(current_dir,pos+"/*.png").replace("\\","/")
        #
        if pos == 'gt':
            gt_files = glob.glob(path)
            gt_files.pop(0)
            if 'Thumbs.db' in gt_files:
                gt_files.remove('Thumbs.db')
            gt_files = [f.replace("\\","/") for f in gt_files]
        #
        elif pos == 'label':
            label_files = glob.glob(path)
            label_files.pop(0)
            if 'Thumbs.db' in label_files:
                label_files.remove('Thumbs.db')
            label_files = [f.replace("\\","/") for f in label_files]
        #
        elif pos == 'lidar':
            lidar_files = glob.glob(path)
            if 'Thumbs.db' in lidar_files:
                lidar_files.remove('Thumbs.db')
            lidar_files = [f.replace("\\","/") for f in lidar_files]
        #
        elif pos == 'rgb':
            rgb_files = glob.glob(path)
            rgb_files.pop(0)
            if 'Thumbs.db' in rgb_files:
                rgb_files.remove('Thumbs.db')
            rgb_files = [f.replace("\\","/") for f in rgb_files]
    return gt_files,label_files,lidar_files,rgb_files

if __name__ == "__main__":
    unique_name = "city1_"
    current_dir = 'dataset/20250114/'
    destination_dir = 'dataset/divp_dataset_5'
    # 1: train, 2: val, 3: val_selection_cropped, 4: test_depth_completion_anonymous
    data_sort = [1,2,1,3,1,4,1]
    # data_sort = [1,2,1,3,1,2,1]
    # data_sort = [4,3,4,3,4,3,4]
    max_len = 40*70

    gt_files,label_files,lidar_files,rgb_files = path_loader(current_dir)

    path_dict = {
        1:{"image":[],"velodyne_raw":[],"uneven_label":[],"groundtruth":[]},
        2:{"image":[],"velodyne_raw":[],"uneven_label":[],"groundtruth":[]},
        3:{"image":[],"velodyne_raw":[],"uneven_label":[],"groundtruth":[]},
        4:{"image":[],"velodyne_raw":[],"uneven_label":[],"groundtruth":[]}
    }
    i = 0
    
    last_len = 0
    while True:
        # print(f"Current: {len(lidar_files)}")
        if last_len >= max_len:
            break
        if last_len < len(lidar_files):
            last_len = len(lidar_files)
            # current_len = len(lidar_files)
            # if last_len < current_len:
            while i < len(lidar_files):
                # print(i)
                dir_id = data_sort[int(np.mod(i,7))]
                path_dict[dir_id]["image"].append(rgb_files[i])
                path_dict[dir_id]["velodyne_raw"].append(lidar_files[i])
                path_dict[dir_id]["uneven_label"].append(label_files[i])
                path_dict[dir_id]["groundtruth"].append(gt_files[i])
                i += 1
            else:
                print("Add-path done")
                dataset_transfer(current_dir,path_dict,destination_dir,unique_name=unique_name)
        
        while True:
            new_gt_files,new_label_files,new_lidar_files,new_rgb_files = path_loader(current_dir)
            if len(new_gt_files)==len(new_label_files) and len(new_gt_files)==len(new_lidar_files):
                break
        [gt_files.append(t) for t in new_gt_files if t not in gt_files]
        [label_files.append(t) for t in new_label_files if t not in label_files]
        [lidar_files.append(t) for t in new_lidar_files if t not in lidar_files]
        [rgb_files.append(t) for t in new_rgb_files if t not in rgb_files]
        gt_files,label_files,lidar_files,rgb_files = sorted(gt_files),sorted(label_files),sorted(lidar_files),sorted(rgb_files)
        time.sleep(1)
