import cv2
import os

cropwidth = 1216
cropheight = 176
upward = True
leftward = False

nowdatasetdir = os.path.join(os.getcwd(), 'denoised_labeled_supervision_cropped')
new_dirname = os.path.join(os.getcwd(), f'denoised_labeled_supervision_{cropwidth}x{cropheight}')

if not os.path.exists(new_dirname):
    os.mkdir(new_dirname)

for pos in os.listdir(nowdatasetdir):
    for root, dirs, files in os.walk(os.path.join(nowdatasetdir,pos)):
        new_root = root.replace(nowdatasetdir,new_dirname)
        if not os.path.exists(new_root):
            os.mkdir(new_root)
        if len(files) != 0:
            print(new_root)
            for file in files:
                img = cv2.imread(os.path.join(root,file), cv2.IMREAD_UNCHANGED)
                height, width = img.shape[:2]
                if upward and leftward:
                    crop_img = img[-cropheight:, -cropwidth:]
                elif upward and not leftward:
                    crop_img = img[-cropheight:, 0:cropwidth]
                elif not upward and leftward:
                    crop_img = img[0:cropheight, -cropwidth:]
                else:
                    crop_img = img[0:cropheight, 0:cropwidth]
                wrote = cv2.imwrite(os.path.join(new_root,file),crop_img)
                if not wrote:
                    print('Failed to write.')

print("\nFinish")