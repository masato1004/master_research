import numpy as np
import matplotlib.pyplot as plt
import glob
import OpenEXR, Imath

# OpenEXRは以下からインストール
# https://www.lfd.uci.edu/~gohlke/pythonlibs/#openexr
# 対応cpを調べるコード
# from pip._internal.utils.compatibility_tags import get_supported
# print(get_supported())

# filePath = "D:\\testdata\\2022-10-22--15-31-51\\EXR_RGBD\depth\\0.exr"
# exr_files = glob.glob(r"C:\Users\mitsuhiro\Documents\Depth_EXR\data\2022-10-22--15-29-48\EXR_RGBD\depth\*")
# exr_files = glob.glob(r"C:\Users\mitsuhiro\Documents\2022\Depth_EXR\data\2022-10-22--15-29-48\EXR_RGBD\depth\*")
exr_files = glob.glob(r"depth\*")

# print([exr_files[10]])

for i, file in enumerate(exr_files):
    pt = Imath.PixelType(Imath.PixelType.FLOAT)
    img_exr = OpenEXR.InputFile(file)

    # print(img_exr.header())
    raw_bytes = img_exr.channel('R', Imath.PixelType(Imath.PixelType.FLOAT))
    depth_vector = np.frombuffer(raw_bytes, dtype=np.float32)

    dw = img_exr.header()['dataWindow']
    size = (dw.max.x - dw.min.x + 1, dw.max.y - dw.min.y + 1)
    # print(size)

    depth_map = np.reshape(depth_vector, (size[1], size[0]))
    if i == 0:
        depth_npy = depth_map
    else:
        depth_npy = np.dstack([depth_npy, depth_map])
    
    # if i == 120:
    #     depth_map = np.rot90(depth_map)
    #     print(depth_map)
    #     plt.imshow(depth_map, "viridis_r")
    #     plt.savefig(r"graph\test_120.png")


np.save('exr', depth_npy)

# color mapの表示

# depth_map = np.rot90(depth_map)
print(depth_map)
plt.imshow(depth_map)
plt.savefig(r"test.png")


# debug memo
# print(depth_npz.shape) : (256, 192, 301)
# print(depth_npz)
# print(depth_npz[0,0,0])
# print(depth_npz[0,0,1]) # right in same frame
# print(depth_npz[0,1,0]) # down in same frame
# print(depth_npz[1,0,0]) # next frame