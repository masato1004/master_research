import torch
import torchvision.transforms as transforms
import os, sys
from PIL import Image
import glob
import numpy as np
import matlab

sys.path.append('C:/Users/INOUE MASATO/research/depth_completion')
from Utils.utils import str2bool, AverageMeter, depth_read 
import Models
import time

best_file_name = glob.glob(os.path.join('./', 'model_best_epoch*'))[0]

channels_in = 4
model = Models.define_model('mod', in_channels = channels_in)
model = model.cuda()
checkpoint = torch.load(best_file_name)
model.load_state_dict(checkpoint['state_dict'])
model = model.cuda()
to_pil = transforms.ToPILImage()
to_tensor = transforms.ToTensor()
model.eval()

# def depth_completion():
def depth_completion(rgb, lidar, crop_h, crop_w):
    # print(rgb.shape, lidar.shape, crop_h, crop_w)
    # assert rgb.shape == (crop_h, crop_w)
    # assert lidar.shape == (crop_h, crop_w)
    # rgb = Image.fromarray(np.uint8(rgb))
    # lidar = Image.fromarray(np.uint16(lidar))
    rgb = to_tensor(rgb).float()

    lidar = depth_read(lidar, 0.0)
    lidar = to_tensor(lidar).float()
    
    input = torch.unsqueeze(lidar, 0).cuda()
    rgb = torch.unsqueeze(rgb, 0).cuda()
    rgb = rgb*255.0

    input = torch.cat((input, rgb), 1)
    with torch.no_grad():
        torch.cuda.synchronize()
        output, _, _, _  = model(input)
        torch.cuda.synchronize()
        output = torch.clamp(output, min=0, max=255)
        output = output * 256.
        output = output[0][0:1].cpu()
        # pil_img = to_pil(output.int())
        matlab_mat = matlab.double(output.int())
    return matlab_mat
    # return torch.cuda.is_available()