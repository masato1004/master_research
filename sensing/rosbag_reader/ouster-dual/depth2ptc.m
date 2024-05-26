file_name = "1708413010.220381.png";
depthImage = imread("depth_imgs/"+file_name);
colorImage = imread("color_imgs/"+file_name);

focalLength      = [1070.6, 1070.58];
principalPoint   = [964.41, 547.833];
imageSize        = size(colorImage,[1,2]);
RadialDistortion = [-0.0546184,0.0269859,-0.0105149];
TangentialDistortion = [3.24293e-05,-0.000156357];
intrinsics       = cameraIntrinsics(focalLength,principalPoint,imageSize,"RadialDistortion",RadialDistortion,"TangentialDistortion",TangentialDistortion);
depthScaleFactor = 65535/15; % Z = 深度画像[u,v]/深度スケール係数 ここが違う
maxCameraDepth   = 15;

ptCloud = pcfromdepth(depthImage,depthScaleFactor,intrinsics,ColorImage=colorImage);
pcshow(ptCloud);