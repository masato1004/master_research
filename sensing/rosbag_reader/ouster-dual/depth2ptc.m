file_name = "1708413010.220381.png";
depthImage = imread("depth_imgs/"+file_name);
colorImage = imread("color_imgs/"+file_name);

focalLength      = [1070.6, 1070.58];
principalPoint   = [964.41, 547.833];
imageSize        = size(colorImage,[1,2]);
intrinsics       = cameraIntrinsics(focalLength,principalPoint,imageSize);
depthScaleFactor = 65535/20; % Z = 深度画像[u,v]/深度スケール係数 ここが違う
maxCameraDepth   = 20;

ptCloud = pcfromdepth(depthImage,depthScaleFactor,intrinsics,ColorImage=colorImage);
pcshow(ptCloud);