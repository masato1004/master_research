list_depth_imgs = dir("depth_imgs/*.png");
list_color_imgs = dir("color_imgs/*.png");

file_name = "1708413010.220381.png";
depthImage_read = imread("depth_imgs/"+list_depth_imgs(1).name);
colorImage_read = imread("color_imgs/"+list_color_imgs(1).name);

%% create original size images for pcfromdepth as new ones
depthImage_original_size=uint16(zeros(1080, 1920));
colorImage_original_size=uint8(ones(1080, 1920, 3));

start_x = 353-1;
start_y = 449-1;
rect_width = 1216-1;
rect_height = 352-1;
depthImage_original_size(start_y:start_y+rect_height,start_x:start_x+rect_width) = depthImage_read;
colorImage_original_size(start_y:start_y+rect_height,start_x:start_x+rect_width,:) = colorImage_read;
depthImage = depthImage_original_size;
colorImage = colorImage_original_size;

%% pc from depth
focalLength      = [1070.6, 1070.58];
principalPoint   = [964.41, 547.833];
imageSize        = size(colorImage,[1,2]);  % 352 1216
% imageSize        = [1080, 1920];
RadialDistortion = [-0.0546184,0.0269859,-0.0105149];
TangentialDistortion = [3.24293e-05,-0.000156357];
intrinsics       = cameraIntrinsics(focalLength,principalPoint,imageSize,"RadialDistortion",RadialDistortion,"TangentialDistortion",TangentialDistortion);
depthScaleFactor = 65535/15; % Z = 深度画像[u,v]/スケールファクタ
maxCameraDepth   = 15;

%% translat with position parameter from cad
rotate_angle_cam2wheel = [0 0 0];
f_translation_cam2wheel = [0 1 0]; % from cad
r_translation_cam2wheel = [-1 0 0]; % from cad
f_tform_cam2wheel = rigidtform3d(rotate_angle_cam2wheel,f_translation_cam2wheel);
r_tform_cam2wheel = rigidtform3d(rotate_angle_cam2wheel,r_translation_cam2wheel);

ptCloud = pcfromdepth(depthImage,depthScaleFactor,intrinsics,ColorImage=colorImage);
tform = rigidtform3d([-90 0 -90],[0 0 0]);
ptCloud = pctransform(ptCloud,tform);
pcshow(ptCloud);

pcin=pointCloud(reshape(ptCloud.Location,[],3));
downptCloud = pcdownsample(ptCloud,'gridAverage',0.50);
[ptCloud, plaen_mesh, plane_tform] = fitplane(pcin,downptCloud,0.01);
ptCloud = pctransform(ptCloud,r_tform_cam2wheel);
pcshow(reshape(ptCloud.Location,[],3),reshape(colorImage,[],3));
