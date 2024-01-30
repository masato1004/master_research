%% after ZED_Mapping.m
%% mapping datas
% C = vertices;
% min_ver=min(vertices(:,3),[],'all');
% C(:,3) = C(:,3)+abs(min_ver);
% max_ver = max(C(:,3),[],'all');
% for r = 1:size(C)
%     C(r,1) = C(r,3)/max_ver;
%     C(r,2) = C(r,3)/max_ver;
%     C(r,3) = C(r,3)/max_ver;
% end
% figure;
% scatter3(vertices(:,1),vertices(:,2),vertices(:,3),S,C);

% ptc_ver = pointCloud(low);
ptc_ver = pointCloud(vertices);
figure
pcshow(ptc_ver);

%% RANSAC0
R = 500;
T = 0.04;
K = 0.1;
% data_size = size(vertices);
% data_num = data_size(1);
% [a_,b_,c_,d_] = ransac_func(vertices, R, T, fix(data_num*K));
% X_max = max(vertices(:,1),[],'all');
% X_min = min(vertices(:,1),[],'all');
% Y_max = max(vertices(:,2),[],'all');
% Y_min = min(vertices(:,2),[],'all');
% [X_pre, Y_pre] = meshgrid(X_min:(X_max-X_min)/8:X_max,Y_min:(Y_max-Y_min)/8:Y_max);
% Z_pre = -a_*X_pre/c_ - b_*Y_pre/c_ - d_/c_;
% hold on;
% mesh(X_pre,Y_pre,Z_pre,"AlphaData",0.5,"EdgeColor","g","FaceColor","none");

%% DOWN SAMPLING
gridStep = 0.03;
% ptc_ver = pointCloud(vertices);
ptCloudA = pcdownsample(ptc_ver,'gridAverage',gridStep);
figure
pcshow(ptCloudA);

%% RANSAC
% R = 100;
% T = 0.08;
% K = 0.4;
new_ver = ptCloudA.Location;
% new_ver = ptCloudB.Location;
data_size = size(new_ver);
data_num = data_size(1);
[a_,b_,c_,d_] = ransac_func(new_ver, R, T, fix(data_num*K));
X_max = max(new_ver(:,1),[],'all');
X_min = min(new_ver(:,1),[],'all');
Y_max = max(new_ver(:,2),[],'all');
Y_min = min(new_ver(:,2),[],'all');
[X_pre, Y_pre] = meshgrid(X_min:(X_max-X_min)/8:X_max,Y_min:(Y_max-Y_min)/8:Y_max);
Z_pre = -a_*X_pre/c_ - b_*Y_pre/c_ - d_/c_;
hold on;
mesh2 = mesh(X_pre,Y_pre,Z_pre,"AlphaData",0.5,"EdgeColor","g","FaceColor","none");
%% DOWN SAMPLING2
stepSize = floor(ptc_ver.Count/ptCloudA.Count);
indices = 1:stepSize:ptc_ver.Count;
ptCloudB = select(ptc_ver,indices);
figure
pcshow(ptCloudB);

%% RANSAC2
% new_ver = ptCloudA.Location;
new_ver = ptCloudB.Location;
data_size = size(new_ver);
data_num = data_size(1);
[a_2,b_2,c_2,d_2] = ransac_func(new_ver, R, T, fix(data_num*K));
X_max = max(new_ver(:,1),[],'all');
X_min = min(new_ver(:,1),[],'all');
Y_max = max(new_ver(:,2),[],'all');
Y_min = min(new_ver(:,2),[],'all');
[X_pre, Y_pre] = meshgrid(X_min:(X_max-X_min)/8:X_max,Y_min:(Y_max-Y_min)/8:Y_max);
Z_pre = -a_2*X_pre/c_2 - b_2*Y_pre/c_2 - d_2/c_2;
hold on;
mesh3 = mesh(X_pre,Y_pre,Z_pre,"AlphaData",0.5,"EdgeColor","g","FaceColor","none");



%% RANSAC
% [a_,b_,c_,d_] = ransac_func(vertices);
% X_max = max(vertices(:,1),[],'all');
% Y_max = max(vertices(:,2),[],'all');
% [X_pre, Y_pre] = meshgrid(0:X_max/8:X_max,0:Y_max/8:Y_max);
% Z_pre = -a_*X_pre/c_ - b_*Y_pre/c_ - d_/c_;
% mesh(X_pre,Y_pre,Z_pre,"AlphaData",0.5,"EdgeColor","g","FaceColor","none");