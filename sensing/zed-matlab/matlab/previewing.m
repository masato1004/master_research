%% REQUIREMENTS
% ransac_func.m in the same directory
% variable "vertices" which included the 3 dimensional points data of a road

%% LOAD POINT CLOUD
% load("");  % when it's needed
close all;
clc;

%% CHECK RAW POINT CLOUD DATA
fig_raw = figure('Name',"raw data", 'Position', [300 400 800 480]);
pcshow(pointCloud(vertices),'BackgroundColor', 'white');
axis on;
xlabel("{\it X} [m]");
ylabel("{\it Y} [m]");
zlabel("{\it Z} [m]");
fontname(fig_raw,"Times New Roman");
fontsize(fig_raw,10.5,"points");

%% PICK UP ROAD POINT CLOUD
% road_data = vertices(vertices(:,1)>=-1.2 & vertices(:,1)<=1.2 & vertices(:,2)<=15 & vertices(:,2)>=5 & vertices(:,3)<=3,:,:);
road_data = vertices(vertices(:,1)>=-1.2 & vertices(:,1)<=1.2 & vertices(:,3)<=3,:,:);
raod_cloud = pointCloud(road_data);
fig_raod = figure('Name',"down sampling", 'Position', [350 370 800 480]);
pcshow(raod_cloud,'BackgroundColor', 'white');
axis on;
xlabel("{\it X} [m]");
ylabel("{\it Y} [m]");
zlabel("{\it Z} [m]");
fontname(fig_raod,"Times New Roman");
fontsize(fig_raod,10.5,"points");

%% DOWN SAMPLING
gridStep = 0.05;
ptCloudA = pcdownsample(raod_cloud,'gridAverage',gridStep);
fig_ds = figure('Name',"down sampling", 'Position', [400 310 800 480]);
pcshow(ptCloudA,'BackgroundColor', 'white');
grid on;
axis on;
xlabel("{\it X} [m]");
ylabel("{\it Y} [m]");
zlabel("{\it Z} [m]");
fontname(fig_ds,"Times New Roman");
fontsize(fig_ds,10.5,"points");

%% RANSAC
% R = 400;   % number of maximum loop
% T = 0.02;  % threshold of a difference between predicted plane
% K = 0.3;   % lower bound of number of points ratio for regarding as a plane
% new_ver = ptCloudA.Location;
% data_size = size(new_ver);
% data_num = data_size(1);
% [a_,b_,c_,d_] = ransac_func(new_ver, R, T, fix(data_num*K));
% X_max = max(new_ver(:,1),[],'all');
% X_min = min(new_ver(:,1),[],'all');
% Y_max = max(new_ver(:,2),[],'all');
% Y_min = min(new_ver(:,2),[],'all');
% [X_pre, Y_pre] = meshgrid(X_min:(X_max-X_min)/8:X_max,Y_min:(Y_max-Y_min)/8:Y_max);
% Z_pre = -a_*X_pre/c_ - b_*Y_pre/c_ - d_/c_;
% hold on;
% mesh2 = mesh(X_pre,Y_pre,Z_pre,"AlphaData",0.5,"EdgeColor","g","FaceColor","none");

%% PCFITPLANE
max_distance = 0.02;  % [m]
model = pcfitplane(raod_cloud,max_distance);
X_max = max(road_data(:,1),[],'all');
X_min = min(road_data(:,1),[],'all');
Y_max = max(road_data(:,2),[],'all');
Y_min = min(road_data(:,2),[],'all');
[X_pre, Y_pre] = meshgrid(X_min:(X_max-X_min)/8:X_max,Y_min:(Y_max-Y_min)/8:Y_max);
Z_pre = -model.Parameters(1)*X_pre/model.Parameters(3) - model.Parameters(2)*Y_pre/model.Parameters(3) - model.Parameters(4)/model.Parameters(3);
hold on;
mesh2 = mesh(X_pre,Y_pre,Z_pre,"AlphaData",0.5,"EdgeColor","r","FaceColor","none");

%% ROTATION FROM SURFACE
% Find a normal vector of RANSAC-plane
[nx,ny,nz]=surfnorm(X_pre,Y_pre,Z_pre); % find normal vector
s_vec=reshape([nx,ny,nz],9,9,3);        % combine normal vector
u = reshape(s_vec(5,5,:),1,3);          % pick-up one vector

% Find a rotation matrix
z_vec = [0,0,1];                        % z vector
angle_r = subspace(z_vec',u');          % [rad] angle
angle_d = rad2deg(angle_r);             % [deg] angle
r_axis = cross(z_vec,u);                % find a axis of rotation

% Rotate the point cloud
new_mesh = reshape([X_pre,Y_pre,Z_pre],9,9,3);
new_mesh = reshape(new_mesh,81,3);
new_mesh = pointCloud(new_mesh);

rotationVector = -angle_r * r_axis/norm(r_axis);
rotationMatrix = rotationVectorToMatrix(rotationVector);
translation = [0 0 0];
tform = rigid3d(rotationMatrix,translation);
ptCloudOut = pctransform(raod_cloud,tform);
new_mesh_out = pctransform(new_mesh,tform);

% Lift the road up for camera-height by mesh
datas = ptCloudOut.Location;
datas = datas(datas(:,3)<=-1.3,:);
datas(:,3) = datas(:,3) - mean(new_mesh_out.Location(:,3));
fig_rot = figure('Name',"rotated", 'Position', [450 280 800 480]);
pcshow(pointCloud(datas),'BackgroundColor', 'white');
axis on;
xlabel("{\it X} [m]");
ylabel("{\it Y} [m]");
zlabel("{\it Z} [m]");
fontname(fig_rot,"Times New Roman");
fontsize(fig_rot,10.5,"points");

%% ROTATION FROM SETTING ANGLE
% Setting angle
set_ang = cam_angle;                           % [deg] angle

rotationVector = -deg2rad(set_ang) * [1 0 0];
rotationMatrix = rotationVectorToMatrix(rotationVector);
translation = [0 0 0];
tform = rigid3d(rotationMatrix,translation);
ptCloudOut = pctransform(raod_cloud,tform);
new_mesh_out = pctransform(new_mesh,tform);

% Lift the road up for camera-height by mesh
datas = ptCloudOut.Location;
datas = datas(datas(:,3)<=-1.3,:);
datas(:,3) = datas(:,3) + cam_height;
fig_rot = figure('Name',"rotated", 'Position', [450 280 800 480]);
pcshow(pointCloud(datas),'BackgroundColor', 'white');
axis on;
xlabel("{\it X} [m]");
ylabel("{\it Y} [m]");
zlabel("{\it Z} [m]");
fontname(fig_rot,"Times New Roman");
fontsize(fig_rot,10.5,"points");

%% PICK UP AS 2D
range_min = 0;         % minimum measurable distance [m]
range_max = 15;        % maximum measurable distance [m]
pick_up_width = 0.03;  % width of datas for a road profile [m]
pick_up_center = 0.2;   % center of pick up position [m]

p_min = pick_up_center - pick_up_width/2;
p_max = pick_up_center + pick_up_width/2;

line = datas(datas(:,1)>=p_min & datas(:,1)<=p_max & datas(:,2)<=range_max & datas(:,2)>=range_min,:,:);
[~,ind] = sort(line(:,2));
prev_profile=line(ind,2:3);

fig_rprofile = figure('name', "Road Displacement from ZED",'Position', [500 250 1400 380]);
plot(prev_profile(:,1),prev_profile(:,2),"LineWidth",1,"Color","#0000ff");
xlabel("Ground Distance from Sensor [m]");
ylabel("Road Displacement [m]");
fontname(fig_rprofile,"Times New Roman");
fontsize(fig_rprofile,10.5,"points");
grid on;