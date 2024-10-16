depth = 8;
mesh2 = pc2surfacemesh(pointCloud([-1*ptc_datas(:,1),ptc_datas(:,2),-1*ptc_datas(:,3)]),"poisson",depth);
datas = mesh2.Vertices;
% surfaceMeshShow(mesh2,"WireFrame",true)
% writeSurfaceMesh(mesh2,"test_surface");
figure
pcshow(datas); hold on;

%% pointcloud interpolation
% xlist = -1:0.05:1;
% ylist = 3:0.05:12;
xlist = -1:0.05:1;
ylist = 1.5:0.05:6.5;
[xq,yq] = meshgrid(xlist,ylist);
elevation_mesh = griddata(double(datas(:,1)),double(datas(:,2)),double(datas(:,3)),xq,yq,"natural");
figure;
mesh(xq,yq,elevation_mesh);
axis equal
% colormap default

%% surface gradient
[fx,fy] = gradient(elevation_mesh,0.05);
figure;

z_vec = [0,0,1];                        % z vector
angle_r = subspace(z_vec',u_vec');          % [rad] angle
gradient_mesh = mesh(-xq,yq,(abs(fy)+abs(fx)));
axis equal
% colormap(autumn(5))
% mesh3 = pc2surfacemesh(pointCloud([reshape(xq,[numel(xq),1]),reshape(yq,[numel(yq),1]),-1*reshape((abs(fy)+abs(fx)),[numel((abs(fy)+abs(fx))),1])]),"poisson",depth);
% surfaceMeshShow(mesh3);

%% draw
figure;
tiledlayout('flow')
ax1 = nexttile;
pcshow(mesh2.Vertices);
%colormap(ax1,winter)
ax2 = nexttile;
mesh(xq,yq,elevation_mesh);
axis equal
colormap(ax2,autumn(5))
nexttile([2,2]);
gradient_mesh = mesh(xq,yq,fy);
axis equal