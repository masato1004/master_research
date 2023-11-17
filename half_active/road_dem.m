depth = 6;
mesh2 = pc2surfacemesh(pointCloud(ptc_datas),"poisson",depth);
% surfaceMeshShow(mesh2)
% writeSurfaceMesh(mesh2,"test_surface");
figure
pcshow(mesh2.Vertices); hold on;

%% pointcloud interpolation
xlist = -1:0.1:1;
ylist = 3:0.1:12;
[xq,yq] = meshgrid(xlist,ylist);
elevation_mesh = griddata(datas(:,1),datas(:,2),datas(:,3),xq,yq,"cubic");
figure;
surf(xq,yq,elevation_mesh);
axis equal
% colormap default

%% surface gradient
[fx,fy] = gradient(elevation_mesh,0.1);
figure;

z_vec = [0,0,1];                        % z vector
angle_r = subspace(z_vec',u_vec');          % [rad] angle
gradient_mesh = surf(xq,yq,abs(fy)+abs(fx));
axis equal
% colormap(autumn(5))

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