x_max = 630;
x_min = 623;

%% mesh
depth = 9;
mesh2 = pc2surfacemesh(pointCloud([pcdmap(pcdmap(:,1)<x_max&pcdmap(:,1)>x_min,1),pcdmap(pcdmap(:,1)<x_max&pcdmap(:,1)>x_min,2),pcdmap(pcdmap(:,1)<x_max&pcdmap(:,1)>x_min,3)]),"poisson",depth);
datas = mesh2.Vertices;
% surfaceMeshShow(mesh2,"WireFrame",true)
% writeSurfaceMesh(mesh2,"test_surface");
figure
pcshow(datas); hold on;

%% grid
xlist = x_min:0.01:x_max;
ylist = -13:0.01:-9;
[xq,yq] = meshgrid(xlist,ylist);
elevation_mesh = griddata(double(datas(:,1)),double(datas(:,2)),double(datas(:,3)),xq,yq,"natural");
figure;
% mesh(xq,yq,elevation_mesh);
scatter(reshape(xq,[height(xq)*width(xq),1]),reshape(yq,[height(yq)*width(yq),1]),3,reshape(elevation_mesh,[height(elevation_mesh)*width(elevation_mesh),1]),'filled');
colormap("turbo");
axis equal
hold on;
% plot3(T_ref(:,1),T_ref(:,2),T_ref(:,3),'Color','red','LineWidth',1.5,'DisplayName','Ref. Path');
% plot3(T_ref(:,1),T_ref(:,2)-1.485/2,T_ref(:,3),'Color','black','LineWidth',1.5,'DisplayName','Ref. Path');
% plot3(T_ref(:,1),T_ref(:,2)+1.485/2,T_ref(:,3),'Color','black','LineWidth',1.5,'DisplayName','Ref. Path'); hold on;
org_p = plot(T_ref(:,1),T_ref(:,2),'Color','red','LineWidth',1.5,'DisplayName','Ref. Path');
whl_p = plot(T_ref(:,1),T_ref(:,2)-1.485/2,'Color','black','LineWidth',1.5,'DisplayName','Ref. Path');
plot(T_ref(:,1),T_ref(:,2)+1.485/2,'Color','black','LineWidth',1.5,'DisplayName','Ref. Path'); hold on;
xlim([x_min, x_max]); ylim([-13 -9]);
xlabel('\itX \rm[m]'); ylabel('\itY \rm[m]');
set(gca,'FontName','Times New Roman','FontSize',9);
xlim([x_min x_max])

%% path generation
start_x = 623;
end_x = 628.5;
[k,~] = dsearchn(T_ref(:,1),[start_x; end_x]);
start_point = T_ref(k(1),1:2);
end_point = T_ref(k(2),1:2);
waypoints = [[start_point,0]; [(start_point(1)+end_point(1))/2,start_point(2)-0.4,0]; [end_point,0]];
refPath = referencePathFrenet(waypoints);
connector = trajectoryGeneratorFrenet(refPath);
% show(refPath);
hold on;
initState = [0 0 0 0 0 0];  % [S ds ddS L dL ddL]
termState = [6 0 0 0 0 0]; % [S ds ddS L dL ddL]
[~,trajGlobal] = connect(connector,initState,termState,length(T_ref(T_ref(:,1)>start_x&T_ref(:,1)<end_x))/10-0.1);
% plot(trajGlobal.Trajectory(:,1),trajGlobal.Trajectory(:,2),'b')
new_path = T_ref;
new_path(new_path(:,1)>start_x&new_path(:,1)<end_x,1:2) = trajGlobal.Trajectory(:,1:2);
% plot3(new_path(:,1),new_path(:,2),new_path(:,3),'Color','red','LineWidth',1.5,'LineStyle','--','DisplayName','Ref. Path');
% plot3(new_path(:,1),new_path(:,2)-1.485/2,new_path(:,3),'Color','black','LineWidth',1.5,'LineStyle','--','DisplayName','Ref. Path');
% plot3(new_path(:,1),new_path(:,2)+1.485/2,new_path(:,3),'Color','black','LineWidth',1.5,'LineStyle','--','DisplayName','Ref. Path'); hold on;
avo_p = plot(new_path(:,1),new_path(:,2),'Color','red','LineWidth',1.5,'LineStyle','--','DisplayName','Ref. Path');
whl_avo_p = plot(new_path(:,1),new_path(:,2)-1.485/2,'Color','black','LineWidth',1.5,'LineStyle','--','DisplayName','Ref. Path');
plot(new_path(:,1),new_path(:,2)+1.485/2,'Color','black','LineWidth',1.5,'LineStyle','--','DisplayName','Ref. Path'); hold on;
axis equal
xlim([x_min, x_max]); ylim([-13 -9]);
xlabel('\itX \rm[m]'); ylabel('\itY \rm[m]');
set(gca,'FontName','Times New Roman','FontSize',9);
legend([org_p,whl_p,avo_p,whl_avo_p],{'Original Path','Original Wheel Track','Avoidance Path','Avoidance Wheel Track'});

%% gradient
[fx,fy] = gradient(elevation_mesh,0.01);
figure;
% z_vec = [0,0,1];                        % z vector
gradient_data = (fy.^2+fx.^2);
% gradient_mesh = mesh(xq,yq,(fy.^2+fx.^2));
scatter(reshape(xq,[height(xq)*width(xq),1]),reshape(yq,[height(yq)*width(yq),1]),3,reshape(gradient_data,[height(gradient_data)*width(gradient_data),1]),'filled');
colormap("turbo");
axis equal
% zlim([0 1])
% clim([0 0.3])
xlim([x_min, x_max]); ylim([-13 -9]);
xlabel('\itY \rm[m]'); ylabel('\itX \rm[m]');
set(gca,'FontName','Times New Roman','FontSize',9);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_max = 686;
x_min = 679;

%% mesh
depth = 9;
mesh2 = pc2surfacemesh(pointCloud([pcdmap(pcdmap(:,1)<x_max&pcdmap(:,1)>x_min,1),pcdmap(pcdmap(:,1)<x_max&pcdmap(:,1)>x_min,2),pcdmap(pcdmap(:,1)<x_max&pcdmap(:,1)>x_min,3)]),"poisson",depth);
datas = mesh2.Vertices;
% surfaceMeshShow(mesh2,"WireFrame",true)
% writeSurfaceMesh(mesh2,"test_surface");
figure
pcshow(datas); hold on;

%% grid
xlist = x_min:0.01:x_max;
ylist = -13:0.01:-9;
[xq,yq] = meshgrid(xlist,ylist);
elevation_mesh = griddata(double(datas(:,1)),double(datas(:,2)),double(datas(:,3)),xq,yq,"natural");
figure;
% mesh(xq,yq,elevation_mesh);
scatter(reshape(xq,[height(xq)*width(xq),1]),reshape(yq,[height(yq)*width(yq),1]),3,reshape(elevation_mesh,[height(elevation_mesh)*width(elevation_mesh),1]),'filled');
colormap("turbo");
axis equal
hold on;
% plot3(T_ref(:,1),T_ref(:,2),T_ref(:,3),'Color','red','LineWidth',1.5,'DisplayName','Ref. Path');
% plot3(T_ref(:,1),T_ref(:,2)-1.485/2,T_ref(:,3),'Color','black','LineWidth',1.5,'DisplayName','Ref. Path');
% plot3(T_ref(:,1),T_ref(:,2)+1.485/2,T_ref(:,3),'Color','black','LineWidth',1.5,'DisplayName','Ref. Path'); hold on;
org_p = plot(T_ref(:,1),T_ref(:,2),'Color','red','LineWidth',1.5,'DisplayName','Ref. Path');
whl_p = plot(T_ref(:,1),T_ref(:,2)-1.485/2,'Color','black','LineWidth',1.5,'DisplayName','Ref. Path');
plot(T_ref(:,1),T_ref(:,2)+1.485/2,'Color','black','LineWidth',1.5,'DisplayName','Ref. Path'); hold on;
xlim([x_min, x_max]); ylim([-13 -9]);
xlabel('\itY \rm[m]'); ylabel('\itX \rm[m]');
set(gca,'FontName','Times New Roman','FontSize',9);

%% path generation
start_x = 680;
end_x = 685;
[k,~] = dsearchn(T_ref(:,1),[start_x; end_x]);
start_point = T_ref(k(1),1:2);
end_point = T_ref(k(2),1:2);
waypoints = [[start_point,0]; [(start_point(1)+end_point(1))/2,start_point(2)-0.4,0]; [end_point,0]];
refPath = referencePathFrenet(waypoints);
connector = trajectoryGeneratorFrenet(refPath);
% show(refPath);
hold on;
initState = [0 0 0 0 0 0];  % [S ds ddS L dL ddL]
termState = [6 0 0 0 0 0]; % [S ds ddS L dL ddL]
[~,trajGlobal] = connect(connector,initState,termState,length(T_ref(T_ref(:,1)>start_x&T_ref(:,1)<end_x))/10-0.1);
% plot(trajGlobal.Trajectory(:,1),trajGlobal.Trajectory(:,2),'b')
% new_path = T_ref;
new_path(new_path(:,1)>start_x&new_path(:,1)<end_x,1:2) = trajGlobal.Trajectory(:,1:2);
% plot3(new_path(:,1),new_path(:,2),new_path(:,3),'Color','red','LineWidth',1.5,'LineStyle','--','DisplayName','Ref. Path');
% plot3(new_path(:,1),new_path(:,2)-1.485/2,new_path(:,3),'Color','black','LineWidth',1.5,'LineStyle','--','DisplayName','Ref. Path');
% plot3(new_path(:,1),new_path(:,2)+1.485/2,new_path(:,3),'Color','black','LineWidth',1.5,'LineStyle','--','DisplayName','Ref. Path'); hold on;
avo_p = plot(new_path(:,1),new_path(:,2),'Color','red','LineWidth',1.5,'LineStyle','--','DisplayName','Ref. Path');
whl_avo_p = plot(new_path(:,1),new_path(:,2)-1.485/2,'Color','black','LineWidth',1.5,'LineStyle','--','DisplayName','Ref. Path');
plot(new_path(:,1),new_path(:,2)+1.485/2,'Color','black','LineWidth',1.5,'LineStyle','--','DisplayName','Ref. Path'); hold on;
axis equal
xlim([x_min, x_max]); ylim([-13 -9]);
xlabel('\itY \rm[m]'); ylabel('\itX \rm[m]');
set(gca,'FontName','Times New Roman','FontSize',9);
legend([org_p,whl_p,avo_p,whl_avo_p],{'Original Path','Original Wheel Track','Avoidance Path','Avoidance Wheel Track'});

%% gradient
[fx,fy] = gradient(elevation_mesh,0.01);
figure;
% z_vec = [0,0,1];                        % z vector
gradient_data = (fy.^2+fx.^2);
% gradient_mesh = mesh(xq,yq,(fy.^2+fx.^2));
scatter(reshape(xq,[height(xq)*width(xq),1]),reshape(yq,[height(yq)*width(yq),1]),3,reshape(gradient_data,[height(gradient_data)*width(gradient_data),1]),'filled');
colormap("turbo");
axis equal
% zlim([0 1])
% clim([0 0.3])
xlim([x_min, x_max]); ylim([-13 -9]);
xlabel('\itY \rm[m]'); ylabel('\itX \rm[m]');
set(gca,'FontName','Times New Roman','FontSize',9);