%% Check only road
datas=ptCloudOut.Location;
road_data = datas(datas(:,1)>=-1 & datas(:,1)<=1 & datas(:,2)<=15 & datas(:,2)>=0,:,:);
% road_data = datas(datas(:,1)>=-1 & datas(:,1)<=1 & datas(:,2)<=9 & datas(:,2)>=5,:,:);
% road_data = datas(datas(:,1)>=-1 & datas(:,1)<=1 & datas(:,3)<=-1.3 & datas(:,3)>=-1.6,:,:);
road_ptc = figure('name',"Road Point Cloud");
pcshow(pointCloud(road_data));

%% Pick up a road profile
% datas=ptCloudOut.Location;
datas = road_data;
% lo = datas(:,1)<=0.01 & datas(:,1)>=-0.01;
lo = datas(:,1)<=0.11 & datas(:,1)>=0.1;
% lo = datas(:,1)>=-0.04  & datas(:,1)<=-0.03;
% datas(lo,:);
line = datas(lo,:);
[~,ind] = sort(line(:,2));
line_neo=line(ind,:);
fig = figure('name', "Road Displacement from ZED",'Position', [500-20 500-20 1400 380]);
% plot(line_neo(:,2),line_neo(:,3)-line_neo(1,3),"LineWidth",1,"Color","#0000ff");

LT = trenddecomp(line_neo(:,3));
plot(line_neo(:,2),line_neo(:,3)+1.450,"LineWidth",1,"Color","#0000ff");
% hold on;
% scatter(line_neo(:,2),line_neo(:,3)+1.450)
% hold on;
% plot(line_neo(:,2),LT+1.450,"LineWidth",1,"Color","#ff0000");
% hold off;
% legend("Law data","Long-term Trend")
ylabel("Displacement [m]");
xlabel("Distance [m]");
xlim([0,max(line_neo(:,2))]);
ylim([-0.1,0.1]);
% axis equal;
fontname(fig,"Times New Roman");
fontsize(fig,16,"points");
grid on;

%% additional road
% aditional = line_neo(line_neo(:,2)>=6 & line_neo(:,2)<=8,:,:);
% aditional(:,2) = aditional(:,2) - 3;
% aditional(:,3) = aditional(:,3) - aditional(end,3)+line_neo(1,3);
% line_neo2 = [aditional; line_neo];
% fig2 = figure('name', "Road Displacement from ZED",'Position', [500-20 500-20 1400 380]);
% plot(line_neo2(:,2),line_neo2(:,3)-line_neo(1,3),"LineWidth",1,"Color","#0000ff");
% % plot(line_neo2(:,2),line_neo2(:,3)+1.460,"LineWidth",1,"Color","#0000ff");
% xlabel("Distance");
% ylabel("displacement");
% ylabel("Displacement [m]");
% xlabel("Distance [m]");
% xlim([0,max(line_neo(:,2))]);
% ylim([-0.1,0.1]);
% % axis equal;
% fontname(fig2,"Times New Roman");
% fontsize(fig2,16,"points");
% grid on;

% aditional2 = [zeros(size(-3:0.001:1.45));
%     -3:0.001:1.45;
%     zeros(size(-3:0.001:1.45))]';
% aditional2(:,3) = aditional2(:,3)-1.450;
% line_neo3 = [aditional2;line_neo];
% fig3 = figure('name', "Road Displacement from ZED",'Position', [500-20 500-20 1400 380]);
% % plot(line_neo3(:,2),line_neo3(:,3)-line_neo(1,3),"LineWidth",1,"Color","#0000ff");
% plot(line_neo3(:,2),line_neo3(:,3)+1.450,"LineWidth",1,"Color","#0000ff");
% xlabel("Distance");
% ylabel("displacement");
% ylabel("Displacement [m]");
% xlabel("Distance [m]");
% xlim([0,max(line_neo3(:,2))]);
% ylim([-0.1,0.1]);
% % axis equal;
% fontname(fig3,"Times New Roman");
% fontsize(fig3,16,"points");
% grid on;

% aditional3 = [zeros(size(line_neo3(end,2)+0.1:0.001:300));
% line_neo3(end,2)+0.1:0.001:300;
% ones(size(line_neo3(end,2)+0.1:0.001:300))*line_neo3(end,3)]';
% aditional3(:,3) = aditional3(:,3);
% line_neo4 = [line_neo3;aditional3];
% line_neo4(:,3) = line_neo4(:,3)+1.45;
% fig4 = figure('name', "Road Displacement from ZED",'Position', [500-20 500-20 1400 380]);
% % plot(line_neo3(:,2),line_neo3(:,3)-line_neo(1,3),"LineWidth",1,"Color","#0000ff");
% plot(line_neo4(:,2),line_neo4(:,3),"LineWidth",1,"Color","#0000ff");
% xlabel("Distance");
% ylabel("displacement");
% ylabel("Displacement [m]");
% xlabel("Distance [m]");
% xlim([-3,max(line_neo4(:,2))]);
% ylim([-0.1,0.1]);
% % axis equal;
% fontname(fig4,"Times New Roman");
% fontsize(fig4,16,"points");
% grid on;
% save("line_neo4_as_truedata.mat")

%% detrend
% D = detrend((line_neo(:,3)+1.450)',2);
% hold on;
% % fig2 = figure('name', "Detrended Road Displacement from ZED",'Position', [500-20 500-20 1400 380]);
% plot(line_neo(:,2),D',"LineWidth",1,"Color","#ff0000");
% plot(line_neo(:,2),line_neo(:,3)+1.450-D',":k","LineWidth",1,"Color","#ff0000");
% xlabel("Distance");
% ylabel("displacement");
% ylabel("Displacement [m]");
% xlabel("Distance [m]");
% xlim([0,max(line_neo(:,2))]);
% ylim([-0.1,0.1]);
% % axis equal;
% % fontname(fig2,"Times New Roman");
% % fontsize(fig2,16,"points");
% grid on;
% legend("Raw Data", "Detrended", "Trend");