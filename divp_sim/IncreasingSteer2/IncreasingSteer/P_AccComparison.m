%% NMPC w/o U_obs data store
time = out.Body_bdy.Cg.Acc.az.Time;
az_com = -out.Body_bdy.Cg.Acc.az.Data;
ay_com = -out.Body_bdy.Cg.Acc.ay.Data;
input_time = rad2deg(out.ControllerOut.steer_cmd.Time);
input_com = rad2deg(out.ControllerOut.steer_cmd.Data); % deg

%% NMPC proposal data store
time = out.Body_bdy.Cg.Acc.az.Time;
az_pro = -out.Body_bdy.Cg.Acc.az.Data;
ay_pro = -out.Body_bdy.Cg.Acc.ay.Data;
input_time = rad2deg(out.ControllerOut.steer_cmd.Time);
input_pro = rad2deg(out.ControllerOut.steer_cmd.Data); % deg

%% az
figure('Position',[100 100 330 180])
az_com_p = plot(time,az_com,"Color",'b','LineStyle','-','LineWidth',2); hold on; grid on;
az_pro_p = plot(time,az_pro,"Color",'r','LineStyle','-','LineWidth',2);
legend([az_com_p,az_pro_p],{'NMPC w/o \itU_{obs}','Proposed'})
xlim([1 5]);
ylim([-0.45 0.45])
xlabel("Time \rm[s]");
ylabel("\ita_{z} \rm[m]");
fontname(gcf,"Times New Roman");
fontsize(gca,11,"points");

%% ay
figure('Position',[100 100 330 180])
ay_com_p = plot(time,ay_com,"Color",'b','LineStyle','-','LineWidth',2); hold on; grid on;
ay_pro_p = plot(time,ay_pro,"Color",'r','LineStyle','-','LineWidth',2);
legend([ay_com_p,ay_pro_p],{'NMPC w/o \itU_{obs}','Proposed'})
xlim([1 5]);
ylim([-0.25 0.25])
xlabel("Time \rm[s]");
ylabel("\ita_{y} \rm[m]");
fontname(gcf,"Times New Roman");
fontsize(gca,11,"points");

% const_p = yline([-2 2],'--k','LineWidth',2,'DisplayName','Constraint');
% legend([ay_com_p,ay_pro_p,const_p(1)],{'NMPC w/o \itU_{obs}','Proposed','Constraint'})
% ylim([-2.5 2.5])

%% steer cmd
plot(out.ControllerOut.steer_cmd.Time,rad2deg(out.ControllerOut.steer_cmd.Data))