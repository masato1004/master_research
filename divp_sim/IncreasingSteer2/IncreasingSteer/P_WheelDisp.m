wfl_z = out.wheel_z.Data(:,1);
wfr_z = out.wheel_z.Data(:,2);
wrl_z = out.wheel_z.Data(:,3);
wrr_z = out.wheel_z.Data(:,4);

wfl_x = out.Body_inert.FrntAxl.Lft.Disp.X.Data;
wfr_x = out.Body_inert.FrntAxl.Rght.Disp.X.Data;
wrl_x = out.Body_inert.RearAxl.Lft.Disp.X.Data;
wrr_x = out.Body_inert.RearAxl.Rght.Disp.X.Data;

wfl_y = out.Body_inert.FrntAxl.Lft.Disp.Y.Data;
wfr_y = out.Body_inert.FrntAxl.Rght.Disp.Y.Data;
wrl_y = out.Body_inert.RearAxl.Lft.Disp.Y.Data;
wrr_y = out.Body_inert.RearAxl.Rght.Disp.Y.Data;

gfl_z = out.Road_Profile.G_FL_z.Data;
gfr_z = out.Road_Profile.G_FR_z.Data;
grl_z = out.Road_Profile.G_RL_z.Data;
grr_z = out.Road_Profile.G_RR_z.Data;

time = out.tout;

figure;
plot(time,[wfl_z],'Color','red','DisplayName','Wheel FL'); hold on; grid on;
plot(time,[wfr_z],'Color','red','DisplayName','Wheel FR','LineStyle','--');
plot(time,[wrl_z],'Color','blue','DisplayName','Wheel RL');
plot(time,[wrr_z],'Color','blue','DisplayName','Wheel RR','LineStyle','--'); legend;
xlabel('Time [s]'); ylabel('Z [m]');
set(gca,'FontName','Arial','FontSize',11);


figure;
plot(wfl_x,[gfl_z],'Color','red','DisplayName','Ground FL'); hold on; grid on;
plot(wfr_x,[gfr_z],'Color','red','DisplayName','Ground FR','LineStyle','--');
plot(wrl_x,[grl_z],'Color','blue','DisplayName','Ground RL');
plot(wrr_x,[grr_z],'Color','blue','DisplayName','Ground RR','LineStyle','--'); legend;
xlabel('Time [s]'); ylabel('Z [m]');
set(gca,'FontName','Arial','FontSize',11);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
plot(wfl_x,[wfl_z],'Color','red','DisplayName','Wheel FL'); hold on; grid on;
plot(wfr_x,[wfr_z],'Color','red','DisplayName','Wheel FR','LineStyle','--');
plot(wrl_x,[wrl_z],'Color','blue','DisplayName','Wheel RL');
plot(wrr_x,[wrr_z],'Color','blue','DisplayName','Wheel RR','LineStyle','--'); legend;
xlabel('X [m]'); ylabel('Z [m]');
set(gca,'FontName','Arial','FontSize',11);


figure;
plot(sqrt(wfl_x.^2+wfl_y.^2),[gfl_z],'Color','red','DisplayName','Ground FL'); hold on; grid on;
plot(sqrt(wfr_x.^2+wfr_y.^2),[gfr_z],'Color','red','DisplayName','Ground FR','LineStyle','--');
plot(sqrt(wrl_x.^2+wrl_y.^2),[grl_z],'Color','blue','DisplayName','Ground RL');
plot(sqrt(wrr_x.^2+wrr_y.^2),[grr_z],'Color','blue','DisplayName','Ground RR','LineStyle','--'); legend;
xlabel('Distance [m]'); ylabel('Z [m]');
set(gca,'FontName','Arial','FontSize',11);