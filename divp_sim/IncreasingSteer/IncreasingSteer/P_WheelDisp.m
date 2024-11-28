wfl_z = -out.Body_inert.FrntAxl.Lft.Disp.Z.Data;
wfr_z = -out.Body_inert.FrntAxl.Rght.Disp.Z.Data;
wrl_z = -out.Body_inert.RearAxl.Lft.Disp.Z.Data;
wrr_z = -out.Body_inert.RearAxl.Rght.Disp.Z.Data;

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
plot(time,[gfl_z],'Color','red','DisplayName','Ground FL'); hold on; grid on;
plot(time,[gfr_z],'Color','red','DisplayName','Ground FR','LineStyle','--');
plot(time,[grl_z],'Color','blue','DisplayName','Ground RL');
plot(time,[grr_z],'Color','blue','DisplayName','Ground RR','LineStyle','--'); legend;
xlabel('Time [s]'); ylabel('Z [m]');
set(gca,'FontName','Arial','FontSize',11);