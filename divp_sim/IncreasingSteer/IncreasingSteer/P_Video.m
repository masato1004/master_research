Body_inert = out.Body_inert;
Driveline = out.Driveline;

positions = [
    out.Body_inert.Geom.Disp.X.Data';
    out.Body_inert.Geom.Disp.Y.Data';
    out.Body_inert.Geom.Disp.Z.Data'
    ];

velocitys = [
    out.Body_inert.Cg.Vel.Xdot.Data';
    out.Body_inert.Cg.Vel.Ydot.Data';
    out.Body_inert.Cg.Vel.Zdot.Data'
    ];

angles = [
    out.Body_inert.Cg.Ang.phi.Data';
    out.Body_inert.Cg.Ang.theta.Data';
    out.Body_inert.Cg.Ang.psi.Data'
    ];

wheel_angles = out.Driveline.WhlAng.Data';

%% reference trajectory
filename = "scenario_1_divp_Veh_NissanXtrail_1.csv";
opts = detectImportOptions(filename);
opts.SelectedVariableNames = ["timestamp", "pos_x","pos_y","pos_z", "yaw_rad", "pitch_rad", "roll_rad"];
pos_table = readtable(filename,opts);
initstate = [pos_table.pos_x(1) pos_table.pos_y(1) pos_table.pos_z(1) pos_table.yaw_rad(1) pos_table.pitch_rad(1) pos_table.roll_rad(1)];
initpos = initstate(1:3);
initagl = [initstate(4) 0 0];
pcdpos = pointCloud([pos_table.pos_x,pos_table.pos_y,pos_table.pos_z]);
R1 = eul2rotm(-initagl);
A1 = [[R1;0,0,0],[0; 0; 0;1]];
pcdpos = pctransform(pointCloud([pcdpos.Location(:,1)-initpos(1),pcdpos.Location(:,2)-initpos(2),pcdpos.Location(:,3)-initpos(3)]),rigidtform3d(A1));
T_ref = pcdpos.Location;

fig_xy = figure('Position',[100 100 500 450]);
videoname = 'test_tracking';
video = VideoWriter(videoname,'MPEG-4');
video.FrameRate = 10;
open(video);

for i = 1:100:length(positions)
    pos = positions(:,i);
    ang = angles(:,i);
    vel = velocitys(:,i);
    wheel_ang = wheel_angles(:,i);
    time = out.tout(i);
    XYplot(time,pos,ang,vel,wheel_ang,T_ref)

    frame = getframe(gcf);
    writeVideo(video,frame);
end
close(video)
ylim([min(positions(1,:))-2,max(positions(1,:))+2]);
xlim([min(positions(2,:))-2,max(positions(2,:))+2]);

function XYplot(time,pos,agl,vel,wheel_agl,T_ref)
persistent track_w a b fig_xy diameter ref_p
if time<0.000001
    track_w = 1.485;
    a = 1.43;
    b = 1.43;
    diameter = 0.653;
end
if time<0.000001
    % map_p = scatter(-pcdmap(:,2),pcdmap(:,1),1,pcdmap(:,3),'filled'); clim([-1 1]);
    % drawnow;
    ref_p = plot(-T_ref(:,2),T_ref(:,1),'Color','red','LineWidth',1.5,'DisplayName','Ref. Path'); hold on;
end
body_yaw = agl(3);
body_x = pos(1);
body_y = pos(2);
wfl_x = body_x+a;
wfl_y = body_y-track_w/2;
wfr_x = body_x+a;
wfr_y = body_y+track_w/2;
wrl_x = body_x-b;
wrl_y = body_y-track_w/2;
wrr_x = body_x-b;
wrr_y = body_y+track_w/2;

R_lwhl = [cos(-wheel_agl(1)) -sin(-wheel_agl(1)) ;sin(-wheel_agl(1)) cos(-wheel_agl(1))];
R_rwhl = [cos(-wheel_agl(2)) -sin(-wheel_agl(2)) ;sin(-wheel_agl(2)) cos(-wheel_agl(2))];
R_bdy = [cos(-body_yaw) -sin(-body_yaw) ;sin(-body_yaw) cos(-body_yaw)];

frnt_axl = R_bdy*[1.43;0];
frnt_axl(1) = frnt_axl(1)+body_x;
frnt_axl(2) = -(frnt_axl(2)-body_y);

wfl_rec_x = [wfl_y-0.1 wfl_y+0.1 wfl_y+0.1 wfl_y-0.1];
wfl_rec_y = [wfl_x-diameter/2 wfl_x-diameter/2 wfl_x+diameter/2 wfl_x+diameter/2];
wfr_rec_x = [wfr_y-0.1 wfr_y+0.1 wfr_y+0.1 wfr_y-0.1];
wfr_rec_y = [wfr_x-diameter/2 wfr_x-diameter/2 wfr_x+diameter/2 wfr_x+diameter/2];
wrl_rec_x = [wrl_y-0.1 wrl_y+0.1 wrl_y+0.1 wrl_y-0.1];
wrl_rec_y = [wrl_x-diameter/2 wrl_x-diameter/2 wrl_x+diameter/2 wrl_x+diameter/2];
wrr_rec_x = [wrr_y-0.1 wrr_y+0.1 wrr_y+0.1 wrr_y-0.1];
wrr_rec_y = [wrr_x-diameter/2 wrr_x-diameter/2 wrr_x+diameter/2 wrr_x+diameter/2];
bdy_rec_x = [body_y-1.731/2 body_y-1.731/3 body_y+1.731/3 body_y+1.731/2 body_y+1.731/2 body_y+1.731/4 body_y-1.731/4 body_y-1.731/2];
bdy_rec_y = [body_x-4.769/2+0.3 body_x-4.769/2 body_x-4.769/2 body_x-4.769/2+0.3 body_x+4.769/2-0.5 body_x+4.769/2 body_x+4.769/2 body_x+4.769/2-0.5];

wfl_rec = [wfl_rec_x;wfl_rec_y];
wfr_rec = [wfr_rec_x;wfr_rec_y];
wrl_rec = [wrl_rec_x;wrl_rec_y];
wrr_rec = [wrr_rec_x;wrr_rec_y];
bdy_rec = [bdy_rec_x;bdy_rec_y];

for i=1:8
    if i <= 4
        wfl_rec(:,i) = R_lwhl*(wfl_rec(:,i)-[wfl_y;wfl_x]) + [wfl_y;wfl_x];
        wfr_rec(:,i) = R_rwhl*(wfr_rec(:,i)-[wfr_y;wfr_x]) + [wfr_y;wfr_x];
    
        wfl_rec(:,i) = R_bdy*(wfl_rec(:,i)-[body_y;body_x]) + [body_y;body_x];
        wfr_rec(:,i) = R_bdy*(wfr_rec(:,i)-[body_y;body_x]) + [body_y;body_x];
        wrl_rec(:,i) = R_bdy*(wrl_rec(:,i)-[body_y;body_x]) + [body_y;body_x];
        wrr_rec(:,i) = R_bdy*(wrr_rec(:,i)-[body_y;body_x]) + [body_y;body_x];
    end
    bdy_rec(:,i) = R_bdy*(bdy_rec(:,i)-[body_y;body_x]) + [body_y;body_x];
end

hold on;
bdy_sc = scatter(body_y,body_x,10,'black','filled','DisplayName','Trajectory');
wfl_sc = scatter(mean(wfl_rec(1,:)),mean(wfl_rec(2,:)),5,'filled','blue');
wfr_sc = scatter(mean(wfr_rec(1,:)),mean(wfr_rec(2,:)),5,'filled','blue');
wrl_sc = scatter(mean(wrl_rec(1,:)),mean(wrl_rec(2,:)),5,'filled','green');
wrr_sc = scatter(mean(wrr_rec(1,:)),mean(wrr_rec(2,:)),5,'filled','green');
txt = round(norm(vel(1:2))*3.6,1)+" km/h";
txt_time = time+" s";
persistent wfl_p wfr_p wrl_p wrr_p bdy_p vel_p time_p cg_sc axl_sc
if time<0.000001
    bdy_p = patch(bdy_rec(1,:),bdy_rec(2,:),'black','FaceAlpha',.5); hold on;
    wfl_p = patch(wfl_rec(1,:),wfl_rec(2,:),'black');
    wfr_p = patch(wfr_rec(1,:),wfr_rec(2,:),'black');
    wrl_p = patch(wrl_rec(1,:),wrl_rec(2,:),'black');
    wrr_p = patch(wrr_rec(1,:),wrr_rec(2,:),'black');
    vel_p = text(body_y-4,body_x+4,txt,'FontSize',11,'FontName','Arial');
    time_p = text(body_y-4,body_x+3.5,txt_time,'FontSize',11,'FontName','Arial');
    cg_sc = scatter(body_y,body_x,40,'black','+','LineWidth',1.5);
    axl_sc = scatter(frnt_axl(2),frnt_axl(1),40,'black','+','LineWidth',1.5);
else
    set(bdy_p,'XData',bdy_rec(1,:),'YData',bdy_rec(2,:));
    set(wfl_p,'XData',wfl_rec(1,:),'YData',wfl_rec(2,:));
    set(wfr_p,'XData',wfr_rec(1,:),'YData',wfr_rec(2,:));
    set(wrl_p,'XData',wrl_rec(1,:),'YData',wrl_rec(2,:));
    set(wrr_p,'XData',wrr_rec(1,:),'YData',wrr_rec(2,:));
    set(cg_sc,'XData',body_y,'YData',body_x);
    set(axl_sc,'XData',frnt_axl(2),'YData',frnt_axl(1));
    set(vel_p,'String',txt,'Position',[body_y-4,body_x+4]);
    set(time_p,'String',txt_time,'Position',[body_y-4,body_x+3.5]);
end

legend([ref_p,bdy_sc],{'Ref. Path','Trajectory'});

hold off;
axis('equal');
grid on;
ylim([body_x-(a+b)*2.5 body_x+(a+b)*2.5]);
xlim([body_y-(a+b)*2.5 body_y+(a+b)*2.5]);
xlabel('Y [m]'); ylabel('X [m]');

set(gca,'FontName','Arial','FontSize',11);
drawnow;

% coder.extrinsic('F_Video');
% F_Video();
end