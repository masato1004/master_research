% check preview road
videoname = "../videos/"+branch+"/"+"_LQR_fprev_rprev_"+"/"+shape+"/PreviewedRoad--"+"-v-"+Vkm_h+"-shape-"+shape+"-hieght-"+max_z0+"-Ld-"+ld;
video = VideoWriter(videoname,'MPEG-4');
video.FrameRate = (1/tc)/100;
open(video);

% Video settings
check = figure('name',"preview road",'Position', [500-20 500-20 1000 380]);
check_plot = plot(wf_global(1,:),wf_global(2,:),"LineWidth",2,"Color","#ff0000");
hold on;
check_plot0 = plot(wf_global(1,:),wf_global(2,:),"LineWidth",2,"Color","#0000ff");
check_plot2 = plot(wf_global(1,:),wf_global(2,:),"LineWidth",2,"Color","#00ff00");
xline(0,'--r',"Front Wheel");
xline(prev_start,'--');
xline(prev_end,'--');
% xline(prev_start-mm_range*V,'--');
mov_line = xline(prev_end,'--');
% xlim([-0.05,0.1]);
xlim([-1,10]);
ylim([-0.1,0.1]);
grid on;
% xlabel("Preview Time [s]");
xlabel("Local Distance from Front Wheel [m]");
ylabel("Previewed Road Displacement [m]");
% time count
txdata = round(TL(1,1),2);
str = {"Time [s]",txdata};
time_text = text(0, 0.01, str);
time_text.FontSize = 20;
fontname(check,"Times New Roman");
fontsize(check,16,"points");