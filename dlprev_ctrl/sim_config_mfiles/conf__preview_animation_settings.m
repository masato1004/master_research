% check preview road
videoname = "../videos/"+conditions+"/preview_check";
video = VideoWriter(videoname,'MPEG-4');
video.FrameRate = (1/tc)/100;
open(video);

% Video settings
check = figure('name',"preview road",'Position', [500-20 500-20 1000 380]);
ground_truth = plot(0:tc*V:5.5,interp1(r_p_prev(1,:),r_p_prev(2,:),0:tc*V:5.5,'linear'),"LineWidth",2,"Color","#000000");
hold on;
% check_plot = plot(wf_global(1,:),wf_global(2,:),"LineWidth",2,"Color","#ff0000");
% check_plot0 = plot(wf_global(1,:),wf_global(2,:),"LineWidth",2,"Color","#0000ff");
check_plot2 = plot(wf_global(1,:),wf_global(2,:),"LineWidth",2,"Color","#00ff00");

check_plot  = scatter(wf_global(1,:),wf_global(2,:),10,'filled',"MarkerFaceColor","#ff0000","MarkerEdgeColor","#ff0000");
check_plot0 = scatter(wf_global(1,:),wf_global(2,:),10,'filled',"MarkerFaceColor","#0000ff","MarkerEdgeColor","#0000ff");
% check_plot2 = scatter(wf_global(1,:),wf_global(2,:),10,'filled',"MarkerFaceColor","#00ff00","MarkerEdgeColor","#00ff00");

% check_plot.Color(4) = 0.1;
% check_plot0.Color(4) = 0.1;
check_plot2.Color(4) = 0.7;
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