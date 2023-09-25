close all
% videoname = "videos/_half_thesis_sensor_/"+"_LQR_fprev_rprev_"+"/"+shape+"/PreviewedRoad";
% video = VideoWriter(videoname,'MPEG-4');
% video.FrameRate = (1/tc)/100;
% open(video);

% close all;
check = figure('name',"preview road",'Position', [500-20 500-20 600 190]);
% check_plot = plot(wf_global(1,:),wf_global(2,:),"LineWidth",2,"Color","#0000ff");
% xline(prev_start-mm_range*V,'--');
% mov_line = xline(prev_end,'--');
% xlim([-0.05,0.1]);
% xlim([-1,10]);
ylim([-0.06,0.14]);
grid on;
hold on;
% xlabel("Preview Time [s]");
xlabel("Local Distance from Front Wheel [m]");
ylabel("Previewed Road Displacement [m]");
% xline(prev_start,'--',"\bf Nearest Preview",DisplayName="");
% xline(prev_end,'--',"\bf Farest Preview",DisplayName="");
% time count

load_dir = "sim_config_mfiles/preview_datas";
listing = dir(load_dir+"/*.mat");
tc = 0.001;
ts = 0.02;
V = 50*1000/3600;

map = colormap(check,"jet");
data_num = 23;
disp_name = ["$\it{k}$","",""];
pick_up = 23;
pu = 1;

all_list = [];

for sc = 1:round(data_num/pick_up):23
    file = load(load_dir + "/" + listing(sc).name);
    vertices = file.vertices;
    wf_local = previewing(vertices);
    wf_local=rmmissing(wf_local,2);
    data_end = wf_local(2,end);

    %% Fig3-8-30
%     check = figure('name',"preview road",'Position', [500-sc*2 500-sc*2 600 190]);
%     % check_plot = plot(wf_global(1,:),wf_global(2,:),"LineWidth",2,"Color","#0000ff");
%     % xline(prev_start-mm_range*V,'--');
%     % mov_line = xline(prev_end,'--');
%     % xlim([-0.05,0.1]);
%     % xlabel("Preview Time [s]");
%     plot(wf_local(1,:),wf_local(2,:),"LineWidth",2,Color="#0000ff");
% %     xlim([4.5,7.5]);
%     ylim([-0.1,0.15]);
%     xlabel("Ground Distance from Sensor [m]");
    ylabel("Displacement [m]");
    fontname(check,"Times New Roman");
    fontsize(check,10.5,"points");
    grid on;

    %% Fig4-2, 4-3
    if sc ~=1
        wf_local(1,:) = wf_local(1,:) + ts*V*(sc-1);
    end
    % wf_local(1,:) = wf_local(1,:) - ts*V*(3-sc);
    all_list = [all_list, wf_local];
    % figure;
    % plot(wf_local(1,:),wf_local(2,:),"LineWidth",2,Color='r');
    % plot(wf_local(1,:),wf_local(2,:),"LineWidth",2,Color=map(sc,:));
    % check_plot = plot(wf_local(1,:),wf_local(2,:),"LineWidth",2,Color=map(sc*65,:));
    drawnow;
    if pu ~= pick_up
        disp_name = ["$\it{k}"+"\rm-"+pu+"$", disp_name];
%         disp_name = ["$\it{\tilde{w}}_{pre}(\it{k}"+"\rm-"+sc+")$", disp_name];

        if sc == 1
            wf_global = wf_local;
            last_minimum = wf_local(1,1);
        else
            Lpass = wf_local(1,1) - last_minimum;
            last_minimum = wf_local(1,1);
            Ltotal = wf_local(1,end) - wf_local(1,1);
            eta = Lpass/Ltotal;
            eta_1 = 0.5*(1-eta);
            eta_2 = 0.5*(1+eta);
            [~,ia,~]=unique(wf_global(1,:));
            wf_global = wf_global(:,ia);
            wf_global = [
                wf_global(1, wf_global(1,:)<wf_local(1,1) & wf_global(1,:)>-200), wf_local(1,:);
                wf_global(2, wf_global(1,:)<wf_local(1,1) & wf_global(1,:)>-200), interp1(wf_global(1,:), wf_global(2,:), wf_local(1, wf_local(1,:)<=wf_global(1,end)),'linear').*eta_1 + wf_local(2, wf_local(1,:)<=wf_global(1,end)).*eta_2, wf_local(2, wf_local(1,:)>wf_global(1,end))
                ];
        end

    end
    % Lpass = wf_local(1,1) - last_minimum;
    % last_minimum = wf_local(1,1);
    % Ltotal = wf_local(1,end) - wf_local(1,1);
    % eta = Lpass/Ltotal;
    % eta_1 = 0.5*(1-eta);
    % eta_2 = 0.5*(1+eta);
    % [~,ia,~]=unique(wf_global(1,:));
    % wf_global = wf_global(:,ia);
    % wf_global = [
    %     wf_global(1, wf_global(1,:)<wf_local(1,1) & wf_global(1,:)>-200), wf_local(1,:);
    %     wf_global(2, wf_global(1,:)<wf_local(1,1) & wf_global(1,:)>-200), interp1(wf_global(1,:), wf_global(2,:), wf_local(1, wf_local(1,:)<=wf_global(1,end)),'linear').*eta_1 + wf_local(2, wf_local(1,:)<=wf_global(1,end)).*eta_2, wf_local(2, wf_local(1,:)>wf_global(1,end))
    %     ];
    % notnan_wfg = rmmissing(wf_global,2);
    % notnan_wfg(1,:) = notnan_wfg(1,:)./V;
    % 
    % [~,ia,~]=unique(notnan_wfg(1,:));
    % notnan_wfg = notnan_wfg(:,ia);
    % grad_time = notnan_wfg(1,1):tc:notnan_wfg(1,end);
    % grad_data1 = interp1(notnan_wfg(1,:), notnan_wfg(2,:), grad_time, "linear");
    % grad_data2 = gradient(grad_data1)./(gradient(grad_time));
    % wf_grad = [
    %     grad_time;
    %     grad_data1;
    %     grad_data2
    %     ];
end
check_plot_global=figure(Position=[500-sc*2 500-sc*2 600 250]);
plot(wf_global(1,:),wf_global(2,:),"LineWidth",2,Color="#0000ff");
fontname(check_plot_global,"Times New Roman");
fontsize(check_plot_global,9.5,"points");
grid on;
xlabel("Local Distance from Front Wheel [m]");
ylabel("Displacement [m]");
pu = pu + 1;

[~,ind] = sort(all_list(1,:));
all_list=all_list(:,ind);
all_plot = figure('name',"preview road",'Position', [500-20 500-20 600 190]);
plot(all_list(1,:),all_list(2,:),"LineWidth",2,Color=[0 0 1]);
legend(disp_name,'Interpreter','latex');
xline(0,'--r',"Front Wheel",DisplayName="");
fontname(check,"Times New Roman");
fontsize(check,10.5,"points");