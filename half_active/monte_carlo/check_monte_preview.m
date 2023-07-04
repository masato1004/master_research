%% Code for drawing of Monte Carlo simulation results
clear;
close all;
% simulation conditions
ld_list = [0.5, 1, 2, 2.5];
max_z0_list = [0.01, 0.045, 0.08];
% ld_list = [0.25, 0.5, 1, 1.5, 2];
% max_z0_list = [0.02, 0.03, 0.04, 0.05, 0.08];

thre = 0.3;

% mat file folder
smoothed_dir = "filtered_all/";
nonsmoothed_dir = "nonfiltered_all/";
onlylpf_dir = "onlylpf_all/";

for i = 1:length(ld_list)
    ld = ld_list(i);
    for k = 1:length(max_z0_list)
        max_z0 = max_z0_list(k);
        load_name = "ld="+ld+"-max_z0="+max_z0;
        load(smoothed_dir+load_name);
        load(nonsmoothed_dir+load_name);
        load(onlylpf_dir+load_name);

        figure('name',"inte-"+load_name+".mat","Position",[100+30*(i-1)+(k-1),100+30*(i-1)+(k-1),265,200]);
        % for s = 1:length(wf_global_nonfilter)
        for s = 4:4
            prev_road = wf_global_nonfilter(s);
            prev_road = prev_road{1};
            plot(prev_road(1,:),prev_road(2,:),"LineWidth",0.5,"Color","#0000ff");
            grid on; hold on;

            prev_road = wf_global_onlylpf(s);
            prev_road = prev_road{1};
            plot(prev_road(1,:),prev_road(2,:),"LineWidth",0.5,"Color","#00ff00");

            prev_road = wf_global_filter(s);
            prev_road = prev_road{1};
            plot(prev_road(1,:),prev_road(2,:),"LineWidth",0.5,"Color","#ff0000");
            xlim([5,18]);
        end
        plot(r_p_prev(1,:),r_p_prev(2,:),"LineWidth",2,"Color","#000000","LineStyle","--")
        legend('Non filtered','Only LPF', 'Proposed','Actual Road')
    end
end
