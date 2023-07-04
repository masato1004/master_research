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

% smoothed_dir = "filtered_specify/";
% nonsmoothed_dir = "nonfiltered_specify/";
% onlylpf_dir = "onlylpf_specify/";

% smoothed_dir = "legacy_all/controled/";
% nonsmoothed_dir = "legacy_all/results/";
% onlylpf_dir = "legacy_all/onlylpf/";

% loading and drawing
for i = 1:length(ld_list)
    ld = ld_list(i);
    for k = 1:length(max_z0_list)
        max_z0 = max_z0_list(k);

        % if max_z0==0.08 || ld==2
            load_name = "ld="+ld+"-max_z0="+max_z0;
            load(smoothed_dir+load_name);
            load(nonsmoothed_dir+load_name);
            load(onlylpf_dir+load_name);

            % resize because of miscoding (500*500 -> 500*1)
            % pitch_inte_list_nonfilter = pitch_inte_list_nonfilter(pitch_inte_list_nonfilter(:,:)~=0);
            % pitch_max_list_nonfilter = pitch_max_list_nonfilter(pitch_max_list_nonfilter(:,:)~=0);
            % input_inte_list_nonfilter = input_inte_list_nonfilter(input_inte_list_nonfilter(:,:)~=0);
            % input_max_list_nonfilter = input_max_list_nonfilter(input_max_list_nonfilter(:,:)~=0);   % 500*1
            % 
            % pitch_inte_list_onlylpf = pitch_inte_list_onlylpf(pitch_inte_list_onlylpf(:,:)~=0);
            % pitch_max_list_onlylpf = pitch_max_list_onlylpf(pitch_max_list_onlylpf(:,:)~=0);
            % input_inte_list_onlylpf = input_inte_list_onlylpf(input_inte_list_onlylpf(:,:)~=0);
            % input_max_list_onlylpf = input_max_list_onlylpf(input_max_list_onlylpf(:,:)~=0);   % 500*1
            % 
            % pitch_inte_list = pitch_inte_list(pitch_inte_list(:,:)~=0);
            % pitch_max_list = pitch_max_list(pitch_max_list(:,:)~=0);
            % input_inte_list = input_inte_list(input_inte_list(:,:)~=0);
            % input_max_list = input_max_list(input_max_list(:,:)~=0);   % 500*1

            inte_fig = figure('name',"inte-"+load_name+".mat","Position",[100+30*(i-1)+(k-1),100+30*(i-1)+(k-1),265,200]);
            scatter(input_inte_list_nonfilter(1,pitch_inte_list_nonfilter(1,:)>thre),pitch_inte_list_nonfilter(1,pitch_inte_list_nonfilter(1,:)>thre),20,[0,0,1].*ones(size(pitch_inte_list_nonfilter(1,pitch_inte_list_nonfilter(1,:)>thre)))'); hold on; grid on; box on;
            scatter(input_inte_list_onlylpf(1,pitch_inte_list_onlylpf(1,:)>thre),pitch_inte_list_onlylpf(1,pitch_inte_list_onlylpf(1,:)>thre),20,[0,1,0].*ones(size(pitch_inte_list_onlylpf(1,pitch_inte_list_onlylpf(1,:)>thre)))');
            scatter(input_inte_list(1,pitch_inte_list(1,:)>thre),pitch_inte_list(1,pitch_inte_list(1,:)>thre),20,[1,0,0].*ones(size(pitch_inte_list(1,pitch_inte_list(1,:)>thre)))');
            ylabel("Integrations of Pitch Angle");
            xlabel("Integrations of Control Input");
            xlim([0,6e+5]);
            ylim([0.5,1.2]);
            legend("Non smoothed","Only LPF","Proposed");
            fontname(inte_fig,"Times New Roman");
            fontsize(inte_fig,9.5,"points");

            % max_fig = figure('name',"max-"+load_name+".mat","Position",[150+30*(i-1)+(k-1),100+30*(i-1)+(k-1),265,200]);
            % scatter(input_max_list_nonfilter(1,pitch_inte_list_nonfilter(1,:)>thre),pitch_max_list_nonfilter(1,pitch_inte_list_nonfilter(1,:)>thre),20,[0,0,1].*ones(size(pitch_max_list_nonfilter(1,pitch_inte_list_nonfilter(1,:)>thre)))'); hold on; grid on; box on;
            % scatter(input_max_list_onlylpf(1,pitch_inte_list_onlylpf(1,:)>thre),pitch_max_list_onlylpf(1,pitch_inte_list_onlylpf(1,:)>thre),20,[0,1,0].*ones(size(pitch_max_list_onlylpf(1,pitch_inte_list_onlylpf(1,:)>thre)))');
            % scatter(input_max_list(1,pitch_inte_list(1,:)>thre),pitch_max_list(1,pitch_inte_list(1,:)>thre),20,[1,0,0].*ones(size(pitch_max_list(1,pitch_inte_list(1,:)>thre)))');
            % ylabel("Maximums of Pitch Angle [rad]");
            % xlabel("Maximums of Control Input [N]");
            % legend("Non smoothed","Only LPF","Proposed");
            % fontname(max_fig,"Times New Roman");
            % fontsize(max_fig,9.5,"points");

            saveas(inte_fig,"figs/inte-"+load_name+".fig");
            % saveas(max_fig,"figs/max-"+load_name+".fig");
        % end
    end
end
