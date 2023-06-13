%% Code for drawing of Monte Carlo simulation results
clear;
close all;
% simulation conditions
ld_list = [0.25, 0.5, 1, 1.5, 2];
max_z0_list = [0.02, 0.03, 0.04, 0.05, 0.08];

% mat file folder
smoothed_dir = "controled/";
nonsmoothed_dir = "results/";

% loading and drawing
for i = 1:5
    ld = ld_list(i);
    for k = 1:5
        max_z0 = max_z0_list(k);

        if max_z0==0.08 || ld==2
            load_name = "ld="+ld+"-max_z0="+max_z0;
            load(smoothed_dir+load_name);
            load(nonsmoothed_dir+load_name);

            % resize because of miscoding (500*500 -> 500*1)
            pitch_inte_list = pitch_inte_list(pitch_inte_list(:,:)~=0);
            pitch_max_list = pitch_max_list(pitch_max_list(:,:)~=0);
            input_inte_list = input_inte_list(input_inte_list(:,:)~=0);
            input_max_list = input_max_list(input_max_list(:,:)~=0);   % 500*1

            inte_fig = figure('name',"inte-"+load_name+".mat");
            scatter(input_inte_list,pitch_inte_list,20,[1,0,0].*ones(size(input_inte_list))); hold on; grid on; box on;
            scatter(input_inte_list_nonfilter,pitch_inte_list_nonfilter,20,[0,0,1].*ones(size(input_inte_list)));
            ylabel("Integrations of Pitch Angle");
            xlabel("Integrations of Control Input");
            fontname(inte_fig,"Times New Roman");
            fontsize(inte_fig,9.5,"points");

            max_fig = figure('name',"max-"+load_name+".mat");
            scatter(input_max_list,pitch_max_list,20,[1,0,0].*ones(size(input_max_list))); hold on; grid on; box on;
            scatter(input_max_list_nonfilter,pitch_max_list_nonfilter,20,[0,0,1].*ones(size(input_max_list)));
            ylabel("Maximums of Pitch Angle [rad]");
            xlabel("Maximums of Control Input [N]");
            fontname(max_fig,"Times New Roman");
            fontsize(max_fig,9.5,"points");

            saveas(inte_fig,"figs/smoothed/inte-"+load_name+".fig");
            saveas(max_fig,"figs/smoothed/max-"+load_name+".fig");
            saveas(inte_fig,"figs/nonsmoothed/inte-"+load_name+".fig");
            saveas(max_fig,"figs/nonsmoothed/max-"+load_name+".fig");
        end
    end
end
