figure('Position',[100 100 800 300]);

obs_num = width(out.ControllerOut.obs_data.obs_detected.Data);
obs_detected_Data = reshape(out.ControllerOut.obs_data.obs_detected.Data,[obs_num,length(out.ControllerOut.obs_data.obs_detected.Data)]);
obs_r_Data = reshape(out.ControllerOut.obs_data.obs_r.Data,[obs_num,length(out.ControllerOut.obs_data.obs_r.Data)]);
obs_theta_Data = reshape(out.ControllerOut.obs_data.obs_theta.Data,[obs_num,length(out.ControllerOut.obs_data.obs_theta.Data)]);
% obs_num = width(out.ControllerOut.obs_data.obs_detected.Data);
for k = 1:obs_num
    detected_idx = obs_detected_Data(k,:)+1;
    color = ['b','r'];
    data = obs_r_Data(k,2:end);
    unvisible_points = data(data<6-opt_params_wb/2);
    data_range = 2:find(data==unvisible_points(1));
    for i = data_range
        if obs_r_Data(k,i).*cos(obs_theta_Data(k,i)) < 20-opt_params_wb/2
            scatter(opt_params_wb/2+obs_r_Data(k,i).*cos(obs_theta_Data(k,i)),obs_r_Data(k,i).*sin(obs_theta_Data(k,i)),30,color(detected_idx(i)),"filled","AlphaData",0.8);
            hold on; grid on;
            drawnow;
        end
    end
    % plot(obs_r_Data(data_range).*cos(obs_theta_Data(data_range)),obs_r_Data(data_range).*sin(obs_theta_Data(data_range)),'-*')
end
axis equal
ylim([-opt_params_lane_width/2,opt_params_lane_width/2])
xlim([0 25])
xline(opt_params_wb,"--k","Front"+newline+"Wheels",'LabelHorizontalAlignment','right','LabelVerticalAlignment','bottom','LabelOrientation','aligned')
xlabel('\itX \rm[m]');
ylabel('\itY \rm[m]');
fontname(gcf,"Times New Roman");
fontsize(gca,11,"points");