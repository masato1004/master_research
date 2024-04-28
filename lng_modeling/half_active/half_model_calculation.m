%% branch: lng-ctrl
close all;
clear;
clc;
branch = "_lng-ctrl_";

%% Simulation configulation files

% Simulation parameter
run("configuration_files/conf__simulation_conditions.m")

% States definition
run("configuration_files/conf__state_space.m")

% Initial conditions
run("configuration_files/conf__initial_conditions.m")

% Road profile settings
run("configuration_files/conf__rpf_settings.m")

% Control design
run("configuration_files/conf__control_design.m")

% Preview data loading
run("configuration_files/conf__preview_data_loader.m")

figfolder = "-QH-"+"all_pitch"+"-v-"+Vkm_h+"-shape-"+shape+"-hieght-"+max_z0+"-Ld-"+ld+"-freq-"+frequency+"-ctrlCycle-"+tc;
conditions = folder_maker(branch,control,shape,figfolder,smoothing_method,added_noise);

% Preview animation settings
if prev_anim
    run("configuration_files/conf__preview_animation_settings.m")
end

%% accelerate functions (without controller)
% body acceleration
% body_acc=@(zb,zbdot,zwf,zwr,ang,zwfdot,zwrdot,angdot,uf,ur) (-k_sf*(L_f*ang+zb-zwf)-k_sr*(-L_r*ang+zb-zwr)-c_sf*(L_f*angdot+zbdot-zwfdot)-c_sr*(-L_r*angdot+zbdot-zwrdot)+uf+ur)/m_b;

% front wheel acceleration
% wf_acc = @(zwf,zwfdot,zb,ang,zbdot,angdot,rp,rpdot,uf) (k_sf*(L_f*ang+zb-zwf)-k_w*(zwf-rp)+c_sf*(L_f*angdot+zbdot-zwfdot)-c_w*(zwfdot-rpdot)-uf)/m_w;

% rear wheel acceleration
% wr_acc = @(zwr,zwrdot,zb,ang,zbdot,angdot,rp,rpdot,ur) (k_sr*(-L_r*ang+zb-zwr)-k_w*(zwr-rp)+c_sr*(-L_r*angdot+zbdot-zwrdot)-c_w*(zwrdot-rpdot)-ur)/m_w;

% angular acceleration
% ang_acc =@(zb,zbdot,zwf,zwr,ang,zwfdot,zwrdot,angdot,uf,ur) (-(k_sf*(L_f*ang+zb-zwf)+c_sf*(L_f*angdot+zbdot-zwfdot))*L_f+(k_sr*(-L_r*ang+zb-zwr)+c_sr*(-L_r*angdot+zbdot-zwrdot))*L_r+(L_f*uf-L_r*ur))/I_b;


%% ========================simulation========================= %%

dw_list = [];
% LOOP
disp('Simulation Started')
for i=1:c-1
    if mod(i,1000) == 0
        disp(round(i*100/(c-1),2) + "%");
        disp("    " + round(TL(i),2)+"[s], " + round(states(1,i),2) + "[m], " + round(states(8,i),2) + "[m/s]");
    end
    % make road preview profile
    if mod(i+(ts/dt-1), ts/dt) == 0
        current_dis = r_p_prev(1,i);
        % load road profile
        if sensing
            if sc <= height(listing)
                file = load("configuration_files/"+load_dir + "/" + listing(sc).name);
                vertices = file.vertices;
                wf_local = previewing(vertices);
                % wf_local(isnan(wf_local(2,:))) = data_end;
                wf_local = rmmissing(wf_local,2);
                data_end = wf_local(2,end);
                % wf_local(2,:) = movmean(wf_local(2,:),width(wf_local(2,wf_local(1,:))));  % move mean with mm_ratio
                % wf_local(2,:) = movmean(wf_local(2,:),round(mm_ratio*width(wf_local)));  % move mean with mm_ratio
                % display("HERE"+sc);
            else
                wf_local = [
                    prev_start:tc*V:prev_end;
                    interp1(r_p_prev(1,:),data_end*ones(size(r_p_prev(1,:))),current_dis+prev_start:tc*V:current_dis+prev_end,'linear')
                    ];
            end
        else
            wf_local = [
                prev_start:tc*V:prev_end;
                makima(r_p_prev(1,:),r_p_prev(2,:),current_dis+prev_start:tc*V:current_dis+prev_end)
                ];
        end
        
        % Noise addition
        if high_freq_noise
            hnoise = hsd*randn(size(wf_local(2,:)));
            wf_local(2,:) = wf_local(2,:) + hnoise;
        end
        if low_freq_noise
            sig = 3-2*randi(2);
            % lnoise = sig*lsd(wf_local(1,:))+(lsd(wf_local(1,:))./3).*randn(size(wf_local(2,:)));
            lnoise = sig*lsd((0.03/7)*randn(),(0.005)*randn(),wf_local(1,:));
            wf_local(2,:) = wf_local(2,:) + lnoise;
            [~,ind] = sort(wf_local(1,:));
            wf_local= wf_local(:,ind);
        end

        if wa
            Lpass = wf_local(1,1) - last_minimum;
            last_minimum = wf_local(1,1);
            Ltotal = wf_local(1,end) - wf_local(1,1);
            eta = Lpass/Ltotal;
            eta_1 = 0.5*(1-eta);
            eta_2 = 0.5*(1+eta);
            % WA
            if sensing
                wf_global = [
                    wf_global(1, wf_global(1,:)<wf_local(1,1) & wf_global(1,:)>-200), wf_local(1,:);
                    wf_global(2, wf_global(1,:)<wf_local(1,1) & wf_global(1,:)>-200), interp1(wf_global(1,:), wf_global(2,:), wf_local(1, wf_local(1,:)<=wf_global(1,end)),'linear').*eta_1 + wf_local(2, wf_local(1,:)<=wf_global(1,end)).*eta_2, wf_local(2, wf_local(1,:)>wf_global(1,end))
                    ];

            else
                wf_global = [
                    wf_global(1, wf_global(1,:)<wf_local(1,1) & wf_global(1,:)>-2), wf_local(1,:);
                    wf_global(2, wf_global(1,:)<wf_local(1,1) & wf_global(1,:)>-2), makima(wf_global(1,:), wf_global(2,:), wf_local(1, wf_local(1,:)<=wf_global(1,end))).*eta_1 + wf_local(2, wf_local(1,:)<=wf_global(1,end)).*eta_2, wf_local(2, wf_local(1,:)>wf_global(1,end))
                    ];
            end

        else
            wf_global = [wf_global, wf_local];
            [~,ind] = sort(wf_global(1,:));
            wf_global=wf_global(:,ind);
        end


        % LPF
        notnan_wfg = rmmissing(wf_global,2);
        notnan_wfg(1,:) = notnan_wfg(1,:)./V;
        % WA + filtfilt
        [~,ia,~]=unique(notnan_wfg(1,:));
        notnan_wfg = notnan_wfg(:,ia);
        grad_time = notnan_wfg(1,1):tc:notnan_wfg(1,end);
        grad_data1 = interp1(notnan_wfg(1,:), notnan_wfg(2,:), grad_time, "linear");
        % disp(size(grad_time));
        if lpf
            grad_data1 = filtfilt(filt_des,double(grad_data1));
        end
        grad_data2 = gradient(grad_data1)./gradient(grad_time);
        wf_grad = [
            grad_time;
            grad_data1;
            grad_data2
            ];

        sc = sc + 1;
    end
    % set(check_plot, "XData", wf_grad(1,:), "YData", wf_grad(3,:));
    % drawnow;

    % load current states
    x = states(1:4,i);
    v = states(5:8,i);

    % load road profile
    x_disf = disturbance(1,i);
    x_disr = disturbance(2,i);
    z_disf = disturbance(3,i);
    z_disr = disturbance(4,i);
    dx_disf =disturbance(5,i);
    dx_disr =disturbance(6,i);
    dz_disf =disturbance(7,i);
    dz_disr =disturbance(8,i);
    
    % load current input
    u_in = u(:,i);

    % Apply current parameter into matrices
    Bp = double(subs(Bmat));
    Ep = double(subs(Emat));
    % G  = double(subs(Gmat));


    % States-Update Runge kutta
    states(:,i+1) = runge(states(:,i), u_in, disturbance(:,i), g, Ap, Bp, Ep, Gmat, dt, r, wheel_traj_f,wheel_traj_r,mileage_f,mileage_r,dis_total);
    accelerations(:,i) = [states(8,i+1)-states(8,i);states(9,i+1)-states(9,i);states(12,i+1)-states(12,i)]./dt;
 
    % find appropriate next input
    if mod(i-1, (tc/dt)) == 0 && i ~= 1 && ~passive && ~NLMPC
        cc = (i-1)/(tc/dt)+1;                   % list slice
        [~,ia,~]=unique(wf_grad(1,:));
        wf_grad = wf_grad(:,ia);
        dw_prev = [
            interp1(wf_grad(1,:),[0,diff(wf_grad(2,:))],[0:M].*tc,'linear');
            dw_r(2, [0:M]+cc);
            interp1(wf_grad(1,:),[0,diff(wf_grad(3,:))],[0:M].*tc,'linear');
            dw_r(4, [0:M]+cc);
            ];
        dw_list = [dw_list,dw_prev];

        % calculate new states
        e(:,cc) = -C*states(:,i);                        % e=r-y
        dx(:,cc) = states(:,i) - states(:,i-(tc/dt));    % dx(k)=x(k)-x(k-1)
        X(1:height(e),cc) = e(:,cc);                     % X=[e;dx]
        X(height(e)+1:end,cc) = dx(:,cc);

        % calculate input
        du(:,cc+1) = next_input(logi_ctrl,M,F,X(:,cc),FDW(:,cc),Fdj,wf_grad(1,1),dw_r(:, cc:cc+M),dw_prev,dw_fr(:, cc:cc+M));

        if cc ~= 1
            u(:, cc+1) = u(:, cc) + du(:, cc+1);
        else
            u(:, cc+1) = du(:, cc+1);
        end
        wf_global(1,:) = wf_global(1,:) - tc*V; last_minimum = last_minimum - tc*V;
        wf_grad(1,:) = wf_grad(1,:) - tc;

        if prev_anim
            set(check_plot0, "XData", wf_local(1,:), "YData", wf_local(2,:));  % blue
            set(check_plot, "XData", wf_global(1,:), "YData", wf_global(2,:)); % red
            set(check_plot2, "XData", wf_grad(1,:).*V, "YData", wf_grad(2,:)); % green
            % display(cc);
            
            txdata = round(TL(1,i),2);
            str = {"Time [s]",txdata};
            time_text.String = str;
            % time_text.Position = [0.05, 0.05];
            time_text.Position = [8, 0.05];
            drawnow;
            frame = getframe(check);
            writeVideo(video,frame);
        end
    elseif NLMPC
        current_mileage_f = mileage_f-makima(dis_total,mileage_f,disturbance(1,i));
        simdata.Measured
        simdata.StageParmeter = repmat(referenceSignal(states(:,i),u,dx_init,Ts),pHorizon,1);
        [mv,nloptions] = nlmpcmove(runner,states(:,i),u_in,simdata);
        u(:,i+1) = mv;
    else
        u(:,i+1) = u(:,i);
    end

    disturbance(1,i+1) = makima(mileage_f-makima(dis_total,mileage_f,disturbance(1,i)),dis_total,r*(states(6,i+1)-states(6,i)));  % x_disf
    disturbance(2,i+1) = makima(mileage_r-makima(dis_total,mileage_r,disturbance(2,i)),dis_total,r*(states(7,i+1)-states(7,i)));  % x_disr
    disturbance(3,i+1) = makima(wheel_traj_f(1,:),wheel_traj_f(2,:),disturbance(1,i+1));                                                                % z_disf
    disturbance(4,i+1) = makima(wheel_traj_f(1,:),wheel_traj_r(2,:),disturbance(2,i+1));                                                                % z_disr
    if i ~= 1
        disturbance(5,i+1) = (diff(disturbance(1,i-1:i))/dt + diff(disturbance(1,i:i+1))/dt)/2;                                         % dx_disf
        disturbance(6,i+1) = (diff(disturbance(2,i-1:i))/dt + diff(disturbance(2,i:i+1))/dt)/2;                                         % dx_disr
        disturbance(7,i+1) = (diff(disturbance(3,i-1:i))/dt + diff(disturbance(3,i:i+1))/dt)/2;                                         % dz_disf
        disturbance(8,i+1) = (diff(disturbance(4,i-1:i))/dt + diff(disturbance(4,i:i+1))/dt)/2;                                         % dz_disr
    else
        disturbance(5,i+1) = diff(disturbance(1,i:i+1))/dt;                                         % dx_disf
        disturbance(6,i+1) = diff(disturbance(2,i:i+1))/dt;                                         % dx_disr
        disturbance(7,i+1) = diff(disturbance(3,i:i+1))/dt;                                         % dz_disf
        disturbance(8,i+1) = diff(disturbance(4,i:i+1))/dt;                                         % dz_disr
    end
end

if prev_anim
    close(video);
end

% pitch_max = max(abs(states(4,:)));
% pitch_integral = trapz(TL(1:end-width(states(4,isnan(states(4,:))))),abs(rad2deg(double(states(4,1:end-width(states(4,isnan(states(4,:)))))))))
% pitchacc_integral = trapz(TL(1:end-width(states(8,isnan(states(8,:))))),abs(rad2deg(double(states(8,1:end-width(states(8,isnan(states(8,:)))))))));
% input_max = max(abs(u(1,:)));
% input_integral = trapz(control_TL(1:end-width(u(1,isnan(u(1,:))))),abs(rad2deg(double(u(1,1:end-width(u(1,isnan(u(1,:)))))))));

pitch_max = max(abs(states(5,:)));
pitch_integral = trapz(TL(1,TL>(start_disturbance-1)/V&TL<(start_disturbance+ld+1)/V),abs(rad2deg(double(states(5,TL>(start_disturbance-1)/V&TL<(start_disturbance+ld+1)/V)))))
pitchacc_integral = trapz(TL(1,TL>(start_disturbance-1)/V&TL<(start_disturbance+ld+1)/V),abs(rad2deg(double(states(8,TL>(start_disturbance-1)/V&TL<(start_disturbance+ld+1)/V)))));
input_max = max(abs(u(1,:)));
input_integral = trapz(control_TL(control_TL>(start_disturbance-1)/V&control_TL<(start_disturbance+ld+1)/V),abs(rad2deg(double(u(1,control_TL>(start_disturbance-1)/V&control_TL<(start_disturbance+ld+1)/V)))));

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %
%                          Drawing figures                          %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %
states_name = [
    "Longitudinal_Position", "Time [s]", "Longitudinal Position [m]";
    "Body_Heave_Displacement", "Time [s]", "Body Heave Displacement [m]";
    "Front_Wheel_Heave_Displacement", "Time [s]", "Front Wheel Heave Displacement [m]";
    "Rear_Wheel_Heave_Displacement", "Time [s]", "Rear Wheel Heave Displacement [m]";
    "Body_Pitch_Angle", "Time [s]", "Body Pitch Angle [deg]";
    "Front_Wheel_Angle", "Time [s]", "Front Wheel Angle [deg]";
    "Rear_Wheel_Angle", "Time [s]", "Rear Wheel Angle [deg]";
    "Velocity", "Time [s]", "Velocity [m/s]";
    "Body_Heave_Velocity", "Time [s]", "Body Heave Velocity [m/s]";
    "Front_Wheel_Heave_Velocity", "Time [s]", "Front Wheel Heave Velocity [m/s]";
    "Rear_Wheel_Heave_Velocity", "Time [s]", "Rear Wheel Heave Velocity [m/s]";
    "Body_Pitch_Angular_Velocity", "Time [s]", "Body Pitch Angular Velocity [deg/s]"
    "Front_Wheel_Angular_Velocity", "Time [s]", "Front Wheel Angular Velocity [deg]";
    "Rear_Wheel_Angular_Velocity", "Time [s]", "Rear Wheel Angular Velocity [deg]";
    ];

% r_fig = figure('name',"Road-profile: Frequency "+frequency+" Hz",'Position', [600 200 600 190]);
% plot(dis,r_p_f,"LineWidth",1,"Color","#0000ff");
% ylabel("Displacement [m]");
% xlabel("Distance Traveled [m]");
% % ylim([-0.01,0.1])
% % ylim([-0.03,0.04])
% fontname(r_fig,"Times New Roman");
% fontsize(r_fig,10.5,"points");
% grid on;

% saveas(r_fig,"figs/"+conditions+"/Road_Profile.fig");
% saveas(r_fig,"jpgs/"+conditions+"/Road_Profile.jpg");

for i=1:height(states)
    if sum(ismember([5,6,7,12,13,14],i))
        drawer(TL,states(i,:)*(180/pi),states_name(i,:),i,conditions);
    else
        drawer(TL,states(i,:),states_name(i,:),i,conditions);
    end
end


%% additional draw
% accelerations
drawer(TL,accelerations(1,:),["Body_Longitudinal_Acceleration", "Time [s]", "Body Longitudinal Acceleration [m/s^2]"],i+1,conditions);
drawer(TL,accelerations(2,:),["Body_Vertical_Acceleration", "Time [s]", "Body Vertical Acceleration [m/s^2]"],i+2,conditions);
drawer(TL,accelerations(3,:)*(180/pi),["Body_Pitch_Angular_Acceleration", "Time [s]", "Body Pitch Angular Acceleration [deg/s^2]"],i+3,conditions);

fig_tile = figure();
for i=1:height(states)
    subplot(3,5,i)
    if sum(ismember([5,6,7,12,13,14],i))
        tile_drawer(TL,states(i,:)*(180/pi),states_name(i,:),i,conditions);
    else
        tile_drawer(TL,states(i,:),states_name(i,:),i,conditions);
    end
end
subplot(3,5,i+1)
tile_drawer(TL,accelerations(1,:),["Body_Longitudinal_Acceleration", "Time [s]", "Body Longitudinal Acceleration [m/s^2]"],i+1,conditions);
fontname(fig_tile,"Times New Roman");
fontsize(fig_tile,18,"points");
saveas(fig_tile,"figs/"+conditions+"/tile");
saveas(fig_tile,"jpgs/"+conditions+"/tile.jpg");

% % preview data
% p_fig = figure('name',"Preview data",'Position', [620 250 600 190]);
% plot(dis_dw,dw_fr(1,:),"LineWidth",1,"Color","#0000ff");
% ylabel("Displacement [m]");
% xlabel("Distance Traveled [m]");
% fontname(p_fig,"Times New Roman");
% fontsize(p_fig,10.5,"points");
% grid on;

% % difference of preview data
% p_fig = figure('name',"Preview data",'Position', [620 250 600 190]);
% yyaxis left;
% plot(dis_dw,dw_fr(1,:),"LineWidth",1,"Color","#0000ff");
% ylabel("Displacement [m]");
% xlabel("Distance Traveled [m]");
% hold on;
% ax = gca;
% ax.YAxis(1).Color = [0 0 1];
% yyaxis right;
% plot(dis_dw,dw_fr(3,:),"LineWidth",1,"Color","#ff0000");
% ylabel("Displacement [m]");
% xlabel("Distance Traveled [m]");
% ax.YAxis(2).Color = [1 0 0];
% % ylim([-0.01,0.1])
% % ylim([-0.03,0.04])
% fontname(p_fig,"Times New Roman");
% fontsize(p_fig,10.5,"points");
% grid on;

% drawer(control_TL,u(1,:),["Front_Wheel_Actuator_Force", "Time [s]", "Front Wheel Actuator Force [N]"],11,figfolder,shape);
% drawer(control_TL,u(2,:),["Rear_Wheel_Actuator_Force", "Time [s]", "Rear Wheel Actuator Force [N]"],12,figfolder,shape);
% fig = figure('name',"Actuator_Force",'Position', [500+20*11 500-20*11 600 190]);
% plot(control_TL,u(1,:),"LineWidth",2,"Color","#0000ff");
% hold on;
% plot(control_TL,u(2,:),"LineWidth",2,"Color","#0000ff","LineStyle","--");
% grid on;
% xlim([0,3]);
% xlabel("Time [s]");
% ylabel("Wheel Actuator Force [N]");
% legend("{\it f_{af}} : Front Wheel", "{\it f_{ar}} : Rear Wheel");
% fontname(fig,"Times New Roman");
% fontsize(fig,10.5,"points");
% saveas(fig,"figs/"+conditions+"/Actuator_Force.fig");
% saveas(fig,"jpgs/"+conditions+"/Actuator_Force.jpg");

% % compare the timings of rear road profile and actuator input
% fig = figure('name',"Actuator_Force_and_Road",'Position', [500+20 500-20 600 190]);
% xlim([0,3]);
% yyaxis left
% plot(control_TL,u(1,:),"LineWidth",2,"Color","#0000ff");
% hold on;
% plot(control_TL,u(2,:),"LineWidth",2,"Color","#0000ff");
% grid on;
% xlabel("Time [s]");
% ylabel("Wheel Actuator Force [N]");
% ylim([-4000,4000]);
% ax = gca;
% ax.YAxis(1).Color = [0 0 1];
% yyaxis right
% ylim([-0.09,0.09]);
% plot(TL,r_p_f,"LineWidth",2,"Color","#ff0000");
% plot(TL,r_p_r,"LineWidth",2,"Color","#ff0000");
% ylabel("Road Displacement [m]");
% legend("{\it f_{af}} : Front Actuator", "{\it f_{ar}} : Rear Actuator", "{\it z_{0f}} : Front Road Displacement", "{\it z_{0r}} : Rear Road Displacement");
% ax = gca;
% ax.YAxis(2).Color = [1 0 0];
% fontname(fig,"Times New Roman");
% fontsize(fig,10.5,"points");
% grid on;
% saveas(fig,"figs/"+conditions+"/Actuator_Force_and_Road.fig");
% saveas(fig,"jpgs/"+conditions+"/Actuator_Force_and_Road.jpg");

if animation
    close all;
    clear frames;
    run("half_model_animation.m")
end