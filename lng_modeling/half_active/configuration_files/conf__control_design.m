%% control design
% parameters
control_TL = 0:tc:T;                       % time line for control results
c_ctrl = width(control_TL);
u = zeros(4,c);                            % input vector
du= zeros(4,c);                            % input-differencial vector
e = zeros(height(C),c);                    % output-error vector
dx= zeros(height(states),c);               % states-differencial vector

X = [
    e;
    dx
    ];        % new states vector

% LQR
Q = diag([1e-06, 1e+05, 1e-03, 1e-03, 1e-03, 1e+06, 1e-03, 1e-03, 1e-03, 1e+06]);       % all_pitch
H = diag([1e-03,1e-03]);

[P, K, ~] = idare(phi, G, Q, H, [], []);   % u = -Kx
F = -K;                                      % u = Fx

% % preview controler
% % preview parameter
M = 43;                        % preview step
FDW = zeros(2, c_ctrl);
Fd = @(j) -(H + G'*P*G)\G'*(((phi+G*F)')^(j))*P*Gd;       % function for Fd(j)
Fdj = zeros(2, height(r_p), M+1);                 % feedforward gain vector list
for j = 0:M
    Fdj(:,:,j+1)=Fd(j);
end

dis_dw = 0:(T*V)/(T/tc):T*V+3;

if ~sensing
    r_p_f_dw = makima(dis_total,road_total_f,dis_dw);
    r_p_r_dw = makima(dis_total,road_total_r,dis_dw);
else
    r_p_f_dw = interp1(road_total_f(:,1)',road_total_f(:,2)'+1.46,dis_dw,'linear');
    r_p_r_dw = interp1(road_total_r(:,1)',road_total_r(:,2)'+1.46,dis_dw,'linear');
end

w_r = [
    zeros(size(r_p_r_dw));
    r_p_r_dw;
    zeros(size(r_p_r_dw));
    gradient(r_p_r_dw)./tc
    ];
dw_r = [0, 0, 0, 0; diff(w_r')]';         % preview road profile differencial

w_fr = [
    r_p_f_dw;
    r_p_r_dw;
    gradient(r_p_f_dw)./tc;
    gradient(r_p_r_dw)./tc
    ];
    
dw_fr = [0, 0, 0, 0; diff(w_fr')]';         % preview road profile differencial


%% Feedforward settings
detailed_dt = 1e-05;
detailed_dis = 0:V*detailed_dt:max_distance;
detailed_wheel_traj_f = makima(wheel_traj_f(1,:),wheel_traj_f(2,:),detailed_dis);
detailed_wheel_traj_r = makima(wheel_traj_r(1,:),wheel_traj_r(2,:),detailed_dis);
rp = [
    detailed_dis;
    detailed_wheel_traj_f;
    detailed_wheel_traj_r;
    gradient(detailed_wheel_traj_f)./detailed_dt
    gradient(detailed_wheel_traj_r)./detailed_dt
];

ideal_xdis_list = (0:c-1)*dt*V;
ideal_zdis_list_f = makima(rp(1,:),rp(2,:),ideal_xdis_list);
ideal_zdis_list_r = makima(rp(1,:),rp(3,:),ideal_xdis_list);
ideal_gradient_zis_f = makima(rp(1,:),rp(4,:),ideal_xdis_list);
ideal_gradient_zis_r = makima(rp(1,:),rp(5,:),ideal_xdis_list);

% ideal_omega_f = sqrt(V^2 + ideal_gradient_zis_f.^2)./r;
% ideal_omega_r = sqrt(V^2 + ideal_gradient_zis_r.^2)./r;

% movmean
ideal_omega_f = movmean(sqrt(V^2 + ideal_gradient_zis_f.^2)./r,60);
ideal_omega_r = movmean(sqrt(V^2 + ideal_gradient_zis_r.^2)./r,60);

% alpha_fk1 = (omega_fk1-omega_fk)/dt;
% alpha_rk1 = (omega_rk1-omega_rk)/dt;

% % calculate next torque
% dx_disf =current_d(5);
% dx_disr =current_d(6);
% dz_disf =current_d(7);
% dz_disr =current_d(8);

% tau_f = I_wf*(alpha_fk1 + (dz_disf*g)/(dx_disf*r*(dz_disf^2/dx_disf^2 + 1)^(1/2)));
% tau_r = I_wr*(alpha_rk1 + (dz_disr*g)/(dx_disr*r*(dz_disr^2/dx_disr^2 + 1)^(1/2)));


% if feedforward
%     u(1:2,:) = [ideal_omega_f; ideal_omega_r];
% end