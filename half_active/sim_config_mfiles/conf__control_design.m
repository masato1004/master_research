%% control design
% parameters
control_TL = 0:tc:T;                       % time line for control results
c_ctrl = width(control_TL);
u = zeros(2,c_ctrl);                            % input vector
du= zeros(2,c_ctrl);                            % input-differencial vector
e = zeros(height(C),c_ctrl);                    % output-error vector
dx= zeros(height(states),c_ctrl);               % states-differencial vector

X = [
    e;
    dx
    ];        % new states vector

% LQR
Q = diag([1e-06, 1e+05, 1e-03, 1e-03, 1e-03, 1e+06, 1e-03, 1e-03, 1e-03, 1e+06]);       % all_pitch
H = diag([1e-03,1e-03]);

[P, K, ~] = idare(phi, G, Q, H, [], []);   % u = -Kx
F = -K;                                      % u = Fx

% preview controler
% preview parameter
M = 43;                        % preview step
FDW = zeros(height(u), c_ctrl);
Fd = @(j) -(H + G'*P*G)\G'*(((phi+G*F)')^(j))*P*Gd;       % function for Fd(j)
Fdj = zeros(height(u), height(r_p), M+1);                 % feedforward gain vector list
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