%% Vehicle parameter
m_b = 960;     % [kg]      body mass
m_w = 40;       % [kg]      wheel mass
k_sf = 30000;   % [N/m]     front spring stiffness
k_sr = 25000;   % [N/m]     rear spring stiffness
% k_sf = 13000;   % [N/m]     front spring stiffness
% k_sr = 17000;   % [N/m]     rear spring stiffness
k_w = 270000;   % [N/m]     tire stiffness
% k_w = 60000;   % [N/m]     tire stiffness
c_sf = 7000;    % [N/(m/s)] front damping
c_sr = 6000;    % [N/(m/s)] rear damping
c_w = 1000;       % [N/(m/s)] tire damping
% c_w = 2000;       % [N/(m/s)] tire damping
I_b = 600;      % [kgm^2]   inertia moment
wb = 2.45;     % wheel base
L_f = wb * 23/55;      % [m]       front length
L_r = wb * 32/55;      % [m]       rear length

cam_fwheel_dis = 0.895;

Vkm_h = 20;     % [km/h]    driving velocity
Vkm_m=Vkm_h/60;
V=Vkm_m*1000/60;% [m/s]
