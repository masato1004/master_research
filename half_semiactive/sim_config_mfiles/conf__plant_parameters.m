%% Vehicle specifications
global m_b m_w k_sf k_sr k_w c_sf c_sr c_w I_b L_f L_r

m_b = 1000;     % [kg]      body mass
m_w = 30;       % [kg]      wheel mass
k_sf = 33000;   % [N/m]     front spring stiffness
k_sr = 37000;   % [N/m]     rear spring stiffness
% k_sf = 13000;   % [N/m]     front spring stiffness
% k_sr = 17000;   % [N/m]     rear spring stiffness
k_w = 270000;   % [N/m]     tire stiffness
% k_w = 60000;   % [N/m]     tire stiffness
c_sf = 3300;    % [N/(m/s)] front damping
c_sr = 3250;    % [N/(m/s)] rear damping
c_w = 1000;       % [N/(m/s)] tire damping
% c_w = 2000;       % [N/(m/s)] tire damping
I_b = 750;      % [kgm^2]   inertia moment
L_f = 1.4;      % [m]       front length
L_r = 1.4;      % [m]       rear length

cam_fwheel_dis = 0.895;

%% Vehicle parameter
Vkm_h = 50;     % [km/h]    driving velocity
Vkm_m=Vkm_h/60;
V=Vkm_m*1000/60;% [m/s]