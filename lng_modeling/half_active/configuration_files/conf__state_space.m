%% Load system matrices from the file
syms x_b dx_b ddx_b        % body longitudinal
syms z_b dz_b ddz_b        % body vertical
syms z_wf dz_wf ddz_wf     % fornt wheel vertical
syms z_wr dz_wr ddz_wr     % rear wheel vertical
syms theta dtheta ddtheta  % body rotation

syms x_disf dx_disf ddx_disf  % road longitudinal
syms x_disr dx_disr ddx_disr  % road longitudinal
syms z_disf dz_disf ddz_disf  % road displacement
syms z_disr dz_disr ddz_disr  % road displacement

syms m_b m_wf m_wr I_b
syms k_sf k_sr k_wf k_wr k_longf k_longr
syms c_sf c_sr c_wf c_wr c_longf c_longr

syms alpha_f alpha_r sus_f sus_r

syms t g L_f L_r S r

% run('../../../lagrange/motion_equation_inoue_M01.m')  % only for the first time or when it was changed
load('system_matrices.mat');

%% Vehicle parameter
m_b = 960;     % [kg]      body mass
m_wf = 40;       % [kg]      wheel mass
m_wr = m_wf;       % [kg]      wheel mass
k_longf = 592180*2;
k_longr = 394790*2;
k_sf = 30000;   % [N/m]     front spring stiffness
k_sr = 25000;   % [N/m]     rear spring stiffness
% k_sf = 13000;   % [N/m]     front spring stiffness
% k_sr = 17000;   % [N/m]     rear spring stiffness
k_wf = 270000;   % [N/m]     tire stiffness
k_wr = k_wf;   % [N/m]     tire stiffness
% k_w = 60000;   % [N/m]     tire stiffness
c_longf = 2360*2;
c_longr = 2117*2;
c_sf = 7000;    % [N/(m/s)] front damping
c_sr = 6000;    % [N/(m/s)] rear damping
c_wf = 1000;       % [N/(m/s)] tire damping
c_wr = c_wf;       % [N/(m/s)] tire damping
wb = 2.45;     % wheel base
L_f = wb * 23/55;      % [m]       front length
L_r = wb * 32/55;      % [m]       rear length
r = 0.55/2;           % [m]   radius of wheel
I_b = m_b*(wb/2)^2;      % [kgm^2]   body inertia moment
I_wf = (m_wf*r^2)/2;     % [kgm^2]   wheel inertia moment
I_wr = (m_wr*r^2)/2;     % [kgm^2]   wheel inertia moment
S = 0.2;     % slip ratio

% environmental parameter
g = 9.80665;

% matrices for gravity term
Gmat = [
    0;
    0;
    0;
    0;
    0;
    0;
    0;
    0;
    -1;
    -1;
    -1;
    0;
    0;
    0
    % -sin(atan(dz_disf/dx_disf))/r;
    % -sin(atan(dz_disr/dx_disr))/r
];

%% Load parameters into matrices
Ap = double(subs(Amat));
Bp = subs(Bmat);
Ep = subs(Emat);
C  = double(subs(Cmat));

% -sin(atan(dz_disf/dx_disf))^2 - sin(atan(dz_disr/dx_disr))^2
% - g*sin(atan(dz_disf/dx_disf))
% - g*sin(atan(dz_disr/dx_disr))
% ((1-S)*r*(tau_r/I_wr) )*cos(atan(dz_disr/dx_disr)) 

%% Discretization
% disc_func = @(tau,Mat) (-Ap\expm(Ap.*(tc-tau)))*Mat;

% A = expm(Ap.*tc);
% B = disc_func(tc,Bp) - disc_func(0,Bp);
% E = disc_func(tc,Ep) - disc_func(0,Ep);

% CA = C*A;
% CB = C*B;
% CE = C*E;

% % dx(k) = x(k) - x(k-1)
% % X(k) = phi*dx(k) + G*du(k) + Gd*dw(k)
% phi = [
%     eye(height(CA)), -CA;
%     zeros(width(CA),height(CA)), A
%     ];

% G = [
%     -CB;
%     B
%     ];

% Gd = [
%     -CE;
%     E
%     ];
