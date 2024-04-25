%% Initial conditions
Vkm_h = 15;     % [km/h]    driving velocity
Vkm_m=Vkm_h/60;
V=Vkm_m*1000/60;% [m/s]

z_wf_init = 0; % -(m_b/2+m_wf)*g/k_wf;    % [m]       front wheel displacement
z_wr_init = 0; % -(m_b/2+m_wr)*g/k_wr;    % [m]       rear wheel displacement
z_b_init = 0;  % (-(m_b/2)*g/k_sf -(m_b/2)*g/k_sr)/2 + (z_wf_init + z_wr_init)/2;
theta_init = 0; % asin(((-(m_b/2)*g/k_sf + z_wf_init) - (-(m_b/2)*g/k_sr + z_wr_init))/wb);

%% states
c       = width(TL);
x_b     = zeros(1,c);    % [m]       body longitudinal position
z_b     = zeros(1,c); z_b(1,1) = z_b_init;    % [m]       body displacement
z_wf    = zeros(1,c); z_wf(1,1) = z_wf_init;    % [m]       front wheel displacement
z_wr    = zeros(1,c); z_wr(1,1) = z_wr_init;    % [m]       rear wheel displacement
theta_b   = zeros(1,c); theta_b(1,1) = theta_init;    % [rad]     body pitch angle
theta_wf   = zeros(1,c);    % [rad]     front wheel angle
theta_wr   = zeros(1,c);    % [rad]     rear wheel angle
dx_b  = zeros(1,c); dx_b(1,1) = V;    % [m/s]     velocity
dz_b  = zeros(1,c);    % [m/s]     body heave velocity
dz_wf = zeros(1,c);    % [m/s]     front wheel heave velocity
dz_wr = zeros(1,c);    % [m/s]     rear wheel heave velocity
dtheta_b = zeros(1,c);  % [rad/s]   body pitch angular velocity
dtheta_wf   = zeros(1,c); dtheta_wf(1,1) = V/r;    % [rad]     front wheel angle
dtheta_wr   = zeros(1,c); dtheta_wr(1,1) = V/r;    % [rad]     rear wheel angle

states = [
    x_b
    z_b;
    z_wf;
    z_wr;
    theta_b;
    theta_wf;
    theta_wr;
    dx_b;
    dz_b;
    dz_wf;
    dz_wr;
    dtheta_b;
    dtheta_wf;
    dtheta_wr
    ];                    % states vector

ddx_b = zeros(1,c);     % [m/s^2]   body longitudinal acceleration
ddz_b = zeros(1,c);     % [m/s^2]   body heave acceleration
ddtheta_b = zeros(1,c);  % [rad/s^2] body pitch angular accleration

accelerations = [
    ddx_b;
    ddz_b;
    ddtheta_b
    ];                    % acceleration vector

%% disturbance
x__disf = zeros(1,c);
x__disr = zeros(1,c);
z__disf = zeros(1,c);
z__disr = zeros(1,c);
dx__disf =zeros(1,c); dx__disf(1,1) = V;
dx__disr =zeros(1,c); dx__disr(1,1) = V;
dz__disf =zeros(1,c);
dz__disr =zeros(1,c);

disturbance = [
    x__disf;
    x__disr;
    z__disf;
    z__disr;
    dx__disf;
    dx__disr;
    dz__disf;
    dz__disr
];