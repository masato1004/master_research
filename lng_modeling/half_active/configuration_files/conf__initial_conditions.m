%% Initial conditions
Vkm_h = 10;     % [km/h]    driving velocity
Vkm_m=Vkm_h/60;
V=Vkm_m*1000/60;% [m/s]

z_wf_init = -(m_b/2+m_wf)*g/k_wf;    % [m]       front wheel displacement
z_wr_init = -(m_b/2+m_wr)*g/k_wr;    % [m]       rear wheel displacement
z_b_init = (-(m_b/2)*g/k_sf -(m_b/2)*g/k_sr)/2 + (z_wf_init + z_wr_init)/2;
theta_init = asin(((-(m_b/2)*g/k_sf + z_wf_init) - (-(m_b/2)*g/k_sr + z_wr_init))/wb);

c       = width(TL);
x_b     = zeros(1,c);    % [m]       body longitudinal position
z_b     = zeros(1,c); z_b(1,1) = z_b_init;    % [m]       body displacement
z_wf    = zeros(1,c); z_wf(1,1) = z_wf_init;    % [m]       front wheel displacement
z_wr    = zeros(1,c); z_wr(1,1) = z_wr_init;    % [m]       rear wheel displacement
theta   = zeros(1,c); theta(1,1) = theta_init;    % [rad]     body pitch angle
x_bDot  = zeros(1,c); x_bDot(1,1) = V;    % [m/s]     velocity
z_bDot  = zeros(1,c);    % [m/s]     body heave velocity
z_wfDot = zeros(1,c);    % [m/s]     front wheel heave velocity
z_wrDot = zeros(1,c);    % [m/s]     rear wheel heave velocity
theta_Dot = zeros(1,c);  % [rad/s]   body pitch angular velocity

states = [
    x_b
    z_b;
    z_wf;
    z_wr;
    theta;
    x_bDot;
    z_bDot;
    z_wfDot;
    z_wrDot;
    theta_Dot
    ];                    % states vector

z_b2Dot = zeros(1,c);     % [m/s^2]   body heave acceleration
theta_2Dot = zeros(1,c);  % [rad/s^2] body pitch angular accleration

accelerations = [
    z_b2Dot;
    theta_2Dot
    ];                    % acceleration vector