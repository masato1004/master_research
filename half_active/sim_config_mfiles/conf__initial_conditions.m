%% Initial conditions
c       = width(TL);
z_b     = zeros(1,c);    % [m]       body displacement
z_wf    = zeros(1,c);    % [m]       front wheel displacement
z_wr    = zeros(1,c);    % [m]       rear wheel displacement
theta   = zeros(1,c);    % [rad]     body pitch angle
z_bDot  = zeros(1,c);    % [m/s]     body heave velocity
z_wfDot = zeros(1,c);    % [m/s]     front wheel heave velocity
z_wrDot = zeros(1,c);    % [m/s]     rear wheel heave velocity
theta_Dot = zeros(1,c);  % [rad/s]   body pitch angular velocity

states = [
    z_b;
    z_wf;
    z_wr;
    theta;
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