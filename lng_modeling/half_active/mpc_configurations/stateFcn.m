function dxdt = stateFcn(x,u,p)
    % In a 2D environment with standard XY axis, the vehicle is a circular disc
    % (20 meters in diamater).  Two thrusts are to the left and right of the
    % center.  Tilting (theta) is defined as positive to left and negative to
    % the right (0 means robot is vertical).
    %
    % x: (1) x position of the center of gravity in m
    %    (2) y position of the center of gravity in m
    %    (3) theta (tilt with respect to the center of gravity) in radian
    %    (4) dxdt
    %    (5) dydt
    %    (6) dthetadt
    %
    % u: (1) thrust on the left, in Newton
    %    (2) thrust on the right, in Newton
    %
    % The continuous-time model is valid only if the vehicle above or at the
    % ground (y>=10).
    
    % Copyright 2023 The MathWorks, Inc.
    %% Vehicle parameter
    m_b = 960;     % [kg]      body mass
    m_wf = 40;       % [kg]      wheel mass
    m_wr = m_wf;       % [kg]      wheel mass
    k_longf = 592180*2;
    k_longr = 394790*2;
    k_sf = 30000;   % [N/m]     front spring stiffness
    k_sr = 25000;   % [N/m]     rear spring stiffness
    k_wf = 270000;   % [N/m]     tire stiffness
    k_wr = k_wf;   % [N/m]     tire stiffness
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

    g = 9.80665;

    A = [                       0,                          0,                   0,                   0,                              0, 0, 0,                        1,                          0,                   0,                   0,                              0, 0, 0;
                                0,                          0,                   0,                   0,                              0, 0, 0,                        0,                          1,                   0,                   0,                              0, 0, 0;
                                0,                          0,                   0,                   0,                              0, 0, 0,                        0,                          0,                   1,                   0,                              0, 0, 0;
                                0,                          0,                   0,                   0,                              0, 0, 0,                        0,                          0,                   0,                   1,                              0, 0, 0;
                                0,                          0,                   0,                   0,                              0, 0, 0,                        0,                          0,                   0,                   0,                              1, 0, 0;
                                0,                          0,                   0,                   0,                              0, 0, 0,                        0,                          0,                   0,                   0,                              0, 1, 0;
                                0,                          0,                   0,                   0,                              0, 0, 0,                        0,                          0,                   0,                   0,                              0, 0, 1;
         -(k_longf + k_longr)/m_b,                          0,                   0,                   0,                              0, 0, 0, -(c_longf + c_longr)/m_b,                          0,                   0,                   0,                              0, 0, 0;
                                0,         -(k_sf + k_sr)/m_b,            k_sf/m_b,            k_sr/m_b,     -(L_f*k_sf - L_r*k_sr)/m_b, 0, 0,                        0,         -(c_sf + c_sr)/m_b,            c_sf/m_b,            c_sr/m_b,     -(L_f*c_sf - L_r*c_sr)/m_b, 0, 0;
                                0,                  k_sf/m_wf, -(k_sf + k_wf)/m_wf,                   0,                (L_f*k_sf)/m_wf, 0, 0,                        0,                  c_sf/m_wf, -(c_sf + c_wf)/m_wf,                   0,                (L_f*c_sf)/m_wf, 0, 0;
                                0,                  k_sr/m_wr,                   0, -(k_sr + k_wr)/m_wr,               -(L_r*k_sr)/m_wr, 0, 0,                        0,                  c_sr/m_wr,                   0, -(c_sr + c_wr)/m_wr,               -(L_r*c_sr)/m_wr, 0, 0;
                                0, -(L_f*k_sf - L_r*k_sr)/I_b,      (L_f*k_sf)/I_b,     -(L_r*k_sr)/I_b, -(k_sf*L_f^2 + k_sr*L_r^2)/I_b, 0, 0,                        0, -(L_f*c_sf - L_r*c_sr)/I_b,      (L_f*c_sf)/I_b,     -(L_r*c_sr)/I_b, -(c_sf*L_f^2 + c_sr*L_r^2)/I_b, 0, 0;
                                0,                          0,                   0,                   0,                              0, 0, 0,                        0,                          0,                   0,                   0,                              0, 0, 0;
                                0,                          0,                   0,                   0,                              0, 0, 0,                        0,                          0,                   0,                   0,                              0, 0, 0];
    
    B = [     0,      0,       0,        0;
              0,      0,       0,        0;
              0,      0,       0,        0;
              0,      0,       0,        0;
              0,      0,       0,        0;
              0,      0,       0,        0;
              0,      0,       0,        0;
              0,      0,       0,        0;
              0,      0,   1/m_b,    1/m_b;
              0,      0, -1/m_wf,        0;
              0,      0,       0,  -1/m_wr;
              0,      0, L_f/I_b, -L_r/I_b;
         1/I_wf,      0,       0,        0;
              0, 1/I_wr,       0,        0];
    
    E = [          0,           0,         0,         0,           0,           0,         0,         0;
                   0,           0,         0,         0,           0,           0,         0,         0;
                   0,           0,         0,         0,           0,           0,         0,         0;
                   0,           0,         0,         0,           0,           0,         0,         0;
                   0,           0,         0,         0,           0,           0,         0,         0;
                   0,           0,         0,         0,           0,           0,         0,         0;
                   0,           0,         0,         0,           0,           0,         0,         0;
         k_longf/m_b, k_longr/m_b,         0,         0, c_longf/m_b, c_longr/m_b,         0,         0;
                   0,           0,         0,         0,           0,           0,         0,         0;
                   0,           0, k_wf/m_wf,         0,           0,           0, c_wf/m_wf,         0;
                   0,           0,         0, k_wr/m_wr,           0,           0,         0, c_wr/m_wr;
                   0,           0,         0,         0,           0,           0,         0,         0;
                   0,           0,         0,         0,           0,           0,         0,         0;
                   0,           0,         0,         0,           0,           0,         0,         0];
    
    G = [                        0;
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
          -sin(atan(u(11)/u(9)))/r;
         -sin(atan(u(12)/u(10)))/r];
    
    dxdt = round(A*x + B*u + E*d + G*g, 14);