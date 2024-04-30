function dxdt = nlmpc_config__stateFcn(x,u,p)
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
    dt = 0.01;
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

    persistent A B E disc_func
    if isempty(A)
        Ac = [                      0,                          0,                   0,                   0,                              0, 0, 0,                        1,                          0,                   0,                   0,                              0, 0, 0;
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
        
        Bc = [    0,      0,       0,        0;
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
        
        Ec = [         0,           0,         0,         0,           0,           0,         0,         0;
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
        disc_func = @(tau,Mat) (-pinv(Ac)*expm(Ac.*(dt-tau)))*Mat;

        A = expm(Ac.*dt);
        B = disc_func(dt,Bc) - disc_func(0,Bc);
        E = disc_func(dt,Ec) - disc_func(0,Ec);
    end
    

    %% set parameters through horizon
    persistent last_param last_d first_states first_d current_d current_dis current_mileage_f current_mileage_r current_wheel_traj_f current_wheel_traj_r delta_x
    
    if isempty(last_param) | last_param(1:8) ~= p(1:8)
        last_param = p;
        current_d = p(1:8);
        current_dis = p(9:38);
        current_mileage_f = p(39:68);
        current_mileage_r = p(69:98);
        current_wheel_traj_f = [p(99:128)];
        current_wheel_traj_r = [p(129:158)];
        first_states = x;
        first_d = current_d;
        last_d = current_d;
    else
        delta_x = x - first_states;
        front_wheel_rotation = delta_x(6)*r;
        rear_wheel_rotation  = delta_x(7)*r;
        
        current_d(1) = makima(current_mileage_f,current_dis,front_wheel_rotation);  % x_disf
        current_d(2) = makima(current_mileage_r,current_dis,rear_wheel_rotation);  % x_disr
        current_d(3) = makima(current_dis,current_wheel_traj_f,current_d(1)-first_d(1));
        current_d(4) = makima(current_dis,current_wheel_traj_r,current_d(2)-first_d(2));
        current_d(5) = diff([last_d(1,1), current_d(1,1)])/dt;
        current_d(6) = diff([last_d(2,1), current_d(2,1)])/dt;
        current_d(7) = diff([last_d(3,1), current_d(3,1)])/dt;
        current_d(8) = diff([last_d(4,1), current_d(4,1)])/dt;

        last_d = current_d;
    end

    
    Gc = [                                    0;
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
        -sin(atan(current_d(7)/current_d(5)))/r;
        -sin(atan(current_d(8)/current_d(6)))/r];

    Gc(isnan(Gc)) = 0;

    G = disc_func(dt,Gc) - disc_func(0,Gc);
    
    dxdt = A*x + B*u + E*current_d + G*g;
    if sum(isnan(current_d)) ~= 0
        disp(current_d)
    end
    dxdt(isnan(dxdt)) = 0;

