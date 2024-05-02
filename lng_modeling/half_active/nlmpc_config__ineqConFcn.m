function cineq = nlmpc_config__ineqConFcn(stage,x,u,dmv,p)
    dt = 0.01;     % control period
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

    persistent A 
    if isempty(A)
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
        
        % B = [     0,      0,       0,        0;
        %           0,      0,       0,        0;
        %           0,      0,       0,        0;
        %           0,      0,       0,        0;
        %           0,      0,       0,        0;
        %           0,      0,       0,        0;
        %           0,      0,       0,        0;
        %           0,      0,       0,        0;
        %           0,      0,   1/m_b,    1/m_b;
        %           0,      0, -1/m_wf,        0;
        %           0,      0,       0,  -1/m_wr;
        %           0,      0, L_f/I_b, -L_r/I_b;
        %      1/I_wf,      0,       0,        0;
        %           0, 1/I_wr,       0,        0];
        
        % E = [          0,           0,         0,         0,           0,           0,         0,         0;
        %                0,           0,         0,         0,           0,           0,         0,         0;
        %                0,           0,         0,         0,           0,           0,         0,         0;
        %                0,           0,         0,         0,           0,           0,         0,         0;
        %                0,           0,         0,         0,           0,           0,         0,         0;
        %                0,           0,         0,         0,           0,           0,         0,         0;
        %                0,           0,         0,         0,           0,           0,         0,         0;
        %      k_longf/m_b, k_longr/m_b,         0,         0, c_longf/m_b, c_longr/m_b,         0,         0;
        %                0,           0,         0,         0,           0,           0,         0,         0;
        %                0,           0, k_wf/m_wf,         0,           0,           0, c_wf/m_wf,         0;
        %                0,           0,         0, k_wr/m_wr,           0,           0,         0, c_wr/m_wr;
        %                0,           0,         0,         0,           0,           0,         0,         0;
        %                0,           0,         0,         0,           0,           0,         0,         0;
        %                0,           0,         0,         0,           0,           0,         0,         0];
        
        % discretization
        % disc_func = @(tau,Mat) (-A\expm(A.*(dt-tau)))*Mat;

        A = expm(A.*dt);
        % B = disc_func(dt,B) - disc_func(0,B);
        % E = disc_func(dt,E) - disc_func(0,E);
    end


    %% set parameters through horizon
    persistent last_param last_d first_states first_d current_d current_dis current_mileage_f current_mileage_r current_wheel_traj_f current_wheel_traj_r delta_x
    
    if stage == 1
        cineq = -1;
    else
        if isempty(last_param) | last_param(1:8) ~= p(1:8)
            last_param = p;
            current_d = p(1:8);
            current_dis = p(9:38);
            current_mileage_f = p(39:68);
            current_mileage_r = p(69:98);
            current_wheel_traj_f = [p(99:128)];
            current_wheel_traj_r = [p(129:158)];
            first_states = x;
            % last_state = x;
            first_d = current_d;
            last_d = current_d;
        else
            next_state = A*x;
            delta_x = next_state - first_states;
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

        wb_constraints = 0.05;
        acc_constraints = 1;
        pitch_constraints = 0.2;

        cineq1 = [current_d(1)-current_d(2); current_d(2)-current_d(1)] - wb_constraints;
        cineq2 = [(next_state(8)-x(8))/dt; (x(8)-next_state(8))/dt] - acc_constraints;
        cineq3 = [next_state(5)+0.01257407; -next_state(5)-0.01257407] - pitch_constraints;

        cineq = [cineq1; cineq2; cineq3];
        % last_state = x;
    end
end