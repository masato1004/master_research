function cineq = nlmpc_config__ineqConFcn(stage,x,u,dmv,e,p)
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
    pHorizon = 20;

    persistent A B E disc_func
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
        
        % discretization
        % disc_func = @(tau,Mat) (-pinv(A)*expm(A.*(dt-tau)))*Mat;

        % A = expm(A.*dt);
        % B = disc_func(dt,B) - disc_func(0,B);
        % E = disc_func(dt,E) - disc_func(0,E);
    end


    %% set parameters through horizon
    persistent ref param_flag last_param last_state last_d first_states first_d current_d current_dis current_mileage_f current_mileage_r delta_x current_wheel_traj_f current_wheel_traj_r
    
    % if stage == 1
    %     cineq = -1;
    %     last_param = p;
    %     current_d = p(1:8);
    %     current_dis = p(9:8+pHorizon+10);
    %     current_mileage_f = p(9+pHorizon+10:8+(pHorizon+10)*2);
    %     current_mileage_r = p(9+(pHorizon+10)*2:8+(pHorizon+10)*3);
    %     current_wheel_traj_f = [p(9+(pHorizon+10)*3:8+(pHorizon+10)*4)];
    %     current_wheel_traj_r = [p(9+(pHorizon+10)*4:8+(pHorizon+10)*5)];
    %     first_states = x;
    %     first_d = current_d;
    %     last_d = current_d;
    %     last_state = x;
    % else
    if isempty(last_param) | any(param_flag ~= p(end-14))
        cineq = [-1; -1];
        ref = p(end-13:end);
        param_flag = p(end-14);
        last_param = p;
        current_d = p(1:8);
        current_dis = p(9:8+pHorizon+10);
        current_mileage_f = p(9+pHorizon+10:8+(pHorizon+10)*2);
        current_mileage_r = p(9+(pHorizon+10)*2:8+(pHorizon+10)*3);
        current_wheel_traj_f = [p(9+(pHorizon+10)*3:8+(pHorizon+10)*4)];
        current_wheel_traj_r = [p(9+(pHorizon+10)*4:8+(pHorizon+10)*5)];
        first_states = x;
        first_d = current_d;
        last_d = current_d;
        last_state = x;
        % first_d = current_d;
        % last_d = current_d;
    elseif any(round(x,10) ~= round(first_states,10))
        delta_x = x - first_states;
        front_wheel_rotation = delta_x(6)*r;
        rear_wheel_rotation  = delta_x(7)*r;

        current_d(1) = first_d(1) + makima(current_mileage_f,current_dis,front_wheel_rotation);  % x_disf
        current_d(2) = first_d(2) + makima(current_mileage_r,current_dis,rear_wheel_rotation);  % x_disr
        % current_d(3) = round(makima(current_dis,current_wheel_traj_f,current_d(1)-first_d(1)),5);
        % current_d(4) = round(makima(current_dis,current_wheel_traj_r,current_d(2)-first_d(2)),5);
        % current_d(5) = diff([last_d(1,1), current_d(1,1)])/(dt*pHorizon);
        % current_d(6) = diff([last_d(2,1), current_d(2,1)])/(dt*pHorizon);
        % current_d(7) = diff([round(last_d(3,1),5), current_d(3,1)])/(dt*pHorizon);
        % current_d(8) = diff([round(last_d(4,1),5), current_d(4,1)])/(dt*pHorizon);

        % current_acc = (x - last_state)/dt;
        last_state = x;

        wb_constraints = 0.01;
        acc_constraints = 1.5;
        pitch_constraints = 0.2;
        lng_constraints = 0.05;
        velocity_constraints = 0.05;
    
        cineq1 = ((current_d(1)-current_d(2))^2 - wb_constraints^2 + e(1));
        cineq2 = ((x(8)-ref(8))^2 - velocity_constraints^2 + e(2));
        % cineq2 = (current_acc(8)^2 - acc_constraints^2 - e(2));
        % cineq3 = stage*0.3*(next_state(5)+0.01257407)^2 - pitch_constraints^2;
        % cineq4 = stage*3*((x(1)-current_d(1))^2 - lng_constraints^2);
        % cineq5 = stage*3*((x(1)-current_d(2))^2 - lng_constraints^2);
    
        cineq = [cineq1; cineq2];
    else
        last_state = x;
        cineq = [-1; -1];
    end

    % G = [                                     0;
    %                                             0;
    %                                             0;
    %                                             0;
    %                                             0;
    %                                             0;
    %                                             0;
    %                                             0;
    %                                             -1;
    %                                             -1;
    %                                             -1;
    %                                             0;
    %     -sin(atan(current_d(7)/current_d(5)))/r;
    %     -sin(atan(current_d(8)/current_d(6)))/r];

    % G = disc_func(dt,G) - disc_func(0,G);

    % if any(u~=0)
    %     disp(u)
    % end
    % dxdt = A*x + E*current_d + G*g;
    % dxdt = A*x + B*u + E*current_d + G*g;
    % next_state = x + dxdt*dt;
    % last_d = current_d;
    % if any(cineq == nan)
    %     cineq = 0;
    % end
        % if any(cineq > 0)
        %     stage
        %     next_state(8)
        %     x(8)
        % end
        % last_state = x;
    % end
end