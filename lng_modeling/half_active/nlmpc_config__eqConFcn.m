function ceq = nlmpc_config__eqConFcn(stage,x,u,dmv,p)
    %
    % x: (1) Longitudinal Position
    %    (2) Body Vertical Displacement
    %    (3) Front Wheel Vertical Displacement
    %    (4) Rear Wheel Vertical Displacement
    %    (5) Body Pitch Angle
    %    (6) Front Wheel Angle
    %    (7) Rear Wheel Angle
    %    (8) Velocity
    %    (9) Body Vertical Velocity
    %    (10) Front Wheel Vertical Velocity
    %    (11) Rear Wheel Vertical Velocity
    %    (12) Body Pitch Angular Velocity
    %    (13) Front Wheel Angular Velocity
    %    (14) Rear Wheel Angular Velocity
    %
    % u: (1) front torque
    %    (2) rear torque
    %    (3) front sus
    %    (4) rear sus
    %
    % d: (1) longitudinal position of front wheel center
    %    (2) longitudinal position of rear wheel center
    %    (3) vertical position of front wheel center
    %    (4) vertical position of rear wheel center
    %    (5) gradient of longitudinal position of front wheel center
    %    (6) gradient of longitudinal position of rear wheel center
    %    (7) gradient of vertical position of front wheel center
    %    (8) gradient of vertical position of rear wheel center
    %
    
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
    pHorizon = 5;

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
        disc_func = @(tau,Mat) (-pinv(A)*expm(A.*(dt-tau)))*Mat;

        % A = expm(A.*dt);
        % B = disc_func(dt,B) - disc_func(0,B);
        % E = disc_func(dt,E) - disc_func(0,E);
    end


    %% set parameters through horizon
    persistent ref param_flag last_param last_state last_d first_states first_d current_d current_dis current_mileage_f current_mileage_r delta_x current_wheel_traj_f current_wheel_traj_r
    
    % if stage == 1
    %     ceq = -1;
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
    if isempty(last_param)
        ceq = [0; 0; 0];
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
    elseif any(param_flag ~= p(end-14))
        ceq = [0; 0; 0];
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
    elseif any(round(x*1e10) ~= round(first_states*1e10)) 
        delta_x = x - first_states;
        front_wheel_rotation = delta_x(6)*r;
        rear_wheel_rotation  = delta_x(7)*r;

        current_d(1) = first_d(1) + makima(current_mileage_f,current_dis,front_wheel_rotation);  % x_disf
        current_d(2) = first_d(2) + makima(current_mileage_r,current_dis,rear_wheel_rotation);  % x_disr
        current_d(3) = round(makima(current_dis,current_wheel_traj_f,current_d(1)-first_d(1))*1e05)*1e-05;
        current_d(4) = round(makima(current_dis,current_wheel_traj_r,current_d(2)-first_d(2))*1e05)*1e-05;
        
        duration_length_f = dt*x(13)*r/2;
        temp_duration_f = [(current_d(1)-first_d(1))-duration_length_f,(current_d(1)-first_d(1)),(current_d(1)-first_d(1))+duration_length_f];
        temp_displacment_f = round(makima(current_dis,current_wheel_traj_f,temp_duration_f)*1e05)*1e-05;
        temp_gradient_f = [sum(diff(temp_duration_f))/2;sum(diff(temp_displacment_f))/2];
        duration_length_r = dt*x(14)*r/2;
        temp_duration_r = [(current_d(2)-first_d(2))-duration_length_r,(current_d(2)-first_d(2)),(current_d(2)-first_d(2))+duration_length_r];
        temp_displacment_r = round(makima(current_dis,current_wheel_traj_r,temp_duration_r)*1e05)*1e-05;
        temp_gradient_r = [sum(diff(temp_duration_r))/2;sum(diff(temp_displacment_r))/2];

        current_d(5) = (x(13)*r)*(temp_gradient_f(1)^2/sum(temp_gradient_f.^2));
        current_d(6) = (x(14)*r)*(temp_gradient_r(1)^2/sum(temp_gradient_r.^2));
        current_d(7) = (x(13)*r)*(temp_gradient_f(2)^2/sum(temp_gradient_f.^2));
        current_d(8) = (x(14)*r)*(temp_gradient_r(2)^2/sum(temp_gradient_r.^2));
        % current_acc = (x - last_state)/dt;
        last_state = x;

        wb_constraints = 0;
        acc_constraints = 1.5;
        pitch_constraints = deg2rad(0.2);
        lng_constraints = 0.05;
        velocity_constraints = 0.05;
    
        % ceq1 = (current_d(5)-ref(8));
        % ceq2 = (current_d(6)-ref(8));
        % ceq1 = (current_d(1) - x(1));
        % ceq2 = (current_d(2) - x(1));
        ceq1 = x(5)-ref(5);
        % ceq2 = x(8)-ref(8);
        ceq2 = 0;
        % ceq2 = ((x(8)-ref(8))^2 - velocity_constraints^2);
        % ceq2 = (current_acc(8)^2 - acc_constraints^2 - e(2));
        % ceq3 = stage*0.3*(next_state(5)+0.01257407)^2 - pitch_constraints^2;
        % ceq4 = stage*3*((x(1)-current_d(1))^2 - lng_constraints^2);
        % ceq5 = stage*3*((x(1)-current_d(2))^2 - lng_constraints^2);
    
        ceq = [ceq1; ceq2];
    else
        last_state = x;
        ceq = [0; 0; 0];
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
    % if any(ceq == nan)
    %     ceq = 0;
    % end
        % if any(ceq > 0)
        %     stage
        %     next_state(8)
        %     x(8)
        % end
        % last_state = x;
    % end
end