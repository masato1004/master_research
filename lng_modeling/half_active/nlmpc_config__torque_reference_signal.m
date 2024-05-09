function [tau_f tau_r] = nlmpc_config__torque_reference_signal(omega_fk, omega_rk, Vref, state_param, first_d, pHorizon, dt, r, I_wf, I_wr, control)
    % plant parameters
    g = 9.80665;
    % r = 0.55/2;
    % m_wf = 40;       % [kg]      wheel mass
    % m_wr = m_wf;       % [kg]      wheel mass
    % I_wf = (m_wf*r^2)/2;     % [kgm^2]   wheel inertia moment
    % I_wr = (m_wr*r^2)/2;     % [kgm^2]   wheel inertia moment

    % load current parameters
    param_flag = state_param(end);
    last_param = state_param;
    current_d = state_param(1:8);
    current_dis = state_param(9:8+pHorizon+10);
    current_mileage_f = state_param(9+pHorizon+10:8+(pHorizon+10)*2);
    current_mileage_r = state_param(9+(pHorizon+10)*2:8+(pHorizon+10)*3);
    current_wheel_traj_f = [state_param(9+(pHorizon+10)*3:8+(pHorizon+10)*4)];
    current_wheel_traj_r = [state_param(9+(pHorizon+10)*4:8+(pHorizon+10)*5)];
    if control == "_nlmpc_"

        % next longitudinal position
        next_lng_pos_f = ((omega_fk*r + Vref)/2)*dt;
        next_lng_pos_r = ((omega_rk*r + Vref)/2)*dt;

        % calculate next disturbance
        next_d = zeros(8,1);
        next_d(1) = first_d(1) + makima(current_mileage_f,current_dis,next_lng_pos_f);  % x_disf
        next_d(2) = first_d(2) + makima(current_mileage_r,current_dis,next_lng_pos_r);  % x_disr
        next_d(3) = makima(current_dis,current_wheel_traj_f,next_d(1)-first_d(1));
        next_d(4) = makima(current_dis,current_wheel_traj_r,next_d(2)-first_d(2));

        duration_length_f = dt*Vref*r;
        temp_duration_f = [(next_d(1)-first_d(1))-duration_length_f,(next_d(1)-first_d(1)),(next_d(1)-first_d(1))+duration_length_f];
        temp_displacment_f = round(makima(current_dis,current_wheel_traj_f,temp_duration_f)*1e05)*1e-05;
        temp_gradient_f = [sum(diff(temp_duration_f)./(dt))/2;sum(diff(temp_displacment_f)./(dt))/2];
        duration_length_r = dt*Vref*r;
        temp_duration_r = [(next_d(2)-first_d(2))-duration_length_r,(next_d(2)-first_d(2)),(next_d(2)-first_d(2))+duration_length_r];
        temp_displacment_r = round(makima(current_dis,current_wheel_traj_r,temp_duration_r)*1e05)*1e-05;
        temp_gradient_r = [sum(diff(temp_duration_r)./(dt))/2;sum(diff(temp_displacment_r)./(dt))/2];

        next_d(5) = (omega_fk*r)*(temp_gradient_f(1)^2/sum(temp_gradient_f.^2));
        next_d(6) = (omega_rk*r)*(temp_gradient_r(1)^2/sum(temp_gradient_r.^2));
        next_d(7) = (omega_fk*r)*(temp_gradient_f(2)^2/sum(temp_gradient_f.^2));
        next_d(8) = (omega_rk*r)*(temp_gradient_r(2)^2/sum(temp_gradient_r.^2));
        
        % calculate ideal rotational acceleration
        omega_fk1 = sqrt(Vref^2 + temp_gradient_f(2,1)^2)/r;
        omega_rk1 = sqrt(Vref^2 + temp_gradient_r(2,1)^2)/r;
        
        alpha_fk1 = (omega_fk1-omega_fk)/dt;
        alpha_rk1 = (omega_rk1-omega_rk)/dt;
        
        dx_disf =(current_d(5)+next_d(5))/2;
        dx_disr =(current_d(6)+next_d(6))/2;
        dz_disf =(current_d(7)+next_d(7))/2;
        dz_disr =(current_d(8)+next_d(8))/2;
    elseif control == "_feedforward_"
        omega_fk1 = Vref(1);
        omega_rk1 = Vref(2);

        alpha_fk1 = (omega_fk1-omega_fk)/dt;
        alpha_rk1 = (omega_rk1-omega_rk)/dt;

        % calculate next torque
        dx_disf =current_d(5);
        dx_disr =current_d(6);
        dz_disf =current_d(7);
        dz_disr =current_d(8);
    end

    tau_f = I_wf*(alpha_fk1 + (dz_disf*g)/(dx_disf*r*(dz_disf^2/dx_disf^2 + 1)^(1/2)));
    tau_r = I_wr*(alpha_rk1 + (dz_disr*g)/(dx_disr*r*(dz_disr^2/dx_disr^2 + 1)^(1/2)));
end