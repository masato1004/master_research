function cineq = nlmpc_config__ineqConFcn(stage,x,u,dmv,p)
    dt = 0.01;
    r = 0.55/2;           % [m]   radius of wheel

    %% set parameters through horizon
    persistent last_param last_d last_state first_states first_d current_d current_dis current_mileage_f current_mileage_r current_wheel_traj_f current_wheel_traj_r delta_x
    if isempty(last_param) | last_param(1:8) ~= p(1:8)
        last_param = p;
        current_d = p(1:8);
        current_dis = p(9:38);
        current_mileage_f = p(39:68);
        current_mileage_r = p(69:98);
        current_wheel_traj_f = [p(99:128)];
        current_wheel_traj_r = [p(129:158)];
        first_states = x;
        last_state = x;
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

    if stage == 1
        cineq = -1;
    else
        wb_constraints = 0.05;
        acc_constraints = 1;

        cineq1 = [current_d(1)-current_d(2); current_d(2)-current_d(1)] - wb_constraints;
        cineq2 = [(x(8)-last_state(8))/dt; (last_state(8)-x(8))/dt] - acc_constraints;

        cineq = [cineq1; cineq2];
    end
end