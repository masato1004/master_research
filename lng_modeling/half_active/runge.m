function dx = runge(x, u, d, g, A, B, E, Gmat, dt,wheel_traj_f,wheel_traj_r,mileage_f,mileage_r,dis_total)
    x1 = x; d1 = d;
    dx_disf =d1(5,1);
    dx_disr =d1(6,1);
    dz_disf =d1(7,1);
    dz_disr =d1(8,1);
    G  = double(subs(Gmat));
    b1 = dt*motion_func(x1, u, d1, g, A, B, E, G);

    x2 = x+b1/2; d2 = find_disturbance(wheel_traj_f,wheel_traj_r,mileage_f,mileage_r,dis_total,d1,x2,x1,dt);
    dx_disf =d2(5,1);
    dx_disr =d2(6,1);
    dz_disf =d2(7,1);
    dz_disr =d2(8,1);
    G  = double(subs(Gmat));
    b2 = dt*motion_func(x2, u, d2, g, A, B, E, G);
    
    x3 = x+b2/2; d3 = find_disturbance(wheel_traj_f,wheel_traj_r,mileage_f,mileage_r,dis_total,d1,x3,x1,dt);
    dx_disf =d3(5,1);
    dx_disr =d3(6,1);
    dz_disf =d3(7,1);
    dz_disr =d3(8,1);
    G  = double(subs(Gmat));
    b3 = dt*motion_func(x3, u, d3, g, A, B, E, G);

    x4 = x+b3; d4 = find_disturbance(wheel_traj_f,wheel_traj_r,mileage_f,mileage_r,dis_total,d1,x4,x1,dt);
    dx_disf =d4(5,1);
    dx_disr =d4(6,1);
    dz_disf =d4(7,1);
    dz_disr =d4(8,1);
    G  = double(subs(Gmat));
    b4 = dt*motion_func(x4, u, d4, g, A, B, E, G);

    dx = x + (b1 + 2*b2 + 2*b3 + b4)/6;
end

function d = find_disturbance(wheel_traj_f,wheel_traj_r,mileage_f,mileage_r,dis_total,current_d,new_states,current_states,dt)
    d(1,1) = makima(mileage_f-makima(dis_total,mileage_f,current_d(1,1)),dis_total,r*(new_states(6,1)-current_states(6,1)));  % x_disf
    d(2,1) = makima(mileage_r-makima(dis_total,mileage_r,current_d(2,1)),dis_total,r*(new_states(7,1)-current_states(7,1)));  % x_disr
    d(3,1) = makima(wheel_traj_f(1,:),wheel_traj_f(2,:),d(1,1));                                                                % z_disf
    d(4,1) = makima(wheel_traj_f(1,:),wheel_traj_r(2,:),d(2,1));                                                                % z_disr
    d(5,1) = diff([current_d(1,1),d(1,1)])/dt;                                         % dx_disf
    d(6,1) = diff([current_d(2,1),d(2,1)])/dt;                                         % dx_disr
    d(7,1) = diff([current_d(3,1),d(3,1)])/dt;                                         % dz_disf
    d(8,1) = diff([current_d(4,1),d(4,1)])/dt;                                         % dz_disr
end