function [ideal_omega_f, ideal_omega_r] = func__rotvref_maker()
    %% Feedforward settings
    detailed_dt = 1e-05;
    detailed_dis = 0:V*detailed_dt:max_distance;
    detailed_wheel_traj_f = makima(wheel_traj_f(1,:),wheel_traj_f(2,:),detailed_dis);
    detailed_wheel_traj_r = makima(wheel_traj_r(1,:),wheel_traj_r(2,:),detailed_dis);
    rp = [
        detailed_dis;
        detailed_wheel_traj_f;
        detailed_wheel_traj_r;
        gradient(detailed_wheel_traj_f)./detailed_dt
        gradient(detailed_wheel_traj_r)./detailed_dt
    ];

    ideal_xdis_list = (0:c-1)*dt*V;
    ideal_zdis_list_f = makima(rp(1,:),rp(2,:),ideal_xdis_list);
    ideal_zdis_list_r = makima(rp(1,:),rp(3,:),ideal_xdis_list);
    ideal_gradient_zis_f = makima(rp(1,:),rp(4,:),ideal_xdis_list);
    ideal_gradient_zis_r = makima(rp(1,:),rp(5,:),ideal_xdis_list);

    ideal_omega_f = sqrt(V^2 + ideal_gradient_zis_f.^2)./r;
    ideal_omega_r = sqrt(V^2 + ideal_gradient_zis_r.^2)./r;
end