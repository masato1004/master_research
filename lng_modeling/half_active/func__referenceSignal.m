function ref = func__referenceSignal(x,u,init,Ts,pHorizon,wheel_traj_f,wheel_traj_r,Vref,r)
    horizon_tl = 0:Ts:pHorizon*Ts;
    horizon_len = length(horizon_tl);
    [ideal_omega_f, ideal_omega_r] = func__rotvref_maker(Ts,pHorizon,horizon_len,Vref,wheel_traj_f,wheel_traj_r,r);

    ref = [
        x(1) + x(8)*horizon_tl;
        repmat(init(2),1,horizon_len);
        repmat(init(3),1,horizon_len);
        repmat(init(4),1,horizon_len);
        repmat(init(5),1,horizon_len);
        x(6) + cumtrapz(ideal_omega_r*Ts);
        x(7) + cumtrapz(ideal_omega_r*Ts);
        repmat(Vref,1,horizon_len);
        zeros(1,horizon_len);
        zeros(1,horizon_len);
        zeros(1,horizon_len);
        zeros(1,horizon_len);
        ideal_omega_f;
        ideal_omega_r
    ];
end

function [ideal_omega_f, ideal_omega_r] = func__rotvref_maker(Ts,pHorizon,horizon_len,Vref,wheel_traj_f,wheel_traj_r,r)
    %% Feedforward settings
    wheel_traj_f(2,wheel_traj_f(2,:) < 1e-6) = 0;
    wheel_traj_r(2,wheel_traj_r(2,:) < 1e-6) = 0;
    detailed_dt = 1e-04;
    detailed_dis = 0:Vref*detailed_dt:Vref*Ts*(pHorizon+10);
    detailed_wheel_traj_f = makima(wheel_traj_f(1,:)-wheel_traj_f(1,1),wheel_traj_f(2,:),detailed_dis);
    detailed_wheel_traj_r = makima(wheel_traj_r(1,:)-wheel_traj_f(2,1),wheel_traj_r(2,:),detailed_dis);
    rp = [
        detailed_dis;
        detailed_wheel_traj_f;
        detailed_wheel_traj_r;
        gradient(detailed_wheel_traj_f)./detailed_dt
        gradient(detailed_wheel_traj_r)./detailed_dt
    ];

    ideal_xdis_list = (0:horizon_len-1)*Ts*Vref;
    % ideal_zdis_list_f = makima(rp(1,:),rp(2,:),ideal_xdis_list);
    % ideal_zdis_list_r = makima(rp(1,:),rp(3,:),ideal_xdis_list);
    ideal_gradient_zis_f = makima(rp(1,:),rp(4,:),ideal_xdis_list);
    ideal_gradient_zis_r = makima(rp(1,:),rp(5,:),ideal_xdis_list);

    ideal_omega_f = sqrt(Vref^2 + ideal_gradient_zis_f.^2)./r;
    ideal_omega_r = sqrt(Vref^2 + ideal_gradient_zis_r.^2)./r;
end