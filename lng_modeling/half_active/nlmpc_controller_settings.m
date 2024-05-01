Ts = tc;       % control period
pHorizon = 20; % prediction horizon (control horizon is same as this)

runner = nlmpcMultistage(pHorizon, height(states), height(u));
runner.Ts = Ts;

runner.Model.StateFcn = 'nlmpc_config__stateFcn';
% runner.Model.StateJacFcn = 'nlmpc_config__stateJacFcn';
runner.Model.ParameterLength = height(disturbance)+(pHorizon+10)*3+(pHorizon+10)*2;

% hard constraints
runner.MV(1).Min = -700;
runner.MV(1).Max =  700;
runner.MV(2).Min = -700;
runner.MV(2).Max =  700;
runner.MV(3).Min = -3000;
runner.MV(3).Max = 3000;
runner.MV(4).Min = -3000;
runner.MV(4).Max = 3000;
runner.States(8).Min = 0;


for ct=1:pHorizon+1
    runner.Stages(ct).CostFcn = 'nlmpc_config__costFcn';
    % runner.Stages(ct).CostJacFcn = 'mpc_configurations/costGradientFcn';
    runner.Stages(ct).IneqConFcn = 'nlmpc_config__ineqConFcn';
    runner.Stages(ct).ParameterLength = height(disturbance)+(pHorizon+10)*3+(pHorizon+10)*2+height(states);
    % runner.Stages(ct).ParameterLength = height(states);
end

runner.UseMVRate = true;

simdata = getSimulationData(runner);

current_mileage_f = makima(dis_total-disturbance(1,1),mileage_f-makima(dis_total,mileage_f,disturbance(1,1)),0:Ts*states(8,1):Ts*states(8,1)*(pHorizon+9));
current_mileage_r = makima(dis_total-disturbance(2,1),mileage_r-makima(dis_total,mileage_r,disturbance(2,1)),0:Ts*states(8,1):Ts*states(8,1)*(pHorizon+9));
current_wheel_traj_f = makima(wheel_traj_f(1,:),wheel_traj_f(2,:),disturbance(1,1):Ts*states(8,1):Ts*states(8,1)*(pHorizon+9)+disturbance(1,1));
current_wheel_traj_r = makima(wheel_traj_r(1,:),wheel_traj_r(2,:),disturbance(2,1):Ts*states(8,1):Ts*states(8,1)*(pHorizon+9)+disturbance(2,1));
simdata.StateFcnParameter = [disturbance(:,1);[0:Ts*states(8,1):Ts*states(8,1)*(pHorizon+9)].';current_mileage_f.';current_mileage_r.';current_wheel_traj_f.';current_wheel_traj_r.'];
simdata.StageParameter = repmat([simdata.StateFcnParameter; nlmpc_config__referenceSignal(states(:,1),u(:,1),V,theta_init,Ts)],pHorizon+1,1);
validateFcns(runner,states(:,1),u(:,1),simdata);
disp('validate done.')

% options = nlmpcmoveopt;