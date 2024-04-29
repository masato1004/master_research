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
runner.MV(3).Min = -2000;
runner.MV(3).Max = 2000;
runner.MV(4).Min = -2000;
runner.MV(4).Max = 2000;


for ct=1:pHorizon+1
    runner.Stages(ct).CostFcn = 'nlmpc_config__costFcn';
    % runner.Stages(ct).CostJacFcn = 'mpc_configurations/costGradientFcn';
    runner.Stages(ct).IneqConFcn = 'nlmpc_config__ineqConFcn';
    runner.Stages(ct).ParameterLength = height(disturbance)+(pHorizon+10)*3+(pHorizon+10)*2+height(states);
    % runner.Stages(ct).ParameterLength = height(states);
end

runner.UseMVRate = true;

simdata = getSimulationData(runner);
size(simdata.StageParameter)
% validateFcns(runner,rand(height(states),1),rand(height(u),1),simdata);

options = nlmpcmoveopt;