Ts = tc;       % control period
pHorizon = 20; % prediction horizon (control horizon is same as this)

runner = nlmpcMultistage(pHorizon, height(states), MV=[1:height(u)], MD=[1:height(disturbance)]);
runner.Ts = Ts;

runner.Model.StateFcn = 'mpc_configurations/stateFcn';
runner.Model.StateJacFcn = 'mpc_configurations/stateJacFcn';
runner.Model.ParameterLength = (pHorizon+10)*2;

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
    runner.Stages(ct).CostFcn = 'mpc_configurations/costFcn';
    % runner.Stages(ct).CostJacFcn = 'mpc_configurations/costGradientFcn';
    runner.Stages(ct).ParameterLength = height(states);
end

runner.UseMVRate = true;

simdata = getSimulationData(runner);
validateFcns(runner,rand(height(states),1),rand(height(u),1),simdata);