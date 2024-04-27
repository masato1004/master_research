Ts = tc;       % control period
pHorizon = 10; % prediction horizon (control horizon is same as this)

runner = nlmpcMultistage(10, height(states), MV=[1 2 3 4], MD=[5 6 7 8 9 10 11 12]);
runner.Ts = Ts;

runner.Model.StateFcn = 'mpc_configurations/stateFcn';
runner.Model.StateJacFcn = 'mpc_configurations/stateJacFcn';

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
    runner.Stages(ct).ParameterLength = 6;
end

runner.UseMVRate = true;