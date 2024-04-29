Ts = tc;       % control period
pHorizon = 20; % prediction horizon (control horizon is same as this)

runner = nlmpcMultistage(pHorizon, height(states), height(u));
runner.Ts = Ts;

runner.Model.StateFcn = 'mpc_configurations/stateFcn';
runner.Model.StateJacFcn = 'mpc_configurations/stateJacFcn';
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
    runner.Stages(ct).CostFcn = 'mpc_configurations/costFcn';
    % runner.Stages(ct).CostJacFcn = 'mpc_configurations/costGradientFcn';
    runner.Stages(ct).ParameterLength = height(states);
    runner.Stages(ct).IneqConFcn = "mpc_configurations/ineqConFcn";
end

runner.UseMVRate = true;

simdata = getSimulationData(runner);
validateFcns(runner,rand(height(states),1),rand(height(u),1),simdata);

options = nlmpcmoveopt;