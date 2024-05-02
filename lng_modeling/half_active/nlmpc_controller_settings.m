Ts = tc;       % control period
pHorizon = 10; % prediction horizon (control horizon is same as this)

runner = nlmpcMultistage(pHorizon, height(states), height(u));
runner.Ts = Ts;
runner.Model.IsContinuousTime = true;

solver = "cgmres";
runner.Optimization.Solver = solver;

if solver == "fmincon"
    runner.Optimization.SolverOptions.Algorithm = 'interior-point';
    runner.Optimization.SolverOptions.SpecifyObjectiveGradient = false;
    runner.Optimization.SolverOptions.SpecifyConstraintGradient = false;
    runner.Optimization.SolverOptions.ConstraintTolerance = 1e-2;
elseif solver == "cgmres"
    % Set the solver parameters.
    runner.Optimization.SolverOptions.StabilizationParameter = 1/runner.Ts;
    runner.Optimization.SolverOptions.MaxIterations = 15;
    runner.Optimization.SolverOptions.Restart = 5;
    runner.Optimization.SolverOptions.BarrierParameter = 1e03;
    runner.Optimization.SolverOptions.TerminationTolerance = 1e-2;
end
% runner.Optimization.SolverOptions.FunValCheck = 'on';
% runner.Optimization.SolverOptions.PlotFcn = 'optimplotfvalconstr';

runner.Model.StateFcn = 'nlmpc_config__stateFcn';
runner.Model.StateJacFcn = 'nlmpc_config__stateJacFcn';
runner.Model.ParameterLength = 1+height(disturbance)+(pHorizon+10)*3+(pHorizon+10)*2;

% hard constraints
runner.UseMVRate = true;

runner.ManipulatedVariables(1).Min = -1000;
runner.ManipulatedVariables(1).Max =  1000;
runner.ManipulatedVariables(2).Min = -1000;
runner.ManipulatedVariables(2).Max =  1000;
runner.ManipulatedVariables(3).Min = -4000;
runner.ManipulatedVariables(3).Max = 4000;
runner.ManipulatedVariables(4).Min = -4000;
runner.ManipulatedVariables(4).Max = 4000;

runner.ManipulatedVariables(1).RateMin = -150;
runner.ManipulatedVariables(1).RateMax =  150;
runner.ManipulatedVariables(2).RateMin = -150;
runner.ManipulatedVariables(2).RateMax =  150;
runner.ManipulatedVariables(3).RateMin = -500;
runner.ManipulatedVariables(3).RateMax = 500;
runner.ManipulatedVariables(4).RateMin = -500;
runner.ManipulatedVariables(4).RateMax = 500;

runner.States(8).Min = 0;

% runner.States(5).Min = -0.2;
% runner.States(5).Max = 0.2;


for ct=1:pHorizon+1
    runner.Stages(ct).CostFcn = 'nlmpc_config__costFcn';
    % runner.Stages(ct).CostJacFcn = 'mpc_configurations/costGradientFcn';
    runner.Stages(ct).IneqConFcn = 'nlmpc_config__ineqConFcn';
    runner.Stages(ct).ParameterLength = 1+height(disturbance)+(pHorizon+10)*3+(pHorizon+10)*2+height(states);
    % runner.Stages(ct).ParameterLength = height(states);
end


simdata = getSimulationData(runner);

current_mileage_f = makima(dis_total-disturbance(1,1),mileage_f-makima(dis_total,mileage_f,disturbance(1,1)),0:Ts*states(8,1):Ts*states(8,1)*(pHorizon+9));
current_mileage_r = makima(dis_total-disturbance(2,1),mileage_r-makima(dis_total,mileage_r,disturbance(2,1)),0:Ts*states(8,1):Ts*states(8,1)*(pHorizon+9));
current_wheel_traj_f = makima(wheel_traj_f(1,:),wheel_traj_f(2,:),disturbance(1,1):Ts*states(8,1):Ts*states(8,1)*(pHorizon+9)+disturbance(1,1));
current_wheel_traj_r = makima(wheel_traj_r(1,:),wheel_traj_r(2,:),disturbance(2,1):Ts*states(8,1):Ts*states(8,1)*(pHorizon+9)+disturbance(2,1));
simdata.StateFcnParameter = [disturbance(:,1);[0:Ts*states(8,1):Ts*states(8,1)*(pHorizon+9)].';current_mileage_f.';current_mileage_r.';current_wheel_traj_f.';current_wheel_traj_r.';1];
simdata.StageParameter = repmat([simdata.StateFcnParameter; nlmpc_config__referenceSignal(states(:,1),u(:,1),V,theta_init,Ts)],pHorizon+1,1);
simdata.InitialGuess = [];
validateFcns(runner,states(:,1),u(:,1),simdata);
disp('validate done.')

% options = nlmpcmoveopt;