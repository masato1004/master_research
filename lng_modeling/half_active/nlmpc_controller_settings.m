Ts = tc;       % control period
pHorizon = 10; % prediction horizon (control horizon is same as this)

runner = nlmpcMultistage(pHorizon, height(states), height(u));
runner.Ts = Ts;
runner.Model.IsContinuousTime = true;

solver = "cgmres";
runner.Optimization.Solver = solver;

if solver == "fmincon"
    runner.Optimization.SolverOptions.Algorithm = 'interior-point';
    runner.Optimization.SolverOptions.SpecifyObjectiveGradient = true;
    runner.Optimization.SolverOptions.SpecifyConstraintGradient = true;
    % runner.Optimization.SolverOptions.MaxFunctionEvaluations = 5000;
    runner.Optimization.SolverOptions.ConstraintTolerance = 1e-02; % defalt 1e-06
    runner.Optimization.SolverOptions.OptimalityTolerance = 1e-03; % defalt 1e-06
    runner.Optimization.SolverOptions.StepTolerance = 1e-06; % defalt 1e-10
    runner.Optimization.SolverOptions.FunctionTolerance = 1e-05; % defalt 1e-06
    runner.Optimization.SolverOptions.UseParallel = true;
    runner.Optimization.SolverOptions.FunValCheck = 'on';
    % runner.Optimization.SolverOptions.PlotFcn = 'optimplotfval';
    % runner.Optimization.SolverOptions.PlotFcn = 'optimplotfvalconstr';
elseif solver == "cgmres"
    % Set the solver parameters.
    runner.Optimization.SolverOptions.StabilizationParameter = 1/(runner.Ts);
    runner.Optimization.SolverOptions.MaxIterations = 10;
    runner.Optimization.SolverOptions.Restart = 20;
    runner.Optimization.SolverOptions.BarrierParameter = 5e02;
    runner.Optimization.SolverOptions.TerminationTolerance = 1e-07;
    runner.Optimization.SolverOptions.FiniteDifferenceStepSize = 1e-09;
end
runner.Optimization.PerturbationRatio = 1e-07;
runner.Optimization.UseSuboptimalSolution = true;

runner.Model.StateFcn = 'nlmpc_config__stateFcn';
runner.Model.StateJacFcn = 'nlmpc_config__stateFcnJacobian';
runner.Model.ParameterLength = 1+height(disturbance)+(pHorizon+10)*3+(pHorizon+10)*2;

% hard constraints
runner.UseMVRate = true;

runner.ManipulatedVariables(1).Min = -700;
runner.ManipulatedVariables(1).Max =  700;
runner.ManipulatedVariables(2).Min = -700;
runner.ManipulatedVariables(2).Max =  700;
runner.ManipulatedVariables(3).Min = -3000;
runner.ManipulatedVariables(3).Max = 3000;
runner.ManipulatedVariables(4).Min = -3000;
runner.ManipulatedVariables(4).Max = 3000;

% runner.ManipulatedVariables(1).RateMin = -600/2;
% runner.ManipulatedVariables(1).RateMax =  600/2;
% runner.ManipulatedVariables(2).RateMin = -600/2;
% runner.ManipulatedVariables(2).RateMax =  600/2;
% runner.ManipulatedVariables(3).RateMin = -500;
% runner.ManipulatedVariables(3).RateMax = 500;
% runner.ManipulatedVariables(4).RateMin = -500;
% runner.ManipulatedVariables(4).RateMax = 500;

% runner.States(8).Min = 0;

% runner.States(5).Min = deg2rad(rad2deg(theta_init)-0.2);
% runner.States(5).Max = deg2rad(rad2deg(theta_init)+0.2);

sp_length = runner.Model.ParameterLength+height(states);
for ct=1:pHorizon+1
    runner.Stages(ct).CostFcn = 'nlmpc_config__costFcn';
    runner.Stages(ct).CostJacFcn = 'nlmpc_config__costFcnJacobian';
    runner.Stages(ct).IneqConFcn = 'nlmpc_config__ineqConFcn';
    % runner.Stages(ct).IneqConJacFcn = 'nlmpc_config__ineqConFcnJacobian';
    runner.Stages(ct).ParameterLength = sp_length;
    runner.Stages(ct).SlackVariableLength = 2;
    % if ct ~= pHorizon+1
    %     runner.Stages(ct).EqConFcn = 'nlmpc_config__eqConFcn';
    % end
end

runner = generateJacobianFunction(runner,"state");
runner = generateJacobianFunction(runner,"cost");
% runner = generateJacobianFunction(runner,"ineqcon");
% runner = generateJacobianFunction(runner,"eqcon");

simdata = getSimulationData(runner);

current_mileage_f = makima(dis_total-disturbance(1,1),mileage_f-makima(dis_total,mileage_f,disturbance(1,1)),0:Ts*states(8,1):Ts*states(8,1)*(pHorizon+9));
current_mileage_r = makima(dis_total-disturbance(2,1),mileage_r-makima(dis_total,mileage_r,disturbance(2,1)),0:Ts*states(8,1):Ts*states(8,1)*(pHorizon+9));
current_wheel_traj_f = makima(wheel_traj_f(1,:),wheel_traj_f(2,:),disturbance(1,1):Ts*states(8,1):Ts*states(8,1)*(pHorizon+9)+disturbance(1,1));
current_wheel_traj_r = makima(wheel_traj_r(1,:),wheel_traj_r(2,:),disturbance(2,1):Ts*states(8,1):Ts*states(8,1)*(pHorizon+9)+disturbance(2,1));
detailed_wheel_traj_f = [disturbance(1,1):dt*states(8,1):dt*states(8,1)*(pHorizon+9)+disturbance(1,1);makima(wheel_traj_f(1,:),wheel_traj_f(2,:),disturbance(1,1):dt*states(8,1):dt*states(8,1)*(pHorizon+9)+disturbance(1,1))];
detailed_wheel_traj_r = [disturbance(2,1):dt*states(8,1):dt*states(8,1)*(pHorizon+9)+disturbance(2,1);makima(wheel_traj_r(1,:),wheel_traj_r(2,:),disturbance(2,1):dt*states(8,1):dt*states(8,1)*(pHorizon+9)+disturbance(2,1))];
reference = func__referenceSignal(states(:,1),u(:,1),states(:,1),Ts,pHorizon,detailed_wheel_traj_f,detailed_wheel_traj_r,V,r);
simdata.StateFcnParameter = [disturbance(:,1);[0:Ts*states(8,1):Ts*states(8,1)*(pHorizon+9)].';current_mileage_f.';current_mileage_r.';current_wheel_traj_f.';current_wheel_traj_r.';1];
sp = zeros(sp_length*(pHorizon+1),1);
for i=1:pHorizon+1
    sp(sp_length*(i-1)+1:(sp_length)*i,1) = [simdata.StateFcnParameter; reference(:,i)];
end
simdata.StageParameter = sp;
% simdata.StageParameter = repmat([simdata.StateFcnParameter; ],pHorizon+1,1);
% simdata.InitialGuess = [];
validateFcns(runner,states(:,1),u(:,1),simdata);
disp('validate done.')

% [coredata, onlinedata] = getCodeGenerationData(runner, states(:,1), u(:,1), StateFcnParameter=simdata.StateFcnParameter, StageParameter=simdata.StageParameter);
% % !del ./nlmpcControllerMEX.mexw64;
% buildMEX(runner, "nlmpcControllerMEX", coredata, onlinedata);
% 
% options = nlmpcmoveopt;