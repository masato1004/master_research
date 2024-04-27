Ts = tc;       % control period
pHorizon = 10; % prediction horizon (control horizon is same as this)

runner = nlmpcMultistage(10, height(states), MV=[1 2 3 4], MD=[5 6 7 8 9 10 11 12]);
runner.Ts = Ts;

runner.Model.StateFcn = 'stateFcn';
runner.Model.StateJacFcn = 'stateJacFcn';

% hard constraints
lander.MV(1).Min = 0;
lander.MV(1).Max = 8;
lander.MV(2).Min = 0;
lander.MV(2).Max = 8;
lander.States(2).Min = 10;