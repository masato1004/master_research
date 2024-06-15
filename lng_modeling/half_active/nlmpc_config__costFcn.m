function J = nlmpc_config__costFcn(stage,x,u,dmv,e,p)
    %
    % x: (1) Longitudinal Position
    %    (2) Body Vertical Displacement
    %    (3) Front Wheel Vertical Displacement
    %    (4) Rear Wheel Vertical Displacement
    %    (5) Body Pitch Angle
    %    (6) Front Wheel Angle
    %    (7) Rear Wheel Angle
    %    (8) Velocity
    %    (9) Body Vertical Velocity
    %    (10) Front Wheel Vertical Velocity
    %    (11) Rear Wheel Vertical Velocity
    %    (12) Body Pitch Angular Velocity
    %    (13) Front Wheel Angular Velocity
    %    (14) Rear Wheel Angular Velocity
    %
    % u: (1) front torque
    %    (2) rear torque
    %    (3) front sus
    %    (4) rear sus
    %
    % d: (1) longitudinal position of front wheel center
    %    (2) longitudinal position of rear wheel center
    %    (3) vertical position of front wheel center
    %    (4) vertical position of rear wheel center
    %    (5) gradient of longitudinal position of front wheel center
    %    (6) gradient of longitudinal position of rear wheel center
    %    (7) gradient of vertical position of front wheel center
    %    (8) gradient of vertical position of rear wheel center
    %
    

    % C = logical([0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1]);
    
    % New Half-Car-Model cost function.
    % R = diag([1e-20 1e-20 1e-20 1e-20]);
    % Q = diag([1e02 1e05 1e10 1e03 1e04 1e-10 1e-10]);
    % dR = diag([1e-02 1e-02 1e-02 1e-02]);
    % if stage == 1
    %     J = (pHorizon+1 - stage)*u'*R*u + (pHorizon+1 - stage)*dmv'*dR*dmv;
    % elseif stage == pHorizon+1
    %     J = (x(C)-ref)'*0.5*Q*(x(C)-ref);
    % else
    %     J = (pHorizon+1 - stage)*((x(C)-ref)'*Q*(x(C)-ref) + dmv'*dR*dmv + (pHorizon+1 - stage)*u'*R*u);
    % end
    % if any(u > 5000) 
    %     J = J*J;
    % end


    ref = p(end-13:end);
    er = x-ref;

    Q = diag([1e-10 1e02 1e-3 1e-3 1e05 1e-10 1e-10 1e02 1e02 1e-10 1e-10 1e02 1e02 1e02]);
    R = diag([1e05 1e05 1e05 1e05]);
    Rd = diag([1e05 1e05 1e05 1e05]);
    Rs = diag([1e07 1e07 1e07 1e07]);
    pHorizon = 10;

    J = er'*0.5*Q*er + u'*0.5*R*u + e'*0.5*Rs*e + dmv'*0.5*Rd*dmv; % ((pHorizon+1 -stage)*Rs./pHorizon) 