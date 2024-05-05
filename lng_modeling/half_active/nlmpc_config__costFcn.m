function J = nlmpc_config__costFcn(stage,x,u,dmv,e,p)
    C = logical([0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1]);
    ref = p(end-13:end);
    er = x(C)-ref(C);
    % New Half-Car-Model cost function.
    % Q = diag([1e-10 1e01 1e-3 1e-3 1e05 1e-10 1e-10 1e05 1e03 1e-10 1e-10 1e02 1e-20 1e-20]);
    % R = diag([1e-20 1e-20 1e-20 1e-20]);
    Q = diag([1e01 1e05 1e03 1e03 1e04 1e-10 1e-10]);
    R = diag([1e-06 1e-06 1e-06 1e-06]);
    dR = diag([1e-04 1e-04 1e-02 1e-02]);
    Rs = diag([1e03 1e03 1e04 1e04 1e03 1e03 1e03 1e03]);
    pHorizon = 15;

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
    J = er'*Q*er + e'*Rs*e + u'*R*u; % ((pHorizon+1 -stage)*Rs./pHorizon)