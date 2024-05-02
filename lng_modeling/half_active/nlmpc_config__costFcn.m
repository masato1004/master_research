function J = nlmpc_config__costFcn(stage,x,u,dmv,p)
    C = logical([0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1]);
    ref = p(end-13:end);
    ref = ref(C);
    % New Half-Car-Model cost function.
    % Q = diag([1e-10 1e01 1e-3 1e-3 1e05 1e-10 1e-10 1e05 1e03 1e-10 1e-10 1e02 1e-20 1e-20]);
    % R = diag([1e-20 1e-20 1e-20 1e-20]);
    Q = diag([1e01 1e06 1e06 1e03 1e02 1e-01 1e-01]);
    R = diag([1e-04 1e-04 1e-08 1e-08]);
    if u > 1000000
        u
    end

    if stage == 1
        J = dmv'*R*dmv;
    elseif stage == 11
        J = (x(C)-ref)'*10*Q*(x(C)-ref);
    else
        J = (x(C)-ref)'*Q*(x(C)-ref) + dmv'*R*dmv;
    end