function J = nlmpc_config__costFcn(stage,x,u,dmv,p)
    ref = p(end-13:end);
    % New Half-Car-Model cost function.
    Q = diag([1e-10 1e-3 1e-5 1e-5 1e20 1e-10 1e-10 1e20 1e05 1e-10 1e-10 1e10 1e-10 1e-10]);
    R = diag([1e-20 1e-20 1e-20 1e-20]);

    if stage == 1
        J = dmv'*R*dmv;
    elseif stage == 11
        J = (x-ref)'*2*Q*(x-ref);
    else
        J = (x-ref)'*Q*(x-ref) + dmv'*R*dmv;
    end