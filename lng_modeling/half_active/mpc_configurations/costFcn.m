function J = costFcn(stage,x,u,dmv,p)
    % New Half-Car-Model cost function.
    Q = diag([1e-10 1e-3 1e-5 1e-5 1e10 1e-10 1e-10 1e05 1e05 1e-10 1e-10 1e10 1e-10 1e-10]);
    R = diag([1e00 1e00 1e00 1e00]);

    if stage == 1
        J = dmv'*R*dmv;
    elseif stage == 11
        J = (x-p)'*2*Q*(x-p);
    else
        J = (x-p)'*Q*(x-p) + dmv'*R*dmv;
    end