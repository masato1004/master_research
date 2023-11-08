function dx = motion_func(x, u, A, B)
    % dx = Ax + Bu +Ed
    dx = A*x + B*u;
end