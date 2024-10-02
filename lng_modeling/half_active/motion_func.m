function dx = motion_func(x, u, d, g, A, B, E, G)
    % dx = Ax + Bu +Ed
    dx = A*x + B*u + E*d +G*g;
end