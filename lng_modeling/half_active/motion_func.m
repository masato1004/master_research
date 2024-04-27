function dx = motion_func(x, u, d, g, A, B, E, G)
    % dx = Ax + Bu +Ed
    dx = round(A*x + B*u + E*d +G*g, 12);
end