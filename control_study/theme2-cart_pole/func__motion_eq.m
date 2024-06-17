function dx = func__motion_eq(x, u, d, r, A, B, E, H)
    % dx = Ax + Bu
    dx = A*x + B*u;
    if sum(size(E)) ~= 0
        dx = dx + E*d;
    end
    if sum(size(H)) ~= 0
        dx = dx + H*r;
    end
end