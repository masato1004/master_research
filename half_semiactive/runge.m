function x_1 = runge(x, v, u, d, A, B, E, dt)

    k1 = dt*v;
    a1 = motion_func([x;v], u, d, A, B, E);
    l1 = dt*a1(end-height(v)+1:end);

    k2 = dt*(v+l1/2);
    a2 = motion_func([x;v]+[k1;l1]./2, u, d, A, B, E);
    l2 = dt*a2(end-height(v)+1:end);

    k3 = dt*(v+l2/2);
    a3 = motion_func([x;v]+[k2;l2]./2, u, d, A, B, E);
    l3 = dt*a3(end-height(v)+1:end);

    k4 = dt*(v+l3/2);
    a4 = motion_func([x;v]+[k3;l3]./2, u, d, A, B, E);
    l4 = dt*a4(end-height(v)+1:end);

    rk = [
        k1, k2, k3, k4;
        l1, l2, l3, l4
        ];
    const = [
        1;
        2;
        2;
        1
        ];
    x_1 = [x;v] + (rk*const/6);
end