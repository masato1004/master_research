function dx = runge_wheel(dtheta_w,torque,dt)
    x1 = dtheta_w;      b1 = dt*(x1+torque*dt);
    x2 = dtheta_w+b1/2; b2 = dt*(x2+torque*dt);
    x3 = dtheta_w+b2/2; b3 = dt*(x3+torque*dt);
    x4 = dtheta_w+b3;   b4 = dt*(x4+torque*dt);
    dx = dtheta_w + (b1 + 2*b2 + 2*b3 + b4)/6;
end