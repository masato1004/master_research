function dx = runge_wheel(r,x_b,dtheta_w,dt)
    x1 = x_b;      b1 = x1+r*dtheta_w*dt;
    x2 = x_b+b1/2; b2 = x2+r*dtheta_w*dt;
    x3 = x_b+b2/2; b3 = x3+r*dtheta_w*dt;
    x4 = x_b+b3;   b4 = x4+r*dtheta_w*dt;
    dx = x_b + (b1 + 2*b2 + 2*b3 + b4)/6;
end