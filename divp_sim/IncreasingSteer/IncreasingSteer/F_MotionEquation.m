function dX = F_MotionEquation(x,y,phi,L,delta)
    % states = [x; y; phi; dx; dy; dphi];

    % A = [
    %     0 0 0 cos(phi) -sin(phi) 0;
    %     0 0 0 sin(phi) -cos(phi) 0;
    %     0 0 0 0 0 1;
    %     ];

    dx = v*cos(phi);
    dy = v*sin(phi);
    dphi = v*tan(delta)/L;
    dv = 0;
    dX = [dx; dy; dphi; dv];
end