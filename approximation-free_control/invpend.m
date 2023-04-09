function xdot = invpend(x,K,Kff,m,M,l,g)
    % INVERTED_PENDULUM Simulation of a cart-pole system with
    % Approximation-Free control
    
    % Unpack state variables
    theta = x(1);
    phi = x(2);
    theta_dot = x(3);
    phi_dot = x(4);
    
    % Define feedback and feedforward control
    u = -K*x + Kff*x;
    
    % Calculate state derivatives
    c = cos(theta);
    s = sin(theta);
    d1 = m*(l^2) + M;
    d = d1 - (m*(l^2)*(c^2))/d1;
    xdot = zeros(4,1);
    xdot(1) = theta_dot;
    xdot(2) = phi_dot;
    xdot(3) = (1/d)*((m*(l^2))*g*s*c - m*(l^2)*s*(phi_dot^2) + u*c);
    xdot(4) = (1/d)*(-m*g*c*s + m*l*(phi^2)*s - u);

end
