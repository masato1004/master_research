%% definition of matrices
% x(k+1) = Ax(k) + Bu(k) + Gw(k)
Ap = [
    0, 0, 0, 0, 1, 0, 0, 0;
    0, 0, 0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 0, 0, 1;
    (-k_sf-k_sr)/m_b, k_sf/m_b, k_sr/m_b, (-k_sf*L_f+k_sr*L_r)/m_b, (-c_sf-c_sr)/m_b, c_sf/m_b, c_sr/m_b, (-c_sf*L_f+c_sr*L_r)/m_b;
    k_sf/m_w, (-k_sf-k_w)/m_w, 0, (k_sf*L_f)/m_w, c_sf/m_w, (-c_sf-c_w)/m_w, 0, (c_sf*L_f)/m_w;
    k_sr/m_w, 0, (-k_sr-k_w)/m_w, (-k_sr *L_r)/m_w, c_sr/m_w, 0, (-c_sr-c_w)/m_w, (-c_sr*L_r)/m_w;
    (-k_sf*L_f+k_sr*L_r)/I_b, (k_sf*L_f)/I_b, (-k_sr*L_r)/I_b, (-k_sf*L_f^2)/I_b, (-c_sf*L_f+c_sr*L_r)/I_b, (c_sf*L_f)/I_b, (-c_sr*L_r)/I_b, ((-c_sf*L_f^2)-(c_sr*L_r^2))/I_b
    ];

Bp = [
    0, 0;
    0, 0;
    0, 0;
    0, 0;
    1/m_b, 1/m_b;
    -1/m_w, 0;
    0, -1/m_w;
    L_f/I_b, -L_r/I_b
    ];

Ep = [
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    k_w/m_w, 0, c_w/m_w, 0;
    0, k_w/m_w, 0, c_w/m_w;
    0, 0, 0, 0;
    ];

C = [
    1, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 1, 0, 0, 0, 0;
    ];

% discretization
disc_func = @(tau,Mat) (-Ap\expm(Ap.*(tc-tau)))*Mat;

A = expm(Ap.*tc);
B = disc_func(tc,Bp) - disc_func(0,Bp);
E = disc_func(tc,Ep) - disc_func(0,Ep);

CA = C*A;
CB = C*B;
CE = C*E;

% dx(k) = x(k) - x(k-1)
% X(k) = phi*dx(k) + G*du(k) + Gd*dw(k)
phi = [
    eye(height(CA)), -CA;
    zeros(width(CA),height(CA)), A
    ];

G = [
    -CB;
    B
    ];

Gd = [
    -CE;
    E
    ];