%% Matrix equation of motion
function dX = motion_func(X,U,w)
    global m_b m_w k_sf k_sr k_w c_sf c_sr c_w I_b L_f L_r

    % Mddx + Cdx + Kx = Dw + Fu
    M_mat = [
        m_b 0 0 0;
        0 m_w 0 0;
        0 0 m_w 0;
        0 0 0 I_b
    ];

    C_mat = [
        c_sf+c_sr           -c_sf,          -c_sr           L_f*c_sf-L_r*c_sr;
        -c_sf               c_sf+c_w        0               -L_f*c_sf;
        -c_sr               0               c_sr+c_w        L_r*c_sr;
        L_f*c_sf-L_r*c_sr   -L_f*c_sf       L_r*c_sr        c_sf*L_f^2+c_sr*L_r^2
    ];

    K_mat = [
        k_sf+k_sr           -k_sf           -k_sr           L_f*k_sf-L_r*k_sr;
        -k_sf               k_sf+k_w        0               -L_f*k_sf;
        -k_sr               0               k_sr+k_w        L_r*k_sr;
        L_f*k_sf            -L_f*k_sf       L_r*k_sr        k_sf*L_f^2+k_sr*L_r^2
    ];

    D_mat = [
        0   0   0   0;
        k_w 0   c_w 0;
        0   k_w 0   c_w;
        0   0   0   0
    ];


    dzdiff_f = (L_f*X(8)+X(5))-w(3);
    dzdiff_r = (-L_r*X(8)+X(5))-w(4);
    F_mat = [
        -dzdiff_f       -dzdiff_r;
        dzdiff_f        0;
        0               dzdiff_r;
        -L_f*dzdiff_f   L_r*dzdiff_r
    ];


    % state space matrices
    A = [
        zeros(size(M_mat))  eye(size(M_mat));
        -M_mat\K_mat        -M_mat\C_mat    ;
    ];

    B = [
        zeros(size(F_mat));
        M_mat\F_mat
    ];

    D = [
        zeros(size(D_mat));
        M_mat\D_mat
    ];

    dX = A*X + B*U + D*w;

    return