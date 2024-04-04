%%%      Motion Equation (Linear)    %%%
clear

syms x_b dx_b ddx_b        % body longitudinal
syms z_b dz_b ddz_b        % body vertical
syms z_wf dz_wf ddz_wf     % fornt wheel vertical
syms z_wr dz_wr ddz_wr     % rear wheel vertical
syms theta dtheta ddtheta  % body rotation

syms x_disf dx_disf ddx_disf  % road longitudinal
syms x_disr dx_disr ddx_disr  % road longitudinal
syms z_disf dz_disf ddz_disf  % road displacement
syms z_disr dz_disr ddz_disr  % road displacement

syms m_b m_wf m_wr I_b
syms k_sf k_sr k_wf k_wr k_longf k_longr
syms c_sf c_sr c_wf c_wr c_longf c_longr

%%% Energy %%%
syms KE PE DE
%%% Common %%%
syms t g u Lf Lr

m_list   = [ m_b m_b m_wf m_wr I_b ].';
k_list   = [ k_longf k_longr k_sf k_sr k_wf k_wr ].';
c_list   = [ c_longf c_longr c_sf c_sr c_wf c_wr ].';

q   = [ x_b z_b z_wf z_wr theta ].';
dq  = [ dx_b dz_b dz_wf dz_wr dtheta ].';
ddq = [ ddx_b ddz_b ddz_wf ddz_wr ddtheta ].';

% Xb = [ x_b  z_b].';
% Xf = [ x_tf  z_b+lf*sin(q(7,1)) ].';
% Xr = [ x_tr  z_b-lr*sin(q(7,1)) ].';
% 
% dXb = jacobian(Xb,q)*dq;
% dXf = jacobian(Xf,q)*dq;
% dXr = jacobian(Xr,q)*dq;

KE = simplify( sum(1/2*(m_list.*dq.^2)) );
KE = simplify(KE);

PE = simplify( 1/2*(k_list(1,1)*(q(1,1)-x_disf)^2 + k_list(2,1)*(q(1,1)-x_disr)^2 + k_list(3,1)*(q(2,1)+q(5,1)*Lf-q(3,1))^2 + k_list(4,1)*(q(2,1)-q(5,1)*Lr-q(4,1))^2 + k_list(5,1)*(q(3,1)-z_disf)^2 + k_list(6,1)*(q(4,1)-z_disr)^2) );
PE = simplify(PE);

DE = simplify( 1/2*(c_list(1,1)*(dq(1,1)-dx_disf)^2 + c_list(2,1)*(dq(1,1)-dx_disr)^2 + c_list(3,1)*(dq(2,1)+dq(5,1)*Lf-dq(3,1))^2 + c_list(4,1)*(dq(2,1)-dq(5,1)*Lr-dq(4,1))^2 + c_list(5,1)*(dq(3,1)-dz_disf)^2 + c_list(6,1)*(dq(4,1)-dz_disr)^2) );
DE = simplify(DE);

%%% M(q) ddq + C(q,dq) dq + K(q) q = B u %%%
n = max(size(q));

M = simplify(jacobian(jacobian(KE,dq),dq));

for i = 1:n
    C1(i,1) = 0*g;
    for j = 1:n
        for k = 1:n
            C1(i,1) = C1(i,1) + (diff(M(i,j),q(k,1))-diff(M(j,k),q(i,1))/2)*dq(j,1)*dq(k,1);
        end
    end
end   

C2 = simplify(jacobian(DE,dq).');

C3 = simplify( C1 + C2 );

K1 = simplify(jacobian(PE,q).');

% x_b
% z_b
% z_wf
% z_wr
% theta

% tau_f
% tau_r
% sus_f
% sus_r

B1 = [  ];

% E1 = [ 0  -1 ].';
E1 = [ -1  -1 ].';

disp(' ')
disp(' M*ddq + C*dq + K*q = B*u + E*F ')
disp(' ')

% M
% C3
% K1
% B1
% E

for i=1:size(M,1)
    K(:,i) = diff(K1, q(i,1));
    C(:,i) = diff(C3, dq(i,1));
end

%%% Amat*X + Bmat*u = Emat*F %%%
%%% Ymat = Cmat*X + Dmat*u %%%
% Amat = [zeros(size(M,1)) eye(size(M,1)); -M\K  -M\C]
% Bmat = [zeros(size(M,1),1);M\B1]
% Emat = [zeros(size(M,1),1);M\E1]
% Cmat = eye(size(Amat,1))
% Dmat = []