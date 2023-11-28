%%%      Motion Equation (Linear)    %%%
clear

syms x_1 dx_1 ddx_1
syms x_2 dx_2 ddx_2
syms x_4 dx_4 ddx_4

syms m1 m2 m3 m4  
syms k1 k3 k4
syms d1 d3 d4 

%%% Energy %%%
syms KE UE DE
%%% Common %%%
syms t g u

q = [ x_1 x_2 x_4 ].';
dq = [ dx_1 dx_2 dx_4 ].';
ddq = [ ddx_1 ddx_2 ddx_4 ].';

% Xb = [ x_b  z_b].';
% Xf = [ x_tf  z_b+lf*sin(q(7,1)) ].';
% Xr = [ x_tr  z_b-lr*sin(q(7,1)) ].';
% 
% dXb = jacobian(Xb,q)*dq;
% dXf = jacobian(Xf,q)*dq;
% dXr = jacobian(Xr,q)*dq;

KE = simplify( 1/2*m1*dq(1,1)^2 + 1/2*(m2+m3)*dq(2,1)^2 + 1/2*m4*dq(3,1)^2 );
KE = simplify(KE);

PE = simplify( 1/2*k1*(q(1,1)-q(2,1))^2 + 1/2*k3*q(2,1)^2  + 1/2*k4*(q(2,1)-q(3,1))^2 );
PE = simplify(PE);

DE = simplify( 1/2*d1*(dq(1,1)-dq(2,1))^2 + 1/2*d3*dq(2,1)^2 + 1/2*d4*(dq(2,1)-dq(3,1))^2 );
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

B1 = [ 0  1  0 ].';

% E1 = [ 0  -1 ].';
E1 = [ -1  -1  0].';

disp(' ')
disp(' M*ddq + C*dq + K*q = B*u + E*F ')
disp(' ')

% M
% C3
% K1
% B1
% E

% K = sym('a', size(M));
% C = sym('a', size(M));

for i=1:size(M,1)
    K(:,i) = diff(K1, q(i,1));
    C(:,i) = diff(C3, dq(i,1));
end

%%% Amat*X + Bmat*u = Emat*F %%%
%%% Ymat = Cmat*X + Dmat*u %%%
Amat = [zeros(size(M,1)) eye(size(M,1)); -M\K  -M\C]
Bmat = [zeros(size(M,1),1);M\B1]
Emat = [zeros(size(M,1),1);M\E1]
Cmat = eye(size(Amat,1))
Dmat = []