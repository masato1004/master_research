%%%      Motion Equation (Linear)    %%%
clear

syms z_b dz_b ddz_b        % body vertical
syms theta_b dtheta_b ddtheta_b  % body rotation

syms z_disf dz_disf ddz_disf  % road displacement
syms z_disr dz_disr ddz_disr  % road displacement

syms m_b I_b
syms k_sf k_sr
syms c_sf c_sr

syms sus_f sus_r

%%% Energy %%%
syms KE PE DE
%%% Common %%%
syms L_f L_r

m_list   = [ m_b I_b ].';
k_list   = [ k_sf k_sr ].';
c_list   = [ c_sf c_sr ].';

q   = [ z_b theta_b ].';
dq  = [ dz_b dtheta_b ].';
ddq = [ ddz_b ddtheta_b ].';

w  = [ z_disf z_disr dz_disf dz_disr ].';

u  = [ sus_f sus_r ].';


KE = simplify( sum((1/2)*(m_list.*(dq.^2))) );
KE = simplify(KE);

PE = simplify( 1/2*(k_list(1,1)*(q(1,1)+L_f*q(2,1)-w(1,1))^2 + k_list(2,1)*(q(1,1)-L_r*q(2,1)-w(2,1))^2) );
PE = simplify(PE);

DE = simplify( 1/2*(c_list(1,1)*(dq(1,1)+L_f*dq(2,1)-w(3,1))^2 + c_list(2,1)*(dq(1,1)-L_r*dq(2,1)-w(4,1))^2) );
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

input_acc = [
    sus_f + sus_r;
    L_f*sus_f - L_r*sus_r;
];
input_acc = simplify(input_acc);

disp(' ')
disp(' M*ddq + C*dq + K*q = B*u + E*w ')
disp(' ')

for i=1:size(M,1)
    K(:,i) = diff(K1, q(i,1));
    C(:,i) = diff(C3, dq(i,1));
end
for j=1:size(w,1)
    E1(:,j) = diff(-K1, w(j,1));
    E2(:,j) = diff(-C3, w(j,1));
end
for k=1:size(u,1)
    B(:,k) = diff(input_acc, u(k,1));
end

E = simplify( E1 + E2 );

%%% Amat*X + Bmat*u = Emat*F %%%
%%% Ymat = Cmat*X + Dmat*u %%%
Amat = [zeros(size(M,1)) eye(size(M,1)); -M\K  -M\C];
Bmat = [zeros(size(M,1),width(B));M\B];
Emat = [zeros(size(M,1),width(E));M\E];
Cmat = diag([1,1,1,1]);
% save('system_matrices', 'Amat', 'Bmat', 'Emat', 'Cmat');