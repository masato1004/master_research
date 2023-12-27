% main_sim.m
clear
close all;

%% Motion Equation
% motion_equation_konishi

%% Parameter
% Human
% m1=56.6;
m1=56.6;
m2=10.8;
k1=7.7*10^4;
% k1=7.7*10;
d1=1.6*10^3;
l1=0.38;%文献参考（＝下腿）

% System
m3=40;
% m3=200;
% k3=1000;
% d3=100;
k3=1000;
d3=100;
l3=3;

g=9.81;

% 
% A = double(sym(Amat));
% B = double(sym(Bmat));
% C = double(sym(Cmat));
% D = double(sym(Dmat));

%% 状態空間モデルを作成し、状態、入力、出力に名前を付ける
% motion_equation.mを実行して、各行列を定義
A = [ 
[           0,                    0,            1,                    0]
[           0,                    0,            0,                    1]
[      -k1/m1,                k1/m1,       -d1/m1,                d1/m1]
[k1/(m2 + m3), -(k1 + k3)/(m2 + m3), d1/(m2 + m3), -(d1 + d3)/(m2 + m3)]
];
 
 
B = [ 
          0
          0
          0
1/(m2 + m3)
];
 
% E = [
%            0
%            0
%            0
% -1/(m2 + m3)
% ];

E = [
           0
           0
       -1/m1
-1/(m2 + m3)
];

C = [
     1     0     0     0
     0     1     0     0
     0     0     1     0
     0     0     0     1
];

D = [];

states = {'x1' 'x2' 'dx1' 'dx2'};
inputs = {'u'};
outputs = {'x1' 'x2' 'dx1' 'dx2'};

sys_mimo = ss(A,B,C,D,'statename',states,...
                     'inputname',inputs,...
                     'outputname',outputs)
%% Simulation
dt = 0.001;
t = 0.0:dt:10.0;      % シミュレーション時間

x0 = [0;0;-5.0;-5.0];       % 初期状態
X = x0;
dX = zeros(size(A,1),1);
U = 0;
K = [0 0 0 0];
%     U = -K*X;

for i=1:length(t)

    if i*dt < 0.01
        F = 6000;
        % F = [0;0;-555;-6000];
    else
        F = 0;
    end

    % Save data
    x(i,:) = X;
    u(i,1) = U;
    f(i,:) = F;
    dx(i,:) = dX;

    % Controller
%     U = -K*X;
    U = 0;

    % 4th Runge-Kutta method
    X1 = X;        b1 = dt*(A*X1 + B*U + E*F);
    X2 = X+b1/2;   b2 = dt*(A*X2 + B*U + E*F);
    X3 = X+b2/2;   b3 = dt*(A*X3 + B*U + E*F);
    X4 = X+b3;     b4 = dt*(A*X4 + B*U + E*F);
    X = X+(b1+2*b2+2*b3+b4)/6;

    dX = A*X + B*U;
end

% figure(1)
% subplot(211), plot(t,x), grid on; xlabel('time[s]'), ylabel('X'); legend(states);
% subplot(212), plot(t,f), grid on; xlabel('time[s]'), ylabel('F[N]'); legend('F');

figure(1)
subplot(411), plot(t,x(:,1:2)), grid on; xlabel('time[s]'), ylabel('X'); legend('x_1','x_2');
subplot(412), plot(t,x(:,3:4)), grid on; xlabel('time[s]'), ylabel('dX'); legend('dx_1','dx_2');
subplot(413), plot(t,dx(:,3:4)), grid on; xlabel('time[s]'), ylabel('ddX'); legend('ddx_1','ddx_2');
subplot(414), plot(t,f), grid on; xlabel('time[s]'), ylabel('F[N]'); legend('F');
