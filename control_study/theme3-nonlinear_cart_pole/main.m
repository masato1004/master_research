close all;
clear;

%% Define simlation condition シミュレーション条件
% loop parameters
T = 20;                 % シミュレーション時間
dt = 1e-02;             % シミュレーション時間幅
TL = 0:dt:T;            % 時間リスト作成
TL_width = width(TL);   % 時間リストの長さ取得（リストの要素数）
control_dt = dt;        % 制御周期（デフォルト：シミュレーション時間幅）
% control_dt = dt*10;    % 制御周期（シミュレーション周期の100倍）

% initial value
x0 = -0.5;                % カート初期位置
theta0 = 0.05;             % 振子初期角度
dx0 = 0;                % カート初期速度
dtheta0 = 0;            % 振子初期角速度

% controller
passive = false;        % パッシブシミュレーション
LQR = false;            % LQR
servo = true;           % 積分型最適サーボ系

%% Model Definition モデルの定義
% define state space: dxdt = Ax(t) + Bu(t) + Ed(t), y(t) = Cx(t) + Du(t)
% 倒立振子の状態空間モデル(MIMOモデル)
syms M m J L mu zeta xi g p1 p2

Amat=[
    0       0       1       0;
    0       0       0       1;
    0       0   -zeta       0;
    0    p1*g p1*zeta     -p2
    ];

Bmat=[
         0;
         0;
        xi;
    -p1*xi
    ];

C=diag([1,1,0,0]);
C(sum(C,2)==0,:)=[];  % eliminate rows filled with 0 不要な行の削除

D=zeros(height(C),1);

E=[
    0;
    0;
    1/(M+m);
    0
];

M = 0.2;    % Mass of cart
m=0.023;    % Mass of pendulum
J=3.20e-4;	% Inertia moment
L=0.2;		% Length
mu=2.74e-4;	% Damping coefficient
zeta=240;   % Physical parameter of DC motor
xi=90;		% Physical parameter of DC motor
g=9.81;     % Gravity accel.

p1=m*L/(J+m*L*L); p2=mu/(J+m*L*L);

% Acceleration function for non-linear sim
ddx = @(x, theta, dx, dtheta,u) (((-m*L*cos(theta)*(m*g*L*sin(theta)-mu*dtheta)/(J+m*L^2)) + (m*L*(dtheta^2)*sin(theta)) -mu*dx)+u) / ((M+m)-((m^2)*(L^2)*(cos(theta)^2)/(J+m*L^2)));
ddtheta = @(x, theta, dx, dtheta, ddx) (-m*L*ddx*cos(theta) + m*g*L*sin(theta) - mu*dtheta)/(J+m*L^2);

% Assignment symbolic variables シンボリック変数の代入
A = double(subs(Amat));
B = double(subs(Bmat));

% define states vector 状態ベクトル、出力ベクトル、入力ベクトルの定義、外乱ベクトル
x = [zeros(4,TL_width)];
y = [zeros(height(C),TL_width)];
u = [zeros(1,TL_width)];
d = [zeros(1,TL_width)];

x(:,1) = [x0;theta0;dx0;dtheta0];  % 初期状態量の代入

%% Model Analysis モデルの解析
state_name = {"x","\theta","dxdt","d\thetadt"};
output_name = state_name(logical(sum(C,1)));
input_name = {"u_{cart}"};
sys_cart = ss(A,B,C,D,"OutputName",output_name,"InputName",input_name);  % continuous time 連続時間システム
tf_vcl = tf(sys_cart);
pole(sys_cart)
% figure;
% pzmap(ss(A,B,C,D))
% grid on;
% bode(sys_cart)

%% Controller Design 制御系設計
% discretization 行列の離散化
sys_cart_d = c2d(sys_cart,control_dt);
Ad = sys_cart_d.A;      % 離散システム行列
Bd = sys_cart_d.B;      % 離散入力係数行列


% ===LQR 最適レギュレータ===
%         x1 x2 x3 x4
Q_lqr = diag([1, 2, 0.1, 0.1]);                        % 状態量重み
R_lqr = diag([0.1]);                                  % 入力重み
[K_lqr,S_lqr,P_lqr] = lqr(A,B,Q_lqr,R_lqr,[]);      % 連続時間最適レギュレータ
% [K_lqr,S_lqr,P_lqr] = dlqr(Ad,Bd,Q_lqr,R_lqr,[]);   % 離散時間最適レギュレータ
pole(ss(A-B*K_lqr,B,C,D))                           % 最適レギュレータを用いた際のシステムの極
% figure;
% pzmap(ss(A-B*K_lqr,B,C,D))
% grid on;
% bode(ss(A-B*K_lqr,E,C,D,"OutputName"output_name,"InputName",disturbance_name))


% ===Servo 最適サーボ系===
x_inf = [0.5; 0; 0; 0];             % 無限時間で達成したい状態量（目標状態）
r = zeros(height(C),TL_width);      % 目標値（0で一定）
r = repmat(C*x_inf,[1,TL_width]);   % 目標値（任意の値で一定）
w = zeros(height(C),TL_width);      % エラーリストの初期化
e = r-C*x;                          % エラーリストの初期化
x_ex = [x;w];                       % 拡大系の定義

% Expanded system 拡大系の定義
% 拡大系システム行列
phi = [
    A, zeros(height(A),height(C));
    -C, zeros(height(C),height(C))
    ];

% 拡大系入力係数行列
gamma = [
    B;
    zeros(height(C),width(B))
];

% 拡大系目標値係数行列
H = [
    zeros(height(A),height(C));
    eye(height(C))
];

% 拡大系観測行列
psi = [
    C, zeros(height(C),height(C))
];

% 拡大系外乱係数行列
eta = [
    E;
    -D
];

sys_ex = ss(phi,gamma,psi,[]);
sys_ex_d = c2d(sys_ex,control_dt);
%               x1 x2 x3 x4 e1 e2 e3 e4
% Q_servo = diag([1e-01, 1, 1, 1, 6, 4]);
% R_servo = diag([0.01]);
% [K_servo,P_servo,~] = lqr(phi,gamma,Q_servo,R_servo,[]);
% Q_servo = diag([1e-02, 1e-03, 1e-03, 1e-03, 1e-01, 1e-01]);
% R_servo = diag([1e-04]);
% [K_servo,P_servo,~] = lqrd(phi,gamma,Q_servo,R_servo,[],control_dt);
Q_servo = diag([1e-06, 1e-06, 1e-03, 1e-03, 1e-02, 1e-02]);
R_servo = diag([1e-05]);
[K_servo,P_servo,~] = dlqr(sys_ex_d.A,sys_ex_d.B,Q_servo,R_servo,[]);

P_11 = P_servo(1:height(A),1:height(B));
P_12 = P_servo(1:height(A),end-(height(e)-1):end);
P_22 = P_servo(end-(height(e)-1):end,end-(width(P_12)-1):end);
F_a=-K_servo(:,1:height(x));
G_a=-K_servo(:,height(x)+1:end);
H_a=([-F_a+(G_a/P_22)*(P_12') eye(width(B))])/([A B;C zeros(height(C),width(B))])*[zeros(height(A),height(C));eye(height(C))];


% ===Kalman Filter カルマンフィルタの設計===
% FOR LQR
y_noised = [zeros(height(C),TL_width)];
x_hat = x;
y_hat = [zeros(height(C),TL_width)];
Q_kalman = diag([1e-04, 1e-05, 1e-02, 1e-01]);
R_kalman = diag([1e-01, 1e-01]);
P_kalman = 0.01*ones(size(A));
L_kalman = P_kalman * C' / (C * P_kalman * C' + R_kalman); % カルマンゲイン

%% Simulation Loop
% modeling error モデル化誤差の再現
% M = M*1.3;
% A = double(subs(Amat));
% B = double(subs(Bmat));
for i = 1:TL_width-1

    % observation 観測値の取得と誤算の算出
    y(:,i) = C*x(:,i);
    e(:,i) = r(:,i) - y(:,i);

    % calculate input
    if ~passive
        if mod(i-1, control_dt/dt) == 0 && i-1 ~=0  % 制御周期且つi-1が存在する

            % add noise to obserbation
            y_noised(:,i) = y(:,i) + sqrt(0.001)*randn(size(y(:,i)));
            
            % KalmanFilter カルマンフィルタで状態推定
            x_hat(:,i) = Ad * x_hat(:,i-1) + Bd * u(:,i-1);                         % 予測ステップ
            P_kalman = Ad * P_kalman * Ad' + Q_kalman;

            L_kalman = P_kalman * C' / (C * P_kalman * C' + R_kalman);              % カルマンゲイン
            x_hat(:,i) = x_hat(:,i) + L_kalman * (y_noised(:,i) - C * x_hat(:,i));  % 更新ステップ
            P_kalman = (eye(size(L_kalman,1)) - L_kalman * C) * P_kalman;

            if LQR
                % u(:,i) = -K_lqr*x(:,i);
                u(:,i) = -K_lqr*x_hat(:,i);  % optimal input
            elseif servo
                % x_ex(1:height(x),i) = x_hat(:,i);
                u(:,i) = -K_servo*(x_ex(:,i)) + H_a*r(:,i) - (G_a/P_22)*(P_12')*x_ex(1:height(A),1) - G_a*x_ex(height(A)+1:end,1);  % optimal input
            end
        elseif i-1 ~= 0
            % e(:,i) = e(:,i-1);
            u(:,i) = u(:,i-1);          % ゼロ次ホールド
            x_hat(:,i) = x_hat(:,i-1);
        end
    end
    
    % update states ルンゲクッタ法による状態量の更新
    x_ex(:,i+1) = func__rungekutta(x_ex(:,i), u(:,i), d(:,i), r(:,i), phi, gamma, eta, H, dt);  % 拡大系状態量更新
    % x(:,i+1) = func__rungekutta(x(:,i), u(:,i), d(:,i), [], A, B, E, [], dt);                   % 状態量更新
   
    % 非線形シミュレーション
    % Runge Kutta
    kx1 = dt*x(3,i);
    kt1 = dt*x(4,i);
    DDX = ddx(x(1,i), x(2,i), x(3,i), x(4,i),u(:,i));
    lx1 = dt*DDX;
    lt1 = dt*ddtheta(x(1,i), x(2,i), x(3,i), x(4,i), DDX);
    
    kx2 = dt*(x(3,i)+lx1/2);
    kt2 = dt*(x(4,i)+lt1/2);
    DDX = ddx(x(1,i)+kx1/2, x(2,i)+kt1/2, x(3,i)+lx1/2, x(4,i)+lt1/2,u(:,i));
    lx2 = dt*DDX;
    lt2 = dt*ddtheta(x(1,i)+kx1/2, x(2,i)+kt1/2, x(3,i)+lx1/2, x(4,i)+lt1/2, DDX);

    kx3 = dt*(x(3,i)+lx2/2);
    kt3 = dt*(x(4,i)+lt2/2);
    DDX = ddx(x(1,i)+kx2/2, x(2,i)+kt2/2, x(3,i)+lx2/2, x(4,i)+lt2/2,u(:,i));
    lx3 = dt*DDX;
    lt3 = dt*ddtheta(x(1,i)+kx2/2, x(2,i)+kt2/2, x(3,i)+lx2/2, x(4,i)+lt2/2, DDX);

    kx4 = dt*(x(3,i)+lx3/2);
    kt4 = dt*(x(4,i)+lt3/2);
    DDX = ddx(x(1,i)+kx3/2, x(2,i)+kt3/2, x(3,i)+lx3/2, x(4,i)+lt3/2,u(:,i));
    lx4 = dt*DDX;
    lt4 = dt*ddtheta(x(1,i)+kx3, x(2,i)+kt3, x(3,i)+lx3, x(4,i)+lt3, DDX);

    % RK = [
    %     kx1, kt1, lz1, lt1;
    %     kx2, kt2, lz2, lt2;
    %     kx3, kt3, lz3, lt3;
    %     kx4, kt4, lz4, lt4;
    %     ];
    RK = [
        kx1, kx2, kx3, kx4;
        kt1, kt2, kt3, kt4;
        lx1, lx2, lx3, lx4;
        lt1, lt2, lt3, lt4;
        ];

    x(:,i+1) = x(:,i) + (RK*[1; 2; 2; 1])/6;
    % x_ex(1:height(x),i+1) = x(:,i+1);
end

% calculate squared error 最適レギュレータの評価関数の中身
x_ex_squared = sum(x_ex.^2,2)
u_squared = sum(u.^2,2)

%% Drawing 図の描画
states_name = ["\itx","\it\theta","d\itx\rmd\itt","d\it\theta\rmd\itt"];
inputs_name = ["\itu\rm(\itt\rm)"];
disturbance_name = ["\itd\rm(\itt\rm)"];

fig = figure('name',"Simulation Results");
% drawing states
subplot(311);
plot(TL,x,"LineWidth",2); %,"Color","#000000~ffffff"
grid on;
xlabel("Time [s]");
ylabel("States Value");
legend(states_name);
% xlim([0,3])

% drawing road inputs
subplot(312);
plot(TL,d,"LineWidth",2);
grid on;
xlabel("Time [s]");
ylabel("Disturbance");
legend(disturbance_name);
% xlim([0,3])

% drawing control inputs
subplot(313);
plot(TL,u,"LineWidth",2);
grid on;
xlabel("Time [s]");
ylabel("Control Input Value");
legend(inputs_name);
% xlim([0,3])

% font
fontname(fig,"Times New Roman");
fontsize(fig,10,"points");

% save figure
controller_bool = [passive,LQR,servo];
controller = ["passive","lqr","servo"];
condition = "_controller-"+controller(controller_bool);
saveas(fig,"fig/"+condition);

% check states estimation by kalman filter
fig_kalman = figure('name',"States Estimation Results");
% drawing states
plot(TL,x_hat,"LineWidth",2); %,"Color","#000000~ffffff"
grid on;
xlabel("Time [s]");
ylabel("Estimated Value");
legend(states_name);

%% Animation アニメーションによる挙動の確認
x_cart1 = x(1,:)';
y_cart1 = zeros(size(x_cart1));
x_p = L*sin(x(2,:)');
y_p = L*cos(x(2,:)');
u = zeros(size(x_cart1));
% t = t';
figure("name","Cart Pole")
hold on; grid on;
hh3 = rectangle('Position',[x_cart1(1,1)-0.1 y_cart1(1,1)-0.15 0.2 0.15],'EdgeColor',[1 0 0]);
hh4 = plot([x_cart1(1,1),x_p(1,1)],[0,y_p(1,1)], Marker=".",MarkerSize=20,LineWidth=2);
axis equal
axis([-10*L 10*L -10*L 10*L])
ht = title("Time: "+round(TL(1),1)+" s"); % String arrays introduced in R2016b
xlabel("\itx \rm[m]")
fontname(gcf,"Times New Roman");
fontsize(gcf,10,"points");
for i = 1:TL_width
    set(hh3(1),pos=[x_cart1(i,1)-0.1 y_cart1(i,1)-0.15 0.2 0.15])
    set(hh4(1),XData=[x_cart1(i,1),x_cart1(i,1)+x_p(i,1)],YData=[0,y_p(i,1)])
    set(ht,String="Time: "+round(TL(i),1)+" s")
    drawnow
end