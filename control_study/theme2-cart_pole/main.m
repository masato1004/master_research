close all;
clear;

%% Define simlation condition シミュレーション条件
% loop parameters
T = 10;                 % シミュレーション時間
dt = 1e-04;             % シミュレーション時間幅
TL = 0:dt:T;            % 時間リスト作成
TL_width = width(TL);   % 時間リストの長さ取得（リストの要素数）
control_dt = dt;        % 制御周期（デフォルト：シミュレーション時間幅）

% initial value
x0 = 0;
theta0 = pi/7;
dx0 = 0;
dtheta0 = 0;

% controller
passive = false;
LQR = false;
servo = true;

%% Model Definition モデルの定義
% define state space: dxdt = Ax(t) + Bu(t) + Ed(t), y(t) = Cx(t) + Du(t)
% 倒立振子の状態空間モデル(MIMOモデル)

M = 0.2;    % Mass of cart
m=0.023;    % Mass of pendulum
J=3.20e-4;	% Inertia moment
L=0.2;		% Length
mu=2.74e-5;	% Damping coefficient
zeta=240;   % Physical parameter of DC motor
xi=90;		% Physical parameter of DC motor
g=9.81;     % Gravity accel.

p1=m*L/(J+m*L*L); p2=mu/(J+m*L*L);

A=[
    0       0       1       0;
    0       0       0       1;
    0       0   -zeta       0;
    0    p1*g p1*zeta     -p2
    ];

B=[
         0;
         0;
        xi;
    -p1*xi
    ];

C=diag([1,1,0,0]);
C(sum(C,2)==0,:)=[];  % eliminate rows filled with 0 不要な行の削除

D=zeros(height(C),1);

E=B;

% define states vector 状態ベクトル、出力ベクトル、入力ベクトルの定義
x = [zeros(4,TL_width)];
y = [zeros(rank(C),TL_width)];
u = [zeros(1,TL_width)];
d = [ones(1,TL_width)];

x(:,1) = [x0;theta0;dx0;dtheta0];  % 初期値の代入

%% Model Analysis モデルの解析
state_name = {"x","\theta","dxdt","d\thetadt"};
output_name = state_name(logical(sum(C,1)));
input_name = {"u_{cart}"};
sys_vcl = ss(A,B,C,D,"OutputName",output_name,"InputName",input_name);  % continuous time 連続時間システム
tf_vcl = tf(sys_vcl)
pole(sys_vcl)
bode(sys_vcl)

%% Controller Design 制御系設計
% discretization 行列の離散化
disc_func = @(tau,Mat) (-pinv(A)*expm(A.*(control_dt-tau)))*Mat;

Ad = expm(A.*control_dt);
Bd = disc_func(control_dt,B) - disc_func(0,B);

% ===LQR 離散時間最適レギュレータ===
%         x1 x2 x3 x4
Q = diag([10, 5, 1, 1]);
R = diag([1]);
[K_lqr,S,P] = lqr(A,B,Q,R,[]);
pole(ss(A-B*K_lqr,B,C,D))
% bode(ss(A-B*K_lqr,E,C,D,"OutputName"output_name,"InputName",disturbance_name))

% ===Servo 離散時間最適サーボ系===
x_inf = [0.5;0;0;0];
u_inf = -d(end);
r = zeros(height(C),TL_width);  % 目標値（0で一定）
r = repmat(C*x_inf,[1,TL_width]);  % 目標値（0で一定）
w = zeros(height(C),TL_width);  % エラーリストの初期化
e = r-C*x;  % エラーリストの初期化
x_ex = [x;w];

% Expanded system 拡大系の定義
phi = [
    A, zeros(height(A),height(C));
    -C, zeros(height(C),height(C))
    % Ad, zeros(height(Ad),height(C));
    % -C*Ad, eye(height(C),height(C))
    ];
G = [
    B;
    zeros(height(C),width(B))
    % Bd;
    % -C*Bd
];

H = [
    zeros(height(A),height(C));
    eye(height(C))
];

psi = [
    C, zeros(height(C),height(C))
];
eta = [
    E;
    -D
];

%               x1 x2 x3 x4 e1 e2 e3 e4
Q_servo = diag([1, 1, 1, 1, 6, 4]);
R_servo = diag([1]);
[K_servo,S,P] = lqr(phi,G,Q_servo,R_servo,[]);

P_11 = S(1:height(A),1:height(B));
P_12 = S(1:height(A),end-(height(e)-1):end);
P_22 = S(end-(height(e)-1):end,end-(width(P_12)-1):end);
F_a=-K_servo(:,1:height(x));
G_a=-K_servo(:,height(x)+1:end);
H_a=([-F_a+(G_a/P_22)*(P_12') eye(width(B))])/([A B;C zeros(height(C),width(B))])*[zeros(height(A),height(C));eye(height(C))];

%% Simulation Loop
% modeling error モデル化誤差の再現（60kg 乗員5名）
% m_b = m_b+300;
% A = double(subs(A));
% B = double(subs(B));
for i = 1:TL_width-1

    % observation 観測値の取得と誤算の算出
    y(:,i) = C*x(:,i);
    e(:,i) = r(:,i) - y(:,i);

    % calculate input
    if ~passive
        if mod(i-1, control_dt/dt) == 0 && i-1 ~=0  % 制御周期且つi-1が存在する
            if LQR
                u(:,i) = -K_lqr*x(:,i);
            elseif servo
                % e(:,i) = r(:,i) - y(:,i);
                u(:,i) = -K_servo*(x_ex(:,i)) - G_a*x_ex(height(A)+1:end,1) - (G_a/P_22)*(P_12')*x_ex(1:height(A),1) + H_a*r(:,i);
                % u(:,i) = -K_servo*[x(:,i)-[0.5;0;0;0]; e(:,i-1)] - G_a*e(:,1) - (G_a/P_22)*(P_12')*x(:,1) + H_a*r(:,i) + u_inf;
            end
        elseif i-1 ~= 0
            % e(:,i) = e(:,i-1);
            u(:,i) = u(:,i-1);
        end
    end

    % update states ルンゲクッタ法による状態量の更新
    x_ex(:,i+1) = func__rungekutta(x_ex(:,i), u(:,i), d(:,i), r(:,i), phi, G, eta, H, dt);
    x(:,i+1) = func__rungekutta(x(:,i), u(:,i), d(:,i), [], A, B, E, [], dt);
end

% calculate squared error 最適レギュレータの評価関数の中身
x_squared = sum(x.^2,2)
e_squared = sum(e.^2,2)
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
xlim([0,3])

% drawing road inputs
subplot(312);
plot(TL,d,"LineWidth",2);
grid on;
xlabel("Time [s]");
ylabel("Road Disturbance");
legend(disturbance_name);
xlim([0,3])

% drawing control inputs
subplot(313);
plot(TL,u,"LineWidth",2);
grid on;
xlabel("Time [s]");
ylabel("Control Input Value");
legend(inputs_name);
xlim([0,3])

% font
fontname(fig,"Times New Roman");
fontsize(fig,10,"points");

% save figure
controller_bool = [passive,LQR,servo];
controller = ["passive","lqr","servo"];
condition = "_controller-"+controller(controller_bool);
saveas(fig,"fig/"+condition);

%% Animation アニメーションによる挙動の確認
x_cart1 = x_ex(1,:)';
y_cart1 = zeros(size(x_cart1));
x_p = L*sin(x_ex(2,:)');
y_p = L*cos(x_ex(2,:)');
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
for i = 1:50:TL_width
    set(hh3(1),pos=[x_cart1(i,1)-0.1 y_cart1(i,1)-0.15 0.2 0.15])
    set(hh4(1),XData=[x_cart1(i,1),x_cart1(i,1)+x_p(i,1)],YData=[0,y_p(i,1)])
    set(ht,String="Time: "+round(TL(i),1)+" s")
    drawnow
end