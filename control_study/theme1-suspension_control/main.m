close all;
clear;

%% Define simlation condition シミュレーション条件
% loop parameters
T = 10;                         % シミュレーション時間
dt = 1e-04;                     % シミュレーション時間幅
TL = 0:dt:T;                    % 時間リスト作成
TL_width = width(TL);           % 時間リストの長さ取得（リストの要素数）
ctrl_dt = dt;                   % 制御周期（デフォルト：シミュレーション時間幅）
% ctrl_dt = dt*100;                % 制御周期（デフォルト：シミュレーション時間幅）  

% environmental parmeters
Vkmh = 60;
V = Vkmh*1000/3600;             % [m/s]  velocity
road_shape = "_unevenness_"     % 単一起伏：_unevenness_, sin波形状：_sin_, (実際の路面を想定したプロファイル：_jari_)
road_frequency = 3;             % [Hz]  車速とこの値から凹凸の幅を決定
road_height = 0.08;              % [m]   凹凸の最大高さ

% controller
passive = true;        % パッシブシミュレーション
LQR = false;            % LQR
servo = false;           % 積分型最適サーボ系

%% Model Definition モデルの定義
syms L_f L_r k_sf k_sr c_sf c_sr m_b I_b

% define state space: dxdt = Ax(t) + Bu(t) + Ed(t), y(t) = Cx(t) + Du(t)
Amat = [
                             0,                              0,                          1,                              0;
                             0,                              0,                          0,                              1;
            -(k_sf + k_sr)/m_b,     -(L_f*k_sf - L_r*k_sr)/m_b,         -(c_sf + c_sr)/m_b,     -(L_f*c_sf - L_r*c_sr)/m_b;
    -(L_f*k_sf - L_r*k_sr)/I_b, -(k_sf*L_f^2 + k_sr*L_r^2)/I_b, -(L_f*c_sf - L_r*c_sr)/I_b, -(c_sf*L_f^2 + c_sr*L_r^2)/I_b
    ];

Bmat = [
          0,        0;
          0,        0;
      1/m_b,    1/m_b;
    L_f/I_b, -L_r/I_b
    ];

%         x1 x2 x3 x4
C = diag([1,1, 1, 1]);  % 1 or 0
C(sum(C,2)==0,:)=[];  % eliminate rows filled with 0 不要な行の削除

D = [];

Emat = [
                 0,               0,              0,               0;
                 0,               0,              0,               0;
          k_sf/m_b,        k_sr/m_b,       c_sf/m_b,        c_sr/m_b;
    (L_f*k_sf)/I_b, -(L_r*k_sr)/I_b, (L_f*c_sf)/I_b, -(L_r*c_sr)/I_b
    ];

% define model parameter
L_f  = 1.47;                % [m]       front length
L_r  = 1.3;                 % [m]       rear length
wb   = L_f+L_r;             % [m]       wheel base

k_sf = 30000;               % [N/m]     front spring stiffness
k_sr = 27000;               % [N/m]     rear spring stiffness
c_sf = 3500;                % [N/(m/s)] front damping
c_sr = 3200;                % [N/(m/s)] rear damping

m_b  = 1000;                % [N]       Body weight
I_b  = (m_b)*(wb/2)^2;      % [kgm^2]   Inertia Moment

% Assignment symbolic variables シンボリック変数の代入
A = double(subs(Amat));
B = double(subs(Bmat));
E = double(subs(Emat));

% define states vector 状態ベクトル、出力ベクトル、入力ベクトルの定義
x = [zeros(4,TL_width)];
y = [zeros(rank(C),TL_width)];
u = [zeros(2,TL_width)];

run("appendix__rpf_settings") % 路面形状を作るファイルの実行（今回は中身は関係ありません。）
d = r_p;  % [z_f; z_r; dz_f; dz_r] 路面プロファイルの定義

%% Model Analysis モデルの解析
state_name = {"x","\theta","dxdt","d\thetadt"};
output_name = state_name(logical(sum(C,1)));
input_name = {"sus_{front}","sus_{rear}"};
disturbance_name = {"z_{disf}","z_{disr}","dzdt_{disf}","dzdt_{disr}"};
sys_vcl = ss(A,B,C,D,"OutputName",output_name,"InputName",input_name);  % continuous time system 連続時間システム
rank(ctrb(A,B))
tf_vcl = tf(sys_vcl)
pole(sys_vcl)
bode(sys_vcl)
% bode(ss(A,E,C,D,"OutputName",output_name,"InputName",disturbance_name))

%% Controller Design 制御系設計
% discretization 行列の離散化
sys_vcl_d = c2d(sys_vcl,ctrl_dt);  % discrete time system 離散時間システム
Ad = sys_vcl_d.A;
Bd = sys_vcl_d.B;

% ===LQR 最適レギュレータ===
%         x1 x2 x3 x4
Q = diag([1e01, 1, 1, 1e11]);       % 状態量重み
R = diag([1e-02 1e-02]);            % 入力重み
[K_lqr,S,P] = lqr(A,B,Q,R,[]);      % 連続時間最適レギュレータ
% [K_lqr,S,P] = dlqr(Ad,Bd,Q,R,[]);   % 離散時間最適レギュレータ
pole(ss(A-B*K_lqr,E,C,D))           % 最適制御有りのシステムの極
% bode(ss(A-B*K_lqr,E,C,D,"OutputName",output_name,"InputName",disturbance_name))

% ===Servo 最適サーボ系===
r = zeros(height(C),TL_width);  % 目標値（0で一定）
e = zeros(height(C),TL_width);  % エラーリストの初期化

% Expanded system 拡大系の定義
phi = [
    A, zeros(height(A),height(C));
    -C, zeros(height(C),height(C))
    ];
gamma = [
    B;
    zeros(height(C),width(B))
];
psi = [
    C, zeros(height(C),height(C))
];

%               x1 x2 x3 x4 e1 e2 e3 e4
Q_servo = diag([1e-01, 3, 1e-02, 1, 1, 3]);
R_servo = diag([1e-3 1e-3]);
[K_servo,S,P] = lqr(phi,gamma,Q_servo,R_servo,[]);
% Q_servo = diag([1e-2, 1e-2, 1e-2, 1e-2, 1e1, 1e-2]);
% R_servo = diag([1e-2 1e-2]);
% [K_servo,S,P] = lqrd(phi,gamma,Q_servo,R_servo,[],ctrl_dt);

P_11 = S(1:height(A),1:height(B));
P_12 = S(1:height(A),end-(height(e)-1):end);
P_22 = S(end-(height(e)-1):end,end-(width(P_12)-1):end);
F_a=-K_servo(:,1:height(x));
G_a=-K_servo(:,height(x)+1:end);
H_a=([-F_a+(G_a/P_22)*(P_12') eye(width(B))])/([A B;C zeros(height(C),width(B))])*[zeros(height(A),height(C));eye(height(C))];

% ===Kalman Filter カルマンフィルタの設計===
% FOR LQR
y_noised = [zeros(height(C),TL_width)];
x_hat = x;
y_hat = [zeros(height(C),TL_width)];
Q_kalman = diag([1e-07, 1e-07, 1e-06, 1e-07]);
R_kalman = diag([1e-04, 1e-04]);
P_kalman = 0.001*ones(size(A));
L_kalman = P_kalman * C' / (C * P_kalman * C' + R_kalman); % カルマンゲイン

%% Simulation Loop
% modeling error モデル化誤差の再現（6kg 乗員5名）
m_b = m_b+60*5;
A = double(subs(Amat));
B = double(subs(Bmat));
E = double(subs(Emat));

for i = 1:TL_width-1

    % observation 観測値の取得と誤算の算出
    y(:,i) = C*x(:,i);
    e(:,i) = r(:,i) - y(:,i);

    % calculate input
    if ~passive
        if mod(i-1, ctrl_dt/dt) == 0 && i-1 ~=0  % 制御周期且つi-1が存在する

            % add noise to obserbation
            y_noised(:,i) = y(:,i) + sqrt(0.001)*randn(size(y(:,i)));               % 観測ノイズの再現
            
            % KalmanFilter カルマンフィルタで状態推定
            x_hat(:,i) = Ad * x_hat(:,i-1) + Bd * u(:,i-1);                         % 予測ステップ
            P_kalman = Ad * P_kalman * Ad' + Q_kalman;

            L_kalman = P_kalman * C' / (C * P_kalman * C' + R_kalman);              % カルマンゲイン
            x_hat(:,i) = x_hat(:,i) + L_kalman * (y_noised(:,i) - C * x_hat(:,i));  % 更新ステップ
            P_kalman = (eye(size(L_kalman,1)) - L_kalman * C) * P_kalman;

            if LQR
                u(:,i) = -K_lqr*x(:,i);
                % u(:,i) = -K_lqr*x_hat(:,i);  % optimal input
            elseif servo
                u(:,i) = -K_servo*[x(:,i); e(:,i-1)] + H_a*r(:,i);
            end
        elseif i-1 ~= 0
            u(:,i) = u(:,i-1);
        end
    end

    % update states ルンゲクッタ法による状態量の更新
    x(:,i+1) = func__rungekutta(x(:,i), u(:,i), d(:,i), [], A, B, E, [], dt);
end

% calculate squared error 最適レギュレータの評価関数の中身
x_squared = sum(x.^2,2)
u_squared = sum(u.^2,2)

%% Drawing 図の描画
states_name = ["\itz","\it\theta","d\itz\rmd\itt","d\it\theta\rmd\itt"];
inputs_name = ["Front", "Rear"];

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
plot(TL,d(1:2,:),"LineWidth",2);
grid on;
xlabel("Time [s]");
ylabel("Road Disturbance");
legend(inputs_name);
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
condition = "V-"+Vkmh+"_roadshape-"+road_shape+"_controller-"+controller(controller_bool);
saveas(fig,"fig/"+condition);

% check states estimation by kalman filter
fig_kalman = figure('name',"States Estimation Results");
% drawing states
plot(TL,x_hat,"LineWidth",2); %,"Color","#000000~ffffff"
grid on;
xlabel("Time [s]");
ylabel("Estimated Value");
legend(states_name);
xlim([0,3])

%% Animation アニメーションによる挙動の確認
save_name = condition;
% run("appendix__animation.m")