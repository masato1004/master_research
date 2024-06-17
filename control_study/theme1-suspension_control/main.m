close all;
clear;

%% Define simlation condition シミュレーション条件
% loop parameters
T = 10;                 % シミュレーション時間
dt = 1e-04;             % シミュレーション時間幅
TL = 0:dt:T;            % 時間リスト作成
TL_width = width(TL);   % 時間リストの長さ取得（リストの要素数）
control_dt = dt;        % 制御周期（デフォルト：シミュレーション時間幅）

% environmental parmeters
Vkmh = 50;
V = Vkmh*1000/3600;  % [m/s]  velocity
road_shape = "_jari_"  % _unevenness_, _sin_, (_jari_)
road_frequency = 6;  % [Hz]  車速から凹凸の幅を決定
road_hgeit = 0.08;   % [m]   凹凸の高さ

% controller
passive = false;
LQR = false;
servo = true;

%% Model Definition モデルの定義
syms L_f L_r k_sf k_sr c_sf c_sr m_b I_b

% define state space: dxdt = Ax(t) + Bu(t) + Ed(t), y(t) = Cx(t) + Du(t)
A = [
                             0,                              0,                          1,                              0;
                             0,                              0,                          0,                              1;
            -(k_sf + k_sr)/m_b,     -(L_f*k_sf - L_r*k_sr)/m_b,         -(c_sf + c_sr)/m_b,     -(L_f*c_sf - L_r*c_sr)/m_b;
    -(L_f*k_sf - L_r*k_sr)/I_b, -(k_sf*L_f^2 + k_sr*L_r^2)/I_b, -(L_f*c_sf - L_r*c_sr)/I_b, -(c_sf*L_f^2 + c_sr*L_r^2)/I_b
    ];

B = [
          0,        0;
          0,        0;
      1/m_b,    1/m_b;
    L_f/I_b, -L_r/I_b
    ];

%         x1 x2 x3 x4
C = diag([1, 0, 0, 1]);  % 1 or 0
C(sum(C,2)==0,:)=[];  % eliminate rows filled with 0 不要な行の削除

D = [];

E = [
                 0,               0,              0,               0;
                 0,               0,              0,               0;
          k_sf/m_b,        k_sr/m_b,       c_sf/m_b,        c_sr/m_b;
    (L_f*k_sf)/I_b, -(L_r*k_sr)/I_b, (L_f*c_sf)/I_b, -(L_r*c_sr)/I_b
    ];

% define model parameter
g = 9.80665;    % [m/s^2]   gravity

L_f  = 1.47;    % [m]       front length
L_r  = 1.3;     % [m]       rear length
wb   = L_f+L_r; % [m]       wheel base

k_sf = 30000;   % [N/m]     front spring stiffness
k_sr = 27000;   % [N/m]     rear spring stiffness
c_sf = 3500;    % [N/(m/s)] front damping
c_sr = 3200;    % [N/(m/s)] rear damping

m_b  = 1000;    % [N]      Body weight
I_b  = (m_b)*(wb/2)^2;  % [kgm^2]     Inertia

% Assignment symbolic variables シンボリック変数の代入
A = double(subs(A));
B = double(subs(B));
E = double(subs(E));

% define states vector 状態ベクトル、出力ベクトル、入力ベクトルの定義
x = [zeros(4,TL_width)];
y = [zeros(rank(C),TL_width)];
u = [zeros(2,TL_width)];

run("config__rpf_settings") % 路面形状を作る（今回は中身は関係ありません。）
d = r_p;  % [z_f; z_r; dz_f; dz_r] 路面プロファイルの定義

%% Model Analysis モデルの解析
state_name = {"x","\theta","dxdt","d\thetadt"};
output_name = state_name(logical(sum(C,1)));
input_name = {"sus_{front}","sus_{rear}"};
disturbance_name = {"z_{disf}","z_{disr}","dzdt_{disf}","dzdt_{disr}"};
sys_vcl = ss(A,B,C,D,"OutputName",output_name,"InputName",input_name);  % continuous time 連続時間システム
tf_vcl = tf(sys_vcl)
pole(sys_vcl)
bode(sys_vcl)
% bode(ss(A,E,C,D,"OutputName",output_name,"InputName",disturbance_name))

%% Controller Design 制御系設計
% discretization 行列の離散化
disc_func = @(tau,Mat) (-A\expm(A.*(control_dt-tau)))*Mat;

Ad = expm(A.*control_dt);
Bd = disc_func(control_dt,B) - disc_func(0,B);

% ===LQR 離散時間最適レギュレータ===
%         x1 x2 x3 x4
Q = diag([1e01, 1, 1, 1e11]);
R = diag([1e-02 1e-02]);
[K_lqr,S,P] = lqr(A,B,Q,R,[]);
pole(ss(A-B*K_lqr,E,C,D))
% bode(ss(A-B*K_lqr,E,C,D,"OutputName"output_name,"InputName",disturbance_name))

% ===Servo 離散時間最適サーボ系===
r = zeros(height(C),TL_width);  % 目標値（0で一定）
e = zeros(height(C),TL_width);  % エラーリストの初期化

% Expanded system 拡大系の定義
phi = [
    A, zeros(height(Ad),height(C));
    -C, zeros(height(C),height(C))
    % Ad, zeros(height(Ad),height(C));
    % -C*Ad, zeros(height(C),height(C))
    ];
G = [
    B;
    zeros(height(C),width(Bd))
    % Bd;
    % -C*Bd
];
psi = [
    C, zeros(height(C),height(C))
];

%               x1 x2 x3 x4 e1 e2 e3 e4
Q_servo = diag([1e-2, 1e-2, 1e-2, 1e-2, 1e1, 1e-2]);
R_servo = diag([1e-2 1e-2]);
[K_servo,S,P] = lqr(phi,G,Q_servo,R_servo,[]);

P_11 = S(1:height(A),1:height(B));
P_12 = S(1:height(A),end-(height(e)-1):end);
P_22 = S(end-(height(A)-1):end,end-(width(P_12)-1):end);
F_a=-R_servo\(B')*P_11;
G_a=-R_servo\(B')*P_12;
H_a=([-F_a+G_a/(P_22)*(P_12') eye(height(C))])*pinv([A B;C zeros(height(C),width(B))])*[zeros(height(A),height(C));eye(height(C))];

%% Simulation Loop
% modeling error モデル化誤差の再現（60kg 乗員5名）
m_b = m_b+300;
A = double(subs(A));
B = double(subs(B));
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
                u(:,i) = -K_servo*[x(:,i); e(:,i-1)] + H_a*r(:,i);
            end
        elseif i-1 ~= 0
            u(:,i) = u(:,i-1);
        end
    end

    % update states ルンゲクッタ法による状態量の更新
    x(:,i+1) = func__rungekutta(x(:,i), u(:,i), d(:,i), A, B, E, dt);
end

% calculate squared error 最適レギュレータの評価関数の中身
x_squared = sum(x.^2,2)
e_squared = sum(e.^2,2)
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

%% Animation アニメーションによる挙動の確認
save_name = condition;
% run("appendix__animation.m")