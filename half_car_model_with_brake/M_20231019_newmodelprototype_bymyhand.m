close all;
clear;

%% Parameters

% Vehicle
m_b = 1000;         % 車体重量                               [kg]
m_wf = 53;          % 前輪ばね下重量                         [kg]
m_wr = 53;       % 後輪ばね下重量                         [kg] 
I_b = 2500;         % 車体の慣性モーメント                    [kg.m2]ss
l_f = 1.05;         % 重心から前輪サスペンションマウントの距離  [m]
l_r = 1.4;          % 重心から後輪サスペンションマウントの距離  [m]
h_b = 0.5;          % 重心の高さ                             [m]
r_w = 0.1;          %車輪半径
k_vf = 25000;     % 前ばね定数                             [N/m]
k_vr = 30000;     % 後ばね定数                             [N/m]
c_vf = 2500;      % 前ダンパ減衰減数                        [N.s/m]
c_vr = 2500;      % 後ダンパ減衰係数                        [N.s/m]
k_hf = 80000;      % 前輪前後方向移動ばね定数                 [N/m]
k_hr = 80000;      % 後輪前後方向移動ばね定数                 [N/m]
c_hf = 1000;          % 前輪前後方向移動減衰係数                  [N.s/m]
c_hr = 1000;          % 後輪前後方向移動減衰係数                  [N.s/m]
k_tf = 270000;      % 前タイヤ弾性係数                           [N/m]
k_tr = 270000;      % 後タイヤ弾性係数                          [N/m]
c_tf = 1000;        % 前タイヤ減衰係数                          [N.s/m]
c_tr = 1000;        % 後タイヤ減衰係数                          [N.s/m]
tantheta_f = 0.1;   % アンチダイブ角
tantheta_r = 0.5;   % アンチリフト角

%Brake
Brake = 2500;              %総ブレーキ力
kbf = 0.5;                  %前ブレーキ配分
f_xf = Brake*kbf;            %フロントブレーキ制動力
f_xr = Brake*(1-kbf);        %リアブレーキ制動力

%% Simulation parameter
Tf = 20;
dt = 0.0001;
tl = 0:dt:Tf;
tl_w = width(tl);

%% Initial condition
%Vehicle
v = 40;             %[km/h]わかりずらいのでキロ時速で入力

x_b_0 = 0;            %[m]
x_wf_0 = 0;
x_wr_0 = 0;
z_b_0 = 0;            %[m]
z_wf_0 = 0;
z_wr_0 = 0;
theta_0 = 0;        %[deg]
x_bdot_0 = v/3.6;     %[m/s]キロ時速で入力したものをメートル秒速に変換
x_wfdot_0 = v/3.6;
x_wrdot_0 = v/3.6;
z_bdot_0 = 0;         %[m/s]
z_wfdot_0 = 0;
z_wrdot_0 = 0;
thetadot_0 = 0;     %[deg/s]

%% State matrix 
% Vehicle

M = [m_b,    0,    0,    0,    0,    0,   0;
       0, m_wf,    0,    0,    0,    0,   0;
       0,    0, m_wr,    0,    0,    0,   0; 
       0,    0,    0,  m_b,    0,    0,   0;
       0,    0,    0,    0, m_wf,    0,   0;
       0,    0,    0,    0,    0, m_wr,   0;
       0,    0,    0,    0,    0,    0, I_b];

C = [            c_hf + c_hr,           -c_hf,           -c_hr,                   0,           0,           0,                             (c_hf + c_hr)*(h_b-r_w);
                       -c_hf,            c_hf,               0,                   0,           0,           0,                                     -c_hf*(h_b-r_w);
                       -c_hr,               0,            c_hr,                   0,           0,           0,                                     -c_hr*(h_b-r_w);
                           0,               0,               0,         c_vf + c_vr,       -c_vf,       -c_vr,                                 c_vf*l_f - c_vr*l_r;
                           0,               0,               0,               -c_vf, c_vf + c_tf,           0,                                           -c_vf*l_f;
                           0,               0,               0,               -c_vr,           0, c_vr + c_tr,                                            c_vr*l_r;
     (c_hf + c_hr)*(h_b-r_w), -c_hf*(h_b-r_w), -c_hr*(h_b-r_w), c_vf*l_f - c_vr*l_r,   -c_vf*l_f,    c_vr*l_r, c_vf*l_f^2 + c_vr*l_r^2 + (c_hf + c_hr)*(h_b-r_w)^2];

K = [            k_hf + k_hr,           -k_hf,           -k_hr,                   0,           0,           0,                             (k_hf + k_hr)*(h_b-r_w);
                       -k_hf,            k_hf,               0,                   0,           0,           0,                                     -k_hf*(h_b-r_w);
                       -k_hr,               0,            k_hr,                   0,           0,           0,                                     -k_hr*(h_b-r_w);
                           0,               0,               0,         k_vf + k_vr,       -k_vf,       -k_vr,                                 k_vf*l_f - k_vr*l_r;
                           0,               0,               0,               -k_vf, k_vf + k_tf,           0,                                           -k_vf*l_f;
                           0,               0,               0,               -k_vr,           0, k_vr + k_tr,                                            k_vr*l_r;
     (k_hf + k_hr)*(h_b-r_w), -k_hf*(h_b-r_w), -k_hr*(h_b-r_w), k_vf*l_f - k_vr*l_r,   -k_vf*l_f,    k_vr*l_r, k_vf*l_f^2 + k_vr*l_r^2 + (k_hf + k_hr)*(h_b-r_w)^2];

F = [0, 0;
     1, 0;
     0, 1;
     0, 0;
     0, 0;
     0, 0;
     0, 0];

A=[zeros(size(M)) eye(size(M));
   -inv(M)*K      -inv(M)*C  ];
B=[zeros(size(F));
   -inv(M)*F];
C = eye(height(A));
D = zeros(height(A),2);
x = zeros(width(A),1);
x(:, 1) = [x_b_0      ;
           x_wf_0     ;
           x_wr_0     ;
           z_b_0      ;
           z_wf_0     ;
           z_wr_0     ;
           theta_0    ;
           x_bdot_0   ;
           x_wfdot_0  ;
           x_wrdot_0  ;
           z_bdot_0   ;
           z_wfdot_0  ;
           z_wrdot_0  ;
           thetadot_0];

bk_input = [zeros(1,1/dt),f_xf.*ones(1,tl_w-1/dt);
            zeros(1,1/dt),f_xr.*ones(1,tl_w-1/dt)];
% disp(A)
% disp(B)
 
sys = ss(A,B,C,D);
%% Simulation loop
for i = 1:tl_w-1
    
    % if x(8,i) < 0 && x(9,i) < 0 && x(10,i) < 0
    %     bk_input = zeros(2,1);
    % end
    if x(8,i) <= 0
        bk_input(:,i) = zeros(2,1);
        X1 = x(:,i);           b1 = dt*motion_func(X1, bk_input(:,i), A, B);
        X2 = x(:,i) + b1/2;    b2 = dt*motion_func(X2, bk_input(:,i), A, B);
        X3 = x(:,i) + b2/2;    b3 = dt*motion_func(X3, bk_input(:,i), A, B);
        X4 = x(:,i) + b3/2;    b4 = dt*motion_func(X4, bk_input(:,i), A, B);
        x(:,i+1) = x(:,i) + (b1+ 2*b2 + 2*b3 + b4)/6;
        x(2,i+1) = x(2,i);
    else
        X1 = x(:,i);           b1 = dt*motion_func(X1, bk_input(:,i), A, B);
        X2 = x(:,i) + b1/2;    b2 = dt*motion_func(X2, bk_input(:,i), A, B);
        X3 = x(:,i) + b2/2;    b3 = dt*motion_func(X3, bk_input(:,i), A, B);
        X4 = x(:,i) + b3/2;    b4 = dt*motion_func(X4, bk_input(:,i), A, B);
        x(:,i+1) = x(:,i) + (b1+ 2*b2 + 2*b3 + b4)/6;
    end
    % if i*dt >= 5
    %     bk_input = zeros(2,1);
    % end

end

%plot(xh(1,:))
%% Drawing
labels = ["x",           "Time [s]", "Vehicle Holizontal Displacement [m]";
          "x_fwheel",    "Time [s]", "F-wheel Holizontal Displacement [m]";
          "x_rwheel",    "Time [s]", "R-wheel Holizontal Displacement [m]";
          "z",           "Time [s]", "Vehicle Vertical Displacement [cm]" ;
          "z_fwheel",    "Time [s]", "F-wheel Vertical Displacement [cm]" ;
          "z_rwheel",    "Time [s]", "R-wheel Vertical Displacement [cm]" ;
          "theta",       "Time [s]", "Vehicle Angle [deg]"                ;
          "xdot",        "Time [s]", "Vehicle Holizontal Velocity [km/h]" ;
          "xdot_fwheel", "Time [s]", "F-wheel Holizontal Velocity [km/h]" ;
          "xdot_rwheel", "Time [s]", "R-wheel Holizontal Velocity [km/h]" ;
          "zdot",        "Time [s]", "Vehicle Vertical Velocity [m/s]"    ;
          "zdot_fwheel", "Time [s]", "F-wheel Vertical Velocity [m/s]"    ;
          "zdot_rwheel", "Time [s]", "R-wheel Vertical Velocity [m/s]"    ;
          "thetadot",    "Time [s]", "Vehicle Angular Velocity [deg/s]"   ];

fig = figure("name", "Results");
for k = 1:height(x)
    if k == 7 || k == 14
        drawer(fig,tl,rad2deg(x(k,:)),labels(k,:),k);
    elseif k == 4 || k == 5 || k == 6
        drawer(fig,tl,x(k,:)*100,labels(k,:),k);
    elseif k == 8 || k == 9 || k == 10
        drawer(fig,tl,3.6.*x(k,:),labels(k,:),k);
    else
        drawer(fig,tl,x(k,:),labels(k,:),k);
    end
end

fprintf('phai最小値: %d\n', rad2deg(min(x(4,1:tl_w-1))));
fprintf('phai最大値: %d\n', rad2deg(max(x(4,1:tl_w-1))));
fprintf('phaidot最小値: %d\n', rad2deg(min(x(8,1:tl_w-1))));
fprintf('phaidot最大値: %d\n', rad2deg(max(x(8,1:tl_w-1))))

run("half_model_animation.m")