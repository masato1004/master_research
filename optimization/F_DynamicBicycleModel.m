function [dxdt, ay]= F_DynamicBicycleModel(v, x, u, L, dt, mass, Iz,Cf,Cr)
    % 離散時間動的二輪車モデル
    %
    % 入力:
    %   x      - 現在の状態ベクトル [x; y; phi; vx; vy; omega]
    %   u      - 制御入力 [delta; ax]
    %   dt     - 離散時間ステップ
    %   params - 車両パラメータ構造体
    %            params.Cf: 前輪コーナリング剛性
    %            params.Cr: 後輪コーナリング剛性
    %            params.Lf: 前輪から重心までの距離
    %            params.Lr: 後輪から重心までの距離
    %            params.m: 車両の質量
    %            params.Iz: ヨー慣性モーメント

    % 状態の展開
    x_pos = x(1); % 車両のX位置
    y_pos = x(2); % 車両のY位置
    phi   = x(3); % ヨー角
    beta = atan(0.5*tan(u));
    vx    = x(4); % 車両前方速度
    vy    = x(5); % 車両横方向速度
    omega = x(6); % ヨーレート

    % 入力の展開
    delta = u; % 操舵角
    ax    = 0; % 前方加速度

    % パラメータの展開
    % Cf = 9e3; % 前輪コーナリング剛性
    % Cr = 10e3; % 後輪コーナリング剛性
    Lf = L/2; % 前輪から重心までの距離
    Lr = L/2; % 後輪から重心までの距離
    m  = mass;  % 質量
    % Iz = Iz; % ヨー慣性モーメント

    % タイヤ力の計算
    Fyf = -Cf * ((vy + Lf * omega)/vx - delta); % 前輪横力
    % Fyf = -Cf * ((beta + Lf * omega)/vx - delta); % 前輪横力
    Fyr = -Cr * (vy - Lr * omega)/vx;             % 後輪横力
    % Fyr = -Cr * (beta-(Lr * omega)/vx);             % 後輪横力
    ay = (Fyf * cos(delta) / m + Fyr / m - vx * omega);
    
    dot_beta = (Lf * Fyf - Lr * Fyr) / Iz;  % 横滑り角の変化率
    % ay = (vx / m) * (dot_beta + (Lf / vx) * Fyf - (Lr / vx) * Fyr);  % 横加速度

    % 状態更新式
    dxdt = [(vx * cos(phi) - vy * sin(phi)); % x更新
        (vx * sin(phi) + vy * cos(phi)); % y更新
        omega;                             % ヨー角更新
        (ax - (Fyf * sin(delta) / m )+ vy * omega); % vx更新
        (Fyf * cos(delta) / m + Fyr / m - vx * omega); % vy更新
        (1 / Iz) * (Lf * Fyf * cos(delta) - Lr * Fyr)];   % omega更新
end

