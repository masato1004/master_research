%% branch: semi-active-proto
close all;
clear;
clc;
branch = "_semi-active-proto_";

%% Vehicle parameter
m_b = 1000;     % [kg]      body mass
m_w = 30;       % [kg]      wheel mass
k_sf = 33000;   % [N/m]     front spring stiffness
k_sr = 37000;   % [N/m]     rear spring stiffness
% k_sf = 13000;   % [N/m]     front spring stiffness
% k_sr = 17000;   % [N/m]     rear spring stiffness
k_w = 270000;   % [N/m]     tire stiffness
% k_w = 60000;   % [N/m]     tire stiffness
c_sf = 3300;    % [N/(m/s)] front damping
c_sr = 3250;    % [N/(m/s)] rear damping
c_w = 1000;       % [N/(m/s)] tire damping
% c_w = 2000;       % [N/(m/s)] tire damping
I_b = 750;      % [kgm^2]   inertia moment
L_f = 1.4;      % [m]       front length
L_r = 1.4;      % [m]       rear length

cam_fwheel_dis = 0.895;

Vkm_h = 50;     % [km/h]    driving velocity
Vkm_m=Vkm_h/60;
V=Vkm_m*1000/60;% [m/s]

%% simulation parameter
T = 10;         % [s]       total simulation time
dt = 0.0001;    % [s]       delta time
tc = 0.001;      % control cycle
fs = 50;         % sampling frequence
ts = 1/fs;       % sampling cycle

animation = true; % draw an animation or not

% control method
passive = false;
LQR = true;
rprev = false;
LQR_rprev = false;
fprev_rprev = false;
LQR_fprev_rprev = false;

ctrl_names = ["_passive_","_LQR_","_rprev_","_LQR_rprev_","_fprev_rprev_","_LQR_fprev_rprev_"];
logi_ctrl = [passive, LQR, rprev, LQR_rprev, fprev_rprev, LQR_fprev_rprev];
control = ctrl_names(logi_ctrl)

%% road profile

sensing = false;
paper = true;
sin_wave = false;
step = false;
manhole = false;
jari = false;

TL = 0:dt:T;                                                                  % time list ([s])
dis = 0:T*V/(T/dt):T*V;                                                       % distance time line ([m])
start_disturbance = 9;                                                        % the start distance of disturbance ([m])
max_z0 = 0.080;                                                                % [m] max road displacement
const = 6;                                                                    % amplitude
max_distance = 300;                                                           % [m] driving mileage
dis_total = 0:max_distance/(T/dt):max_distance;                               % distance list for road profile ([m])
dis_total_f = 0:max_distance/(T/dt):max_distance-start_disturbance;                           % distance list for front road profile ([m])
dis_total_r = 0:max_distance/(T/dt):max_distance-start_disturbance-(L_f+L_r);                 % distance list for rear road profile ([m])

% sensing
if sensing
    true_datas = load("line_neo4.mat");
%     true_datas = load("line_neo4_as_truedata.mat");
    true_profile = true_datas.line_neo4;
    [~,ia,~]=unique(true_profile(:,1));
    true_profile = true_profile(ia,:);
    road_total_f = true_profile;
    road_total_r = true_profile; road_total_r(:,1) = road_total_r(:,1)+(L_f+L_r);
    [~,ia,~]=unique(road_total_r(:,1));
    road_total_r = road_total_r(ia,:,:);
    r_p_f = interp1(road_total_f(:,1)',road_total_f(:,2)',dis,'linear');                                   % front road profile
    r_p_r = interp1(road_total_r(:,1)',road_total_r(:,2)',dis,'linear');                                   % rear road profile
    r_p_f = lowpass(r_p_f,0.5,1/dt);
    r_p_r = lowpass(r_p_r,0.5,1/dt);
    frequency = 0; max_z0 = 0; ld = 4;
    shape = "_sensing2_"

% same as paper
elseif paper
    Td = 2/V;
    ld = Td*V;
    dis_length = 0:max_distance/(T/dt):ld;
    road_total_f = [zeros(1,int32(T*start_disturbance/(dt*max_distance))), (max_z0/2)*(1-cos(2*pi*dis_length/ld)), zeros(size(dt:max_distance/(T/dt):max_distance-(start_disturbance+ld)))];  % converting front disturbance and buffer ([m])
    road_total_r = [zeros(1,int32(T*(start_disturbance+L_f+L_r)/(dt*max_distance))), (max_z0/2)*(1-cos(2*pi*dis_length/ld)), zeros(1,width(dis_total)-width([zeros(1,int32(T*(start_disturbance+L_f+L_r)/(dt*max_distance))), (max_z0/2)*(1-cos(2*pi*dis_length/ld))]))];  % converting rear disturbance and buffer ([m])
    % road_total_r = [zeros(1,int32(T*(3+L_f+L_r)/(dt*max_distance))), (max_z0/2)*(1-cos(2*pi*dis_length/ld)), zeros(size(0:max_distance/(T/dt):max_distance-(3+ld)-(L_f+L_r)))];  % converting rear disturbance and buffer ([m])
    r_p_f = makima(dis_total,road_total_f,dis);                                   % front road profile
    r_p_r = makima(dis_total,road_total_r,dis);                                   % rear road profile
    frequency = 0;
    shape = "_paper_"

% sin wave
elseif sin_wave
    disturbance_total_f = max_z0*0.5+max_z0*sin(const*dis_total_f-pi/2)/2;        % road disturbance for front wheel ([m])
    disturbance_total_r = max_z0*0.5+max_z0*sin(const*dis_total_r-pi/2)/2;        % road disturbance for rear wheel ([m])
    road_total_f = [zeros(1,int32(T*3/(dt*max_distance))), disturbance_total_f];  % converting front disturbance and buffer ([m])
    road_total_r = [zeros(1,int32(T*(3+L_f+L_r)/(dt*max_distance))), disturbance_total_r];  % converting rear disturbance and buffer ([m])
    
    r_p_f = makima(dis_total,road_total_f,dis);                                   % front road profile
    r_p_r = makima(dis_total,road_total_r,dis);                                   % rear road profile
    ld = 0; frequency = (const/(2*pi))*V
    shape = "_sin_"

% step
elseif step
    road_total_f = [zeros(1,int32(T*3/(dt*max_distance))), max_z0*ones(size(dis_total_f))];  % converting front disturbance and buffer ([m])
    road_total_r = [zeros(1,int32(T*(3+L_f+L_r)/(dt*max_distance))), max_z0*ones(size(dis_total_r))];  % converting rear disturbance and buffer ([m])
    r_p_f = makima(dis_total,road_total_f,dis);                                   % front road profile
    r_p_r = makima(dis_total,road_total_r,dis);                                   % rear road profile
    frequency = 0; ld = 0;
    shape = "_step_"

% manhole
elseif manhole
    manhole_L = 0.6
    m_length = 0:max_distance/(T/dt):manhole_L;
    road_total_f = [zeros(1,int32(T*3/(dt*max_distance))), max_z0*ones(size(m_length)), zeros(size(dt:max_distance/(T/dt):max_distance-(3+manhole_L)))];  % converting front disturbance and buffer ([m])
    road_total_r = [zeros(1,int32(T*(3+L_f+L_r)/(dt*max_distance))), max_z0*ones(size(m_length)), zeros(size(dt:max_distance/(T/dt):max_distance-(3+manhole_L)-(L_f+L_r)))];  % converting rear disturbance and buffer ([m])
    r_p_f = makima(dis_total,road_total_f,dis);                                   % front road profile
    r_p_r = makima(dis_total,road_total_r,dis);                                   % rear road profile
    frequency = 0; ld = 0;
    shape = "_manhole_"

% jari
elseif jari
    jari_data = csvread("jari.csv",2,0);
    jari_total = jari_data(:,1);
    road_total = jari_data(:,2)-jari_data(1,2);
    disturbance_total_f = makima(jari_total,road_total,0:max_distance/(T/dt):max_distance-3);
    disturbance_total_r = makima(jari_total,road_total,0:max_distance/(T/dt):max_distance-3-(L_f+L_r));
    road_total_f = [zeros(1,int32(T*3/(dt*max_distance))), disturbance_total_f];  % converting front disturbance and buffer ([m])
    road_total_r = [zeros(1,int32(T*(3+L_f+L_r)/(dt*max_distance))+1), disturbance_total_r];  % converting rear disturbance and buffer ([m])
    r_p_f = makima(dis_total,road_total_f,dis);                                   % front road profile
    r_p_r = makima(dis_total,road_total_r,dis);                                   % rear road profile
    frequency = 0; max_z0 = 0; ld = 0;
    shape = "_jari_"
end

r_p = [
    r_p_f;
    r_p_r;
    gradient(r_p_f)./dt;
    gradient(r_p_r)./dt
    ];
r_p_prev = [
    dis;
    r_p_f;
    r_p_r;
    gradient(r_p_f)./dt;
    gradient(r_p_r)./dt
    ];

%% Initial condition
c = width(TL);
z_b = zeros(1,c);        % [m]       body displacement
z_wf = zeros(1,c);       % [m]       front wheel displacement
z_wr = zeros(1,c);       % [m]       rear wheel displacement
theta = zeros(1,c);      % [rad]     body pitch angle
z_bDot = zeros(1,c);     % [m/s]     body heave velocity
z_wfDot = zeros(1,c);    % [m/s]     front wheel heave velocity
z_wrDot = zeros(1,c);    % [m/s]     rear wheel heave velocity
theta_Dot = zeros(1,c);  % [rad/s]   body pitch angular velocity

states = [
    z_b;
    z_wf;
    z_wr;
    theta;
    z_bDot;
    z_wfDot;
    z_wrDot;
    theta_Dot
    ];                    % states vector

z_b2Dot = zeros(1,c);     % [m/s^2]   body heave acceleration
theta_2Dot = zeros(1,c);  % [rad/s^2] body pitch angular accleration

accelerations = [
    z_b2Dot;
    theta_2Dot
    ];                    % acceleration vector

%% definition of matrices
% x(k+1) = Ax(k) + Bu(k) + Gw(k)
Ap = [
    0, 0, 0, 0, 1, 0, 0, 0;
    0, 0, 0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 0, 0, 1;
    (-k_sf-k_sr)/m_b, k_sf/m_b, k_sr/m_b, (-k_sf*L_f+k_sr*L_r)/m_b, (-c_sf-c_sr)/m_b, c_sf/m_b, c_sr/m_b, (-c_sf*L_f+c_sr*L_r)/m_b;
    k_sf/m_w, (-k_sf-k_w)/m_w, 0, (k_sf*L_f)/m_w, c_sf/m_w, (-c_sf-c_w)/m_w, 0, (c_sf*L_f)/m_w;
    k_sr/m_w, 0, (-k_sr-k_w)/m_w, (-k_sr *L_r)/m_w, c_sr/m_w, 0, (-c_sr-c_w)/m_w, (-c_sr*L_r)/m_w;
    (-k_sf*L_f+k_sr*L_r)/I_b, (k_sf*L_f)/I_b, (-k_sr*L_r)/I_b, (-k_sf*L_f^2)/I_b, (-c_sf*L_f+c_sr*L_r)/I_b, (c_sf*L_f)/I_b, (-c_sr*L_r)/I_b, ((-c_sf*L_f^2)-(c_sr*L_r^2))/I_b
    ];

Bp = [
    0, 0;
    0, 0;
    0, 0;
    0, 0;
    1/m_b, 1/m_b;
    -1/m_w, 0;
    0, -1/m_w;
    L_f/I_b, -L_r/I_b
    ];

Ep = [
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    k_w/m_w, 0, c_w/m_w, 0;
    0, k_w/m_w, 0, c_w/m_w;
    0, 0, 0, 0;
    ];

C = [
    1, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 1, 0, 0, 0, 0;
    ];

% discretization

disc_func = @(tau,Mat) (-Ap\expm(Ap.*(tc-tau)))*Mat;

A = expm(Ap.*tc);
B = disc_func(tc,Bp) - disc_func(0,Bp);
E = disc_func(tc,Ep) - disc_func(0,Ep);

CA = C*A;
CB = C*B;
CE = C*E;

% dx(k) = x(k) - x(k-1)
% X(k) = phi*dx(k) + G*du(k) + Gd*dw(k)
phi = [
    eye(height(CA)), -CA;
    zeros(width(CA),height(CA)), A
    ];

G = [
    -CB;
    B
    ];

Gd = [
    -CE;
    E
    ];

%% control design
control_TL = 0:tc:T;                       % time line for control results
c_ctrl = width(control_TL);
u = zeros(2,c_ctrl);                            % input vector
du= zeros(2,c_ctrl);                            % input-differencial vector
e = zeros(height(C),c_ctrl);                    % output-error vector
dx= zeros(height(states),c_ctrl);               % states-differencial vector

X = [
    e;
    dx
    ];        % new states vector

% LQR
% Q = diag([1,1,1e+09,1,1,1,1,1,1,1]);       % dx_heave
% Q = diag([1, 100, 1, 1, 1, 1000]);       % dx_pitch
% Q = diag([100,100,1,1,1,1,1,1,1,1]);       % e_pitch_heave
% Q = diag([1, 1e+10, 1, 1, 1, 1, 1, 1, 1, 1]);       % all
% Q = diag([1, 1e+09, 1, 1, 1, 1e+10, 1, 1, 1, 1e+11]);       % e_pitch
Q = diag([1e-06, 1e+05, 1e-03, 1e-03, 1e-03, 1e+06, 1e-03, 1e-03, 1e-03, 1e+06]);       % all_pitch
H = diag([1e-03,1e-03]);
% [P, K, ~] = care(phi, G, Q, H);   % u = -Kx
[P, K, ~] = idare(phi, G, Q, H, [], []);   % u = -Kx
F=-K;                                      % u = Fx

% preview
M = 43;                        % preview step
FDW = zeros(height(u), c_ctrl);
Fd = @(j) -(H + G'*P*G)\G'*(((phi+G*F)')^(j))*P*Gd;       % function for Fd(j)
Fdj = zeros(height(u), height(r_p), M+1);                 % feedforward gain vector list
for j = 0:M
    Fdj(:,:,j+1)=Fd(j);
end

dis_dw = 0:(T*V)/(T/tc):T*V+3;

if ~sensing
    r_p_f_dw = makima(dis_total,road_total_f,dis_dw);
    r_p_r_dw = makima(dis_total,road_total_r,dis_dw);
else
    r_p_f_dw = interp1(road_total_f(:,1)',road_total_f(:,2)'+1.46,dis_dw,'linear');
    r_p_r_dw = interp1(road_total_r(:,1)',road_total_r(:,2)'+1.46,dis_dw,'linear');
end

w_r = [
    zeros(size(r_p_r_dw));
    r_p_r_dw;
    zeros(size(r_p_r_dw));
    gradient(r_p_r_dw)./tc
    ];
dw_r = [0, 0, 0, 0; diff(w_r')]';         % preview road profile differencial

% w_fr = [
%     r_p_f_dw;
%     r_p_r_dw;
%     zeros(size(r_p_r_dw));
%     zeros(size(r_p_r_dw))
%     ];
w_fr = [
    r_p_f_dw;
    r_p_r_dw;
    gradient(r_p_f_dw)./tc;
    gradient(r_p_r_dw)./tc
    ];
dw_fr = [0, 0, 0, 0; diff(w_fr')]';         % preview road profile differencial


%% accelerate functions (without controller)
% body acceleration
body_acc=@(zb,zbdot,zwf,zwr,ang,zwfdot,zwrdot,angdot,uf,ur) (-k_sf*(L_f*ang+zb-zwf)-k_sr*(-L_r*ang+zb-zwr)-c_sf*(L_f*angdot+zbdot-zwfdot)-c_sr*(-L_r*angdot+zbdot-zwrdot)+uf+ur)/m_b;

% front wheel acceleration
wf_acc = @(zwf,zwfdot,zb,ang,zbdot,angdot,rp,rpdot,uf) (k_sf*(L_f*ang+zb-zwf)-k_w*(zwf-rp)+c_sf*(L_f*angdot+zbdot-zwfdot)-c_w*(zwfdot-rpdot)-uf)/m_w;

% rear wheel acceleration
wr_acc = @(zwr,zwrdot,zb,ang,zbdot,angdot,rp,rpdot,ur) (k_sr*(-L_r*ang+zb-zwr)-k_w*(zwr-rp)+c_sr*(-L_r*angdot+zbdot-zwrdot)-c_w*(zwrdot-rpdot)-ur)/m_w;

% angular acceleration
ang_acc =@(zb,zbdot,zwf,zwr,ang,zwfdot,zwrdot,angdot,uf,ur) (-(k_sf*(L_f*ang+zb-zwf)+c_sf*(L_f*angdot+zbdot-zwfdot))*L_f+(k_sr*(-L_r*ang+zb-zwr)+c_sr*(-L_r*angdot+zbdot-zwrdot))*L_r+(L_f*uf-L_r*ur))/I_b;

% check preview road
% videoname = "videos/"+branch+"/"+"_LQR_fprev_rprev_"+"/"+shape+"/PreviewedRoad--"+"-v-"+Vkm_h+"-shape-"+shape+"-hieght-"+max_z0+"-Ld-"+ld;
% video = VideoWriter(videoname,'MPEG-4');
% video.FrameRate = (1/tc)/100;
% open(video);



%% ========================simulation========================= %%

% LOADING
cc = 1;
sc = 1;
prev_start = 5.06;
prev_end = 7;
current_dis = r_p_prev(1,1);
if sensing
    load_dir = "preview_datas";
    listing = dir(load_dir+"/*.mat");
    file = load(load_dir + "/" + listing(2).name);
    vertices = file.vertices;
    
    mm_ratio = 0.35;
    mm_range = 5*ts;
    
    filt_des = designfilt("lowpassiir",FilterOrder=6, HalfPowerFrequency=0.006,DesignMethod="butter",SampleRate=1);
    
    
    wf_local = previewing(vertices);
    data_end = wf_local(2,round(end/2));
    wf_global = wf_local; last_minimum = 1.9;
else
    listing = [];
    wf_local = [
        prev_start:tc*V:prev_end;
        makima(r_p_prev(1,:),zeros(size(r_p_prev(1,:))),current_dis+prev_start:tc*V:current_dis+prev_end)
        ];
    wf_global = wf_local; last_minimum = wf_global(1,1);
end
wf_grad = [
    wf_global(1,:)./V;
    wf_global(2,:);
    gradient(wf_global(2,:))./(gradient(wf_global(1,:))./V)
    ];
dw_prev = zeros(4,M+1);

% % Video settings
% check = figure('name',"preview road",'Position', [500-20 500-20 1000 380]);
% check_plot = plot(wf_global(1,:),wf_global(2,:),"LineWidth",2,"Color","#ff0000");
% hold on;
% check_plot0 = plot(wf_global(1,:),wf_global(2,:),"LineWidth",2,"Color","#0000ff");
% check_plot2 = plot(wf_global(1,:),wf_global(2,:),"LineWidth",2,"Color","#00ff00");
% xline(0,'--r',"Front Wheel");
% xline(prev_start,'--');
% xline(prev_end,'--');
% % xline(prev_start-mm_range*V,'--');
% mov_line = xline(prev_end,'--');
% % xlim([-0.05,0.1]);
% xlim([-1,10]);
% ylim([-0.1,0.1]);
% grid on;
% % xlabel("Preview Time [s]");
% xlabel("Local Distance from Front Wheel [m]");
% ylabel("Previewed Road Displacement [m]");
% % time count
% txdata = round(TL(1,1),2);
% str = {"Time [s]",txdata};
% time_text = text(0, 0.01, str);
% time_text.FontSize = 20;
% fontname(check,"Times New Roman");
% fontsize(check,16,"points");

% LOOP
for i=1:c-1

    % make road preview profile
    if mod(i+(ts/dt-1), ts/dt) == 0
        current_dis = r_p_prev(1,i);
        % load road profile
        if sensing
            if sc <= height(listing)
                file = load(load_dir + "/" + listing(sc).name);
                vertices = file.vertices;
                wf_local = previewing(vertices);
                % wf_local(isnan(wf_local(2,:))) = data_end;
                wf_local = rmmissing(wf_local,2);
                data_end = wf_local(2,end);
                % wf_local(2,:) = movmean(wf_local(2,:),width(wf_local(2,wf_local(1,:))));  % move mean with mm_ratio
                % wf_local(2,:) = movmean(wf_local(2,:),round(mm_ratio*width(wf_local)));  % move mean with mm_ratio
                % display("HERE"+sc);
            else
                wf_local = [
                    prev_start:tc*V:prev_end;
                    interp1(r_p_prev(1,:),data_end*ones(size(r_p_prev(1,:))),current_dis+prev_start:tc*V:current_dis+prev_end,'linear')
                    ];
                    
                % display("HERE"+sc);
            end
        else
            wf_local = [
                prev_start:tc*V:prev_end;
                makima(r_p_prev(1,:),r_p_prev(2,:),current_dis+prev_start:tc*V:current_dis+prev_end)
                ];
        end

        % set(check_plot0, "XData", wf_local(1,:), "YData", wf_local(2,:));
        % drawnow;

        Lpass = wf_local(1,1) - last_minimum;
        last_minimum = wf_local(1,1);
        Ltotal = wf_local(1,end) - wf_local(1,1);
        eta = Lpass/Ltotal;
        eta_1 = 0.5*(1-eta);
        eta_2 = 0.5*(1+eta);

        [~,ia,~]=unique(wf_global(1,:));
        wf_global = wf_global(:,ia);

        % wf_global = [wf_global, wf_local];
        % [~,ind] = sort(wf_global(1,:));
        % wf_global=wf_global(:,ind);


        % mov_num = mm_ratio*width(wf_local);

        % notnan_wfg = rmmissing(wf_global,2);
        % notnan_wfg(1,:) = notnan_wfg(1,:)./V;

        if sensing
            % WA + filtfilt
            [~,ia,~]=unique(notnan_wfg(1,:));
            notnan_wfg = notnan_wfg(:,ia);
            grad_time = notnan_wfg(1,1):tc:notnan_wfg(1,end);
            % disp(size(grad_time));
            grad_data1 = filtfilt(filt_des,double(interp1(notnan_wfg(1,:), notnan_wfg(2,:), grad_time, "linear")));
            grad_data2 = gradient(grad_data1)./(gradient(grad_time));
            wf_grad = [
                grad_time;
                grad_data1;
                grad_data2
                ];
        else
            % WA
            wf_global = [
                wf_global(1, wf_global(1,:)<wf_local(1,1) & wf_global(1,:)>-0.1), wf_local(1,:);
                wf_global(2, wf_global(1,:)<wf_local(1,1) & wf_global(1,:)>-0.1), makima(wf_global(1,:), wf_global(2,:), wf_local(1, wf_local(1,:)<=wf_global(1,end))).*eta_1 + wf_local(2, wf_local(1,:)<=wf_global(1,end)).*eta_2, wf_local(2, wf_local(1,:)>wf_global(1,end))
                ];
            wf_grad = [
                wf_global(1,:)./V;
                wf_global(2,:);
                gradient(wf_global(2,:))./(gradient(wf_global(1,:))./V)
                ];
        end
        sc = sc + 1;
    end
    % set(check_plot, "XData", wf_grad(1,:), "YData", wf_grad(3,:));
    % drawnow;

    % load current states
    x = states(1:4,i);
    v = states(5:8,i);

    % load road profile
    d = r_p(:,i);
    
    % load current input
    u_in = u(:,cc);

    % states-update with Runge-Kutta
    % Runge kutta
    states(:,i+1) = runge(x, v, u_in, d, Ap, Bp, Ep, dt);
    accelerations(:,i+1) = [states(5,i+1);states(8,i+1)]./dt;

    % find appropriate next input
    if mod(i-1, (tc/dt)) == 0 && i ~= 1 && ~passive
        cc = (i-1)/(tc/dt)+1;                   % list slice
        [~,ia,~]=unique(wf_grad(1,:));
        wf_grad = wf_grad(:,ia);
        dw_prev = [
            interp1(wf_grad(1,:),[0,diff(wf_grad(2,:))],[0:M].*tc,'linear');
            dw_r(2, [0:M]+cc);
            interp1(wf_grad(1,:),[0,diff(wf_grad(3,:))],[0:M].*tc,'linear');
            dw_r(4, [0:M]+cc);
            ];

        % calculate new states
        e(:,cc) = -C*states(:,i);                        % e=r-y
        dx(:,cc) = states(:,i) - states(:,i-(tc/dt));    % dx(k)=x(k)-x(k-1)
        X(1:height(e),cc) = e(:,cc);                     % X=[e;dx]
        X(height(e)+1:end,cc) = dx(:,cc);

        % calculate input
        du(:,cc+1) = next_input(logi_ctrl,M,F,X(:,cc),FDW(:,cc),Fdj,wf_grad(1,1),dw_r(:, cc:cc+M),dw_prev,dw_fr(:, cc:cc+M));
        fv_diff = states(1,i)-states(2,i);
        fr_diff = states(1,i)-states(3,i);

        % TEST pre semi-active-suspension
        if sign(fv_diff)+sign(du(1,cc+1)) == 0
            du(1,cc+1) = 0;
        end
        if sign(fr_diff)+sign(du(2,cc+1)) == 0
            du(2,cc+1) = 0;
        end

        if cc ~= 1
            u(:, cc+1) = u(:, cc) + du(:, cc+1);
        else
            u(:, cc+1) = du(:, cc+1);
        end
        wf_global(1,:) = wf_global(1,:) - tc*V; last_minimum = last_minimum - tc*V;
        wf_grad(1,:) = wf_grad(1,:) - tc;
        % set(check_plot0, "XData", wf_local(1,:), "YData", wf_local(2,:));
        % set(check_plot, "XData", wf_global(1,:), "YData", wf_global(2,:));
        % set(check_plot2, "XData", wf_grad(1,:).*V, "YData", wf_grad(2,:));
        % display(cc);
        
        % txdata = round(TL(1,i),2);
        % str = {"Time [s]",txdata};
        % time_text.String = str;
        % % time_text.Position = [0.05, 0.05];
        % time_text.Position = [8, 0.05];
        % drawnow;
        % frame = getframe(check);
        % writeVideo(video,frame);
    end
end
% close(video);

pitch_integral = trapz(TL(1:end-width(states(4,isnan(states(4,:))))),abs(rad2deg(double(states(4,1:end-width(states(4,isnan(states(4,:)))))))))
pitchacc_integral = trapz(TL(1:end-width(states(8,isnan(states(8,:))))),abs(rad2deg(double(states(8,1:end-width(states(8,isnan(states(8,:)))))))))

%% =============================================================== %%
%                          Drawing figures                          %
% ================================================================= %
states_name = [
    "Body_Heave_Displacement", "Time [s]", "Body Heave Displacement [m]";
    "Front_Wheel_Heave_Displacement", "Time [s]", "Front Wheel Heave Displacement [m]";
    "Rear_Wheel_Heave_Displacement", "Time [s]", "Rear Wheel Heave Displacement [m]";
    "Body_Pitch_Angle", "Time [s]", "Body Pitch Angle [deg]";
    "Body_Heave_Velocity", "Time [s]", "Body Heave Velocity [m/s]";
    "Front_Wheel_Heave_Velocity", "Time [s]", "Front Wheel Heave Velocity [m/s]";
    "Rear_Wheel_Heave_Velocity", "Time [s]", "Rear Wheel Heave Velocity [m/s]";
    "Body_Pitch_Angular_Velocity", "Time [s]", "Body Pitch Angular Velocity [deg/s]"
    ];

r_fig = figure('name',"Road-profile: Frequency "+frequency+" Hz",'Position', [600 200 600 190]);
plot(dis,r_p_f,"LineWidth",1,"Color","#0000ff");
ylabel("Displacement [m]");
xlabel("Distance Traveled [m]");
% ylim([-0.01,0.1])
% ylim([-0.03,0.04])
fontname(r_fig,"Times New Roman");
fontsize(r_fig,10.5,"points");
grid on;

figfolder = "-QH-"+"all_pitch"+"-v-"+Vkm_h+"-shape-"+shape+"-hieght-"+max_z0+"-Ld-"+ld+"-freq-"+frequency+"-ctrlCycle-"+tc;
made_successfully = folder_maker(branch,control,shape,figfolder);

saveas(r_fig,"figs/"+branch+"/"+control+"/"+shape+"/"+figfolder+"/Road_Profile.fig");
saveas(r_fig,"jpgs/"+branch+"/"+control+"/"+shape+"/"+figfolder+"/Road_Profile.jpg");

for i=1:height(states)
    if i==4 || i==8
        drawer(TL,states(i,:)*(180/pi),states_name(i,:),i,figfolder,branch,control,shape);
    else
        drawer(TL,states(i,:),states_name(i,:),i,figfolder,branch,control,shape);
    end
end


%% additional draw
drawer(TL,accelerations(1,:),["Body_Heave_Acceleration", "Time [s]", "Body Heave Acceleration [m/s^2]"],9,figfolder,branch,control,shape);
drawer(TL,accelerations(2,:)*(180/pi),["Body_Pitch_Angular_Acceleration", "Time [s]", "Body Pitch Angular Acceleration [deg/s^2]"],10,figfolder,branch,control,shape);

% drawer(control_TL,u(1,:),["Front_Wheel_Actuator_Force", "Time [s]", "Front Wheel Actuator Force [N]"],11,figfolder,shape);
% drawer(control_TL,u(2,:),["Rear_Wheel_Actuator_Force", "Time [s]", "Rear Wheel Actuator Force [N]"],12,figfolder,shape);
fig = figure('name',"Actuator_Force",'Position', [500+20*11 500-20*11 600 190]);
plot(control_TL,u(1,:),"LineWidth",2,"Color","#0000ff");
hold on;
plot(control_TL,u(2,:),"LineWidth",2,"Color","#0000ff","LineStyle","--");
grid on;
xlim([0,3]);
xlabel("Time [s]");
ylabel("Wheel Actuator Force [N]");
legend("{\it f_{af}} : Front Wheel", "{\it f_{ar}} : Rear Wheel");
fontname(fig,"Times New Roman");
fontsize(fig,10.5,"points");
saveas(fig,"figs/"+branch+"/"+control+"/"+shape+"/"+figfolder+"/Actuator_Force.fig");
saveas(fig,"jpgs/"+branch+"/"+control+"/"+shape+"/"+figfolder+"/Actuator_Force.jpg");

% compare the timings of rear road profile and actuator input
fig = figure('name',"Actuator_Force_and_Road",'Position', [500+20 500-20 600 190]);
xlim([0,3]);
yyaxis left
plot(control_TL,u(1,:),"LineWidth",2,"Color","#0000ff");
hold on;
plot(control_TL,u(2,:),"LineWidth",2,"Color","#0000ff");
grid on;
xlabel("Time [s]");
ylabel("Wheel Actuator Force [N]");
ylim([-4000,4000]);
ax = gca;
ax.YAxis(1).Color = [0 0 1];
yyaxis right
ylim([-0.09,0.09]);
plot(TL,r_p_f,"LineWidth",2,"Color","#ff0000");
plot(TL,r_p_r,"LineWidth",2,"Color","#ff0000");
ylabel("Road Displacement [m]");
legend("{\it f_{af}} : Front Actuator", "{\it f_{ar}} : Rear Actuator", "{\it z_{0f}} : Front Road Displacement", "{\it z_{0r}} : Rear Road Displacement");
ax = gca;
ax.YAxis(2).Color = [1 0 0];
fontname(fig,"Times New Roman");
fontsize(fig,10.5,"points");
grid on;
saveas(fig,"figs/"+branch+"/"+control+"/"+shape+"/"+figfolder+"/Actuator_Force_and_Road.fig");
saveas(fig,"jpgs/"+branch+"/"+control+"/"+shape+"/"+figfolder+"/Actuator_Force_and_Road.jpg");

if animation
    close all;
    clear frames;
    run("half_model_animation.m")
end