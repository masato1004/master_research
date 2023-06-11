monte_iter = 10000;

trend_error_var = ;
height_error_var = ;

%% Dfine simulation condition with boolean
% draw an animation or not
animation = false;
prev_anim = true;

% add noise or not
high_freq_noise = true;
low_freq_noise  = true;

% spatial smoothing method
wa  = false;
lpf = false;

% control method
passive = false;
LQR     = false;
rprev   = false;
LQR_rprev       = false;
fprev_rprev     = false;
LQR_fprev_rprev = true;

% road profile
sensing = false;
paper   = true;
sin_wave = false;
step    = false;
manhole = false;
jari    = false;

%% simulation parameter
T   = 10;          % [s]       total simulation time
dt  = 0.0001;     % [s]       delta time
tc  = 0.001;      % control cycle
fs  = 50;         % sampling frequence
ts  = 1/fs;       % sampling cycle
cc  = 1;          % control cycle counter
sc  = 1;          % sampling cycle counter
hsd = 0.02/3;     % standard deviation of additional high freq noise
% lsd = @(x) 4.185*(x.^2)/1000;     % standard deviation of additional low freq noise
lsd = @(a,b,x) x.*a + b;     % standard deviation of additional low freq noise


%% conbine settings
noise_name = ["_high-freq-noise_" "_low-freq-noise_"];
noise_logi = [high_freq_noise low_freq_noise];
if width(noise_name(noise_logi)) == 2
    added_noise = "_high-low-freq-noise_";
elseif width(noise_name(noise_logi)) == 0
    added_noise = "";
else
    added_noise= noise_name(noise_logi)
end

smoothing_name = ["_wa_" "_lpf_"];
smoothing_logi = [wa lpf];
if width(smoothing_name(smoothing_logi)) == 2
    smoothing_method = "_wa-lpf_"
elseif width(smoothing_name(smoothing_logi)) == 0
    smoothing_method = "";
else
    smoothing_method= smoothing_name(smoothing_logi)
end

ctrl_names = ["_passive_","_LQR_","_rprev_","_LQR_rprev_","_fprev_rprev_","_LQR_fprev_rprev_"];
logi_ctrl = [passive, LQR, rprev, LQR_rprev, fprev_rprev, LQR_fprev_rprev];
control = ctrl_names(logi_ctrl)

shape_names = ["_sensing2_","_paper_","_sin_","_step_","_manhole_","_jari_"];
logi_shape = [sensing, paper, sin_wave, step, manhole, jari];
shape = shape_names(logi_shape)

%% LPF settings
if lpf
    filt_des = designfilt("lowpassiir",FilterOrder=6, HalfPowerFrequency=0.006,DesignMethod="butter",SampleRate=1);
end